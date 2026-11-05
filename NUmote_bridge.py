#!/usr/bin/env python3
"""
Runs ON the robot: streams the left and right head cameras to NUmote over TCP.

The phone-app video stream (port 51111) only carries one camera, and DDS doesn't reach the
laptop, so this subscribes to both ROS 2 camera topics locally and forwards them as JPEGs.

Wire format, repeated per frame:  <u8 camera index (0 = left, 1 = right)> <u32 LE length> <JPEG>

Run on the robot (the env vars match the robot's own camera process, otherwise no frames arrive):
	source /opt/ros/humble/setup.bash
	export RMW_IMPLEMENTATION=rmw_fastrtps_cpp RMW_FASTRTPS_USE_QOS_FROM_XML=0 \
		FASTRTPS_DEFAULT_PROFILES_FILE=/opt/booster/config/dds_configs/single-board_configs/fastdds_profile.xml
	export NUMOTE_STRONG_KICK_TOKEN='<same-long-random-value-configured-on-the-laptop>'
	python3 k1_camera_relay.py [port] [jpeg_quality]

The camera port is unauthenticated and read-only. An optional robot-action port accepts HMAC-authenticated kick and dance commands; keep the robot on a trusted network.
"""
import hashlib
import hmac
import json
import os
import random
import socket
import struct
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

TOPICS = ("/boostercamera/head/rgb", "/boostercamera/head/right/rgb")  # index = camera id
PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 51120
QUALITY = int(sys.argv[2]) if len(sys.argv) > 2 else 80
STRONG_KICK_PORT = int(os.environ.get("NUMOTE_STRONG_KICK_PORT", "51121"))
STRONG_KICK_TOKEN = os.environ.get("NUMOTE_STRONG_KICK_TOKEN", "")
SDK_DOMAIN_ID = int(os.environ.get("NUMOTE_BOOSTER_SDK_DOMAIN_ID", "0"))
SDK_NETWORK_INTERFACE = os.environ.get("NUMOTE_BOOSTER_SDK_NETWORK_INTERFACE", "lo")
SDK_ROBOT_NAME = os.environ.get("NUMOTE_BOOSTER_SDK_ROBOT_NAME", "")
K1_DANCE_CHOICES = (
	("Arabic Dance", "kArbicDance"),
	("Michael Dance 1", "kMichaelDance1"),
	("Michael Dance 2", "kMichaelDance2"),
	("Michael Dance 3", "kMichaelDance3"),
	("Shan He Gu Ren Dance", "kShanHeGuRenDance"),
	("Gai Ge Chun Feng Dance", "kGaiGeChunFengDance"),
	("Michael Dance 1 + 2", "kMichaelDance1And2"),
)

latest = [None] * len(TOPICS)  # (sequence, packet) per camera
seq = 0
cond = threading.Condition()
clients = 0
sdk_lock = threading.Lock()
sdk_client = None
nonce_lock = threading.Lock()
used_nonces = {}


def to_bgr(msg):
	data = np.frombuffer(msg.data, dtype=np.uint8)
	if msg.encoding == "nv12":
		yuv = data.reshape(msg.height * 3 // 2, msg.step)[:, :msg.width]
		return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV12)
	channels = 3 if msg.encoding in ("rgb8", "bgr8") else 1
	img = data.reshape(msg.height, msg.step)[:, :msg.width * channels].reshape(msg.height, msg.width, channels)
	return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if msg.encoding == "rgb8" else img


def on_image(cam):
	def callback(msg):
		global seq
		if not clients:  # don't spend CPU encoding when nobody is watching
			return
		try:
			ok, jpeg = cv2.imencode(".jpg", to_bgr(msg), [cv2.IMWRITE_JPEG_QUALITY, QUALITY])
		except Exception as e:
			print(f"camera {cam}: can't encode {msg.encoding} {msg.width}x{msg.height}: {e}", flush=True)
			return
		if not ok:
			return
		packet = struct.pack("<BI", cam, len(jpeg)) + jpeg.tobytes()
		with cond:
			seq += 1
			latest[cam] = (seq, packet)
			cond.notify_all()
	return callback


def serve_client(conn, addr):
	global clients
	sent = [0] * len(TOPICS)
	with cond:
		clients += 1
	print(f"client {addr[0]} connected", flush=True)
	try:
		while True:
			with cond:
				cond.wait_for(lambda: any(f and f[0] > s for f, s in zip(latest, sent)), timeout=5)
				pending = [(cam, f) for cam, (f, s) in enumerate(zip(latest, sent)) if f and f[0] > s]
			for cam, (frame_seq, _) in pending:
				sent[cam] = frame_seq
			# a slow client only ever gets the newest frame per camera, never a backlog
			conn.sendall(b"".join(packet for _, (_, packet) in pending))
	except OSError:
		pass
	finally:
		with cond:
			clients -= 1
		conn.close()
		print(f"client {addr[0]} disconnected", flush=True)


def accept_loop(server):
	while True:
		conn, addr = server.accept()
		conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
		threading.Thread(target=serve_client, args=(conn, addr), daemon=True).start()


def authorize_strong_kick(request):
	if not STRONG_KICK_TOKEN:
		return False, "robot-action relay is not configured"
	command = request.get("command")
	if command not in ("shoot", "dance"):
		return False, "unsupported command"

	timestamp = request.get("timestamp")
	nonce = request.get("nonce")
	signature = request.get("signature")
	if type(timestamp) is not int or not isinstance(nonce, str) or not isinstance(signature, str):
		return False, "invalid request"
	if abs(time.time() - timestamp) > 30:
		return False, "request expired"

	signed_data = f"{command}\n{timestamp}\n{nonce}".encode()
	expected = hmac.new(STRONG_KICK_TOKEN.encode(), signed_data, hashlib.sha256).hexdigest()
	if not hmac.compare_digest(signature, expected):
		return False, "invalid signature"

	now = time.time()
	with nonce_lock:
		for used_nonce, used_at in tuple(used_nonces.items()):
			if now - used_at > 60:
				del used_nonces[used_nonce]
		if nonce in used_nonces:
			return False, "request already used"
		used_nonces[nonce] = now
	return True, ""


def get_sdk_client():
	global sdk_client
	if sdk_client is None:
		import booster_robotics_sdk_python as sdk

		sdk.ChannelFactory.Instance().Init(SDK_DOMAIN_ID, SDK_NETWORK_INTERFACE)
		client = sdk.B1LocoClient()
		if SDK_ROBOT_NAME:
			client.InitWithName(SDK_ROBOT_NAME)
		else:
			client.Init()
		if not client.WaitForService(5000):
			raise RuntimeError("Booster SDK locomotion service was not found")
		sdk_client = (sdk, client)
	return sdk_client


def execute_strong_kick():
	with sdk_lock:
		sdk, client = get_sdk_client()
		status = client.SendApiRequest(sdk.LocoApiId.kShoot, "")
		if status != 0:
			raise RuntimeError(f"Booster SDK rejected kShoot (status {status})")


def execute_random_dance():
	dance_name, dance_enum_name = random.choice(K1_DANCE_CHOICES)
	with sdk_lock:
		sdk, client = get_sdk_client()
		dance_id = int(getattr(sdk.WholeBodyDanceId, dance_enum_name))
		params = json.dumps({"dance_id": dance_id}, separators=(",", ":"))
		status = client.SendApiRequest(sdk.LocoApiId.kWholeBodyDance, params)
		if status != 0:
			raise RuntimeError(f"Booster SDK rejected {dance_name} (status {status})")
	return dance_name


def serve_command_client(conn, addr):
	with conn:
		conn.settimeout(8)
		try:
			with conn.makefile("rb") as stream:
				line = stream.readline(4097)
			if not line or len(line) > 4096:
				raise ValueError("invalid request size")
			request = json.loads(line)
			if not isinstance(request, dict):
				raise ValueError("invalid request")
			authorized, error = authorize_strong_kick(request)
			if not authorized:
				raise PermissionError(error)
			command = request["command"]
			if command == "shoot":
				execute_strong_kick()
				response = {"success": True}
			else:
				dance_name = execute_random_dance()
				response = {"success": True, "dance": dance_name}
			print(f"{command} accepted from {addr[0]}", flush=True)
		except Exception as e:
			response = {"success": False, "error": str(e)}
			print(f"robot-action request from {addr[0]} failed: {e}", flush=True)
		conn.sendall(json.dumps(response, separators=(",", ":")).encode() + b"\n")


def accept_command_loop(server):
	while True:
		conn, addr = server.accept()
		threading.Thread(target=serve_command_client, args=(conn, addr), daemon=True).start()


def main():
	rclpy.init()
	node = rclpy.create_node("numote_camera_relay")
	for cam, topic in enumerate(TOPICS):
		node.create_subscription(Image, topic, on_image(cam), qos_profile_sensor_data)

	server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
	server.bind(("0.0.0.0", PORT))
	server.listen()
	threading.Thread(target=accept_loop, args=(server,), daemon=True).start()
	print(f"camera relay on port {PORT}: {', '.join(TOPICS)}", flush=True)
	if STRONG_KICK_TOKEN:
		kick_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		kick_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		try:
			kick_server.bind(("0.0.0.0", STRONG_KICK_PORT))
			kick_server.listen()
			threading.Thread(target=accept_command_loop, args=(kick_server,), daemon=True).start()
			print(f"authenticated robot-action relay on port {STRONG_KICK_PORT}", flush=True)
		except OSError as e:
			kick_server.close()
			print(f"strong-kick relay unavailable: {e}", flush=True)
	else:
		print("robot-action relay disabled: set NUMOTE_STRONG_KICK_TOKEN", flush=True)
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass


if __name__ == "__main__":
	main()