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
    python3 k1_camera_relay.py [port] [jpeg_quality]

The port is unauthenticated and read-only (camera frames only); keep the robot on a trusted network.
"""
import socket
import struct
import sys
import threading

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

TOPICS = ("/boostercamera/head/rgb", "/boostercamera/head/right/rgb")  # index = camera id
PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 51120
QUALITY = int(sys.argv[2]) if len(sys.argv) > 2 else 80

latest = [None] * len(TOPICS)  # (sequence, packet) per camera
seq = 0
cond = threading.Condition()
clients = 0


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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
