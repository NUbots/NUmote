#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import time

import booster_robotics_sdk_python as br


def build_default_start_param(enable_face_tracking: bool = False) -> br.StartAiChatParameter:
    """
    Build a sample StartAiChatParameter.
    Adjust system_prompt / welcome_msg / TTS / ASR as needed.
    """
    # Example TTS configuration
    tts = br.TtsConfig(
        voice_type="zh_male_wennuanahu_moon_bigtts",
        ignore_bracket_text=[1, 2],
    )

    # Example LLM configuration
    llm = br.LlmConfig(
        system_prompt="You are a helpful humanoid robot assistant.",
        welcome_msg="Hello, I am the Booster robot assistant.",
        prompt_name="default",
    )

    # Example ASR configuration
    asr = br.AsrConfig(
        interrupt_speech_duration=800,
        interrupt_keywords=["stop", "hold on", "do not speak"],
    )

    p = br.StartAiChatParameter()
    p.interrupt_mode = True
    p.tts_config = tts
    p.llm_config = llm
    p.asr_config = asr
    p.enable_face_tracking = enable_face_tracking
    return p


def print_help():
    print(
        """
Available commands (AI control):
  help                  - Show this help
  start                 - Start AI Chat (face tracking disabled)
  start_ft              - Start AI Chat (face tracking enabled)
  stop                  - Stop AI Chat
  speak <text>          - Make the robot speak a sentence (bypass LLM)
  ft_on                 - Enable face tracking
  ft_off                - Disable face tracking
  raw_start             - Start AI Chat via SendApiRequest
  raw_stop              - Stop AI Chat via SendApiRequest
  raw_resp              - Send a sample request via SendApiRequestWithResponse
  quit / exit           - Exit
"""
    )


def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} networkInterface [robotName]")
        sys.exit(-1)

    net_if = sys.argv[1]
    robot_name = sys.argv[2] if len(sys.argv) > 2 else None

    # Initialize DDS
    br.ChannelFactory.Instance().Init(0, net_if)

    # Initialize AiClient
    ai = br.AiClient()
    if robot_name:
        ai.InitWithName(robot_name)
    else:
        ai.Init()

    print("AI CLI started.")
    print_help()

    while True:
        try:
            line = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nExit.")
            break

        if not line:
            continue

        # Support commands such as: speak hello world
        parts = line.split(maxsplit=1)
        cmd = parts[0]
        arg = parts[1] if len(parts) > 1 else ""

        try:
            # ---------- Help ----------
            if cmd in ("help", "?"):
                print_help()

            # ---------- Start / stop AI Chat ----------
            elif cmd == "start":
                param = build_default_start_param(enable_face_tracking=False)
                ai.StartAiChat(param)
                print("[OK] StartAiChat (face_tracking = False) sent.")

            elif cmd == "start_ft":
                param = build_default_start_param(enable_face_tracking=True)
                ai.StartAiChat(param)
                print("[OK] StartAiChat (face_tracking = True) sent.")

            elif cmd == "stop":
                ai.StopAiChat()
                print("[OK] StopAiChat sent.")

            # ---------- Speak text directly ----------
            elif cmd == "speak":
                if not arg:
                    print("Usage: speak <text>")
                    continue
                sp = br.SpeakParameter(arg)
                ai.Speak(sp)
                print(f"[OK] Speak sent: {arg}")

            # ---------- Control face tracking ----------
            elif cmd == "ft_on":
                ai.StartFaceTracking()
                print("[OK] StartFaceTracking sent.")

            elif cmd == "ft_off":
                ai.StopFaceTracking()
                print("[OK] StopFaceTracking sent.")

            # ---------- Raw SendApiRequest examples ----------
            elif cmd == "raw_start":
                # Use the same parameter as 'start', but send via raw API with JSON body
                param = build_default_start_param(enable_face_tracking=False)
                body = param.to_json_str()
                ai.SendApiRequest(br.AiApiId.kStartAiChat, body)
                print("[OK] raw SendApiRequest(kStartAiChat, json_body)")

            elif cmd == "raw_stop":
                ai.SendApiRequest(br.AiApiId.kStopAiChat, "")
                print("[OK] raw SendApiRequest(kStopAiChat, '')")

            elif cmd == "raw_resp":
                # Example: send a kSpeak request with response
                if not arg:
                    arg = "This is a test request with response."
                sp = br.SpeakParameter(arg)
                body = sp.to_json_str()
                resp = ai.SendApiRequestWithResponse(br.AiApiId.kSpeak, body)
                # Currently AiResponse is only structured in C++; print the object for inspection
                print("[OK] SendApiRequestWithResponse returned AiResponse:", resp)

            # ---------- Exit ----------
            elif cmd in ("quit", "exit"):
                print("Bye.")
                break

            else:
                print(f"Unknown command: {cmd}")
                print("Type 'help' to show available commands.")

        except Exception as e:
            # Catch pybind-wrapped runtime_error when underlying API returns non-zero
            print(f"[ERROR] Request failed: {e}")


if __name__ == "__main__":
    main()
