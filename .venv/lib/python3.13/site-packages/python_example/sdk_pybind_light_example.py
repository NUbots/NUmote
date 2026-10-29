#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import time

# 假设编译出的 .so 名字是 _core，在 python 中作为 booster_robotics_sdk_python 导入
import booster_robotics_sdk_python as br


def print_help():
    print(
        """
Available commands (Light control):
  help                      - Show this help
  set_color <r> <g> <b>     - Set LED color (RGB values 0-255). Example: set_color 255 0 0
  stop_light                - Stop/Turn off LED Light Control
  quit / exit               - Exit
"""
    )


def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} networkInterface [robotName]")
        print(f"Example: {sys.argv[0]} eno1")
        sys.exit(-1)

    net_if = sys.argv[1]
    robot_name = sys.argv[2] if len(sys.argv) > 2 else None

    # 1. 初始化 DDS 通道工厂
    print(f"Initializing ChannelFactory on {net_if}...")
    br.ChannelFactory.Instance().Init(0, net_if)

    # 2. 初始化 LightControlClient
    light_client = br.LightControlClient()
    if robot_name:
        print(f"Initializing LightControlClient for robot: {robot_name}...")
        light_client.InitWithName(robot_name)
    else:
        print("Initializing LightControlClient with default name...")
        light_client.Init()

    print("Light Control CLI started.")
    print_help()

    while True:
        try:
            line = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nExit.")
            break

        if not line:
            continue

        # 分割命令和参数
        parts = line.split()
        cmd = parts[0]
        args = parts[1:]

        try:
            # ---------- Help ----------
            if cmd in ("help", "?"):
                print_help()

            # ---------- Set LED Color ----------
            elif cmd == "set_color":
                if len(args) != 3:
                    print("Usage: set_color <r> <g> <b>")
                    print("Example: set_color 255 0 0  (Red)")
                    continue
                
                try:
                    r = int(args[0])
                    g = int(args[1])
                    b = int(args[2])
                    
                    # 简单的范围检查
                    if not (0 <= r <= 255 and 0 <= g <= 255 and 0 <= b <= 255):
                        print("[ERROR] RGB values must be between 0 and 255.")
                        continue

                    # 调用 C++ 绑定的 SetLEDLightColor(uint8_t r, g, b)
                    light_client.SetLEDLightColor(r, g, b)
                    print(f"[OK] SetLEDLightColor sent: R={r}, G={g}, B={b}")
                    
                except ValueError:
                    print("[ERROR] Arguments must be integers.")

            # ---------- Stop Light ----------
            elif cmd == "stop_light":
                # 调用 C++ 绑定的 StopLEDLightControl()
                light_client.StopLEDLightControl()
                print("[OK] StopLEDLightControl sent.")

            # ---------- Exit ----------
            elif cmd in ("quit", "exit"):
                print("Bye.")
                break

            else:
                print(f"Unknown command: {cmd}")
                print("Type 'help' to show available commands.")

        except Exception as e:
            # 捕获由 C++ 层抛出的 runtime_error (例如返回值不为0时)
            print(f"[ERROR] Request failed: {e}")


if __name__ == "__main__":
    main()