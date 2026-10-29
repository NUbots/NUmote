#!/usr/bin/env python3
import sys
import time
from pathlib import Path

# 假设编译好的 .so 包名为 booster_robotics_sdk_python
import booster_robotics_sdk_python as b1


# ==========================================
# 辅助函数：简化 CustomTrainedTraj 的构建过程
# ==========================================
def create_traj_config_from_paths(traj_path: str, model_path: str) -> b1.CustomTrainedTraj:
    """
    根据 traj_path 和 model_path 创建配置对象。
    这里硬编码了默认的 PD 参数 (Kp=40, Kd=3) 和 Scale=1.0。
    如果需要不同的参数，可以在这里修改。
    """

    # 2. 构建参数对象
    params = b1.CustomModelParams([], [], [])

    # 3. 构建模型配置 (默认使用 MuJoCo 关节顺序)
    model_config = b1.CustomModel(
        model_path, 
        [params], 
        b1.JointOrder.kMuJoCo
    )

    # 4. 构建最终轨迹对象
    return b1.CustomTrainedTraj(traj_path, model_config)


def print_help():
    print(r"""
========== B1 CLI Commands ==========
# Basic
  help / ?                Show help
  quit / exit             Exit

# --- Custom Trajectory (Simplified) ---
  load_simple <t_path> <m_path>  Load trajectory using ONLY paths.
                                 (Uses default PD: Kp=40, Kd=3)
                                 Returns <tid> for activation.
  act_traj <tid>          Activate loaded trajectory (Start moving).
  unload_traj <tid>       Unload/Stop trajectory.

# --- Locomotion ---
  stand                   ChangeMode(kWalking) -> Stand up
  damp                    ChangeMode(kDamping) -> Relax joints
  stop                    Move(0, 0, 0)
  w / s                   Forward / Backward (Move)
  a / d                   Left / Right (Move)
  q / e                   Rotate Left / Right (Move)

# --- Other ---
  ro                      ResetOdometry()
  gm                      GetMode()
  gs                      GetStatus()
====================================
""")

def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <network_interface> [robot_name]")
        sys.exit(-1)

    net_if = sys.argv[1]
    robot_name = sys.argv[2] if len(sys.argv) >= 3 else ""

    # Initialize SDK
    b1.ChannelFactory.Instance().Init(0, net_if)
    client = b1.B1LocoClient()
    
    if robot_name:
        client.InitWithName(robot_name)
    else:
        client.Init()

    print(f"B1 CLI initialized on {net_if}.")
    print_help()

    while True:
        try:
            # 保持和之前一样的交互风格：等待用户输入
            raw = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nExit.")
            break

        if not raw:
            continue

        parts = raw.split()
        cmd = parts[0].lower()
        args = parts[1:]

        try:
            # ===== 1. Simplified Custom Trajectory Commands =====
            if cmd == "load_simple":
                if len(args) != 2:
                    print("Usage: load_simple <traj_file_path> <model_file_path>")
                    print("Example: load_simple ./walk.bin ./policy.xml")
                    continue
                
                t_path = args[0]
                m_path = args[1]

                # 检查文件是否存在
                if not Path(t_path).is_file():
                    print(f"Error: Traj file not found: {t_path}")
                    continue
                if not Path(m_path).is_file():
                    print(f"Error: Model file not found: {m_path}")
                    continue

                # 使用辅助函数构建复杂对象
                print(f"Building config from: {t_path}, {m_path} ...")
                traj_config = create_traj_config_from_paths(t_path, m_path)

                # 调用 C++ 接口
                tid = client.LoadCustomTrainedTraj(traj_config)
                print(f"--------------------------------------------------")
                print(f"SUCCESS! Trajectory Loaded.")
                print(f"Trajectory ID (TID): {tid}")
                print(f"Next step: type 'act_traj {tid}' to start.")
                print(f"--------------------------------------------------")

            elif cmd == "act_traj":
                if len(args) != 1:
                    print("Usage: act_traj <tid>")
                    continue
                tid = args[0]
                client.ActivateCustomTrainedTraj(tid)
                print(f"Command sent: Activate trajectory [{tid}]")

            elif cmd == "unload_traj":
                if len(args) != 1:
                    print("Usage: unload_traj <tid>")
                    continue
                tid = args[0]
                client.UnloadCustomTrainedTraj(tid)
                print(f"Command sent: Unload trajectory [{tid}]")

            # ===== 2. Basic Commands (Helper for testing) =====
            elif cmd in ("help", "?"):
                print_help()

            elif cmd in ("quit", "exit"):
                break

            elif cmd == "stand":
                client.ChangeMode(b1.RobotMode.kWalking)
                print("Mode -> Walking")

            elif cmd == "damp":
                client.ChangeMode(b1.RobotMode.kDamping)
                print("Mode -> Damping")

            elif cmd == "stop":
                client.Move(0.0, 0.0, 0.0)
                print("Move -> Stop")

            elif cmd == "w":
                client.Move(0.4, 0.0, 0.0)
                print("Move -> Forward")

            elif cmd == "s":
                client.Move(-0.4, 0.0, 0.0)
                print("Move -> Backward")

            elif cmd == "a":
                client.Move(0.0, 0.2, 0.0)
                print("Move -> Left")

            elif cmd == "d":
                client.Move(0.0, -0.2, 0.0)
                print("Move -> Right")
            
            elif cmd == "q":
                client.Move(0.0, 0.0, 0.3)
                print("Move -> Rotate Left")

            elif cmd == "e":
                client.Move(0.0, 0.0, -0.3)
                print("Move -> Rotate Right")

            elif cmd == "ro":
                client.ResetOdometry()
                print("Odometry reset.")

            elif cmd == "gm":
                resp = client.GetMode()
                print(f"Current Mode: {resp.mode}")

            elif cmd == "gs":
                resp = client.GetStatus()
                print(f"Status: Mode={resp.current_mode}, Body={resp.current_body_control}")

            else:
                print(f"Unknown command: {cmd}")

        except Exception as e:
            print(f"Request Failed: {e}")

if __name__ == "__main__":
    main()