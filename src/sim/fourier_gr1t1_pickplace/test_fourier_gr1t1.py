#!/usr/bin/env python3
# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0

"""
Fourier GR1T1 Pick-Place 场景加载脚本
"""

import argparse
import torch
from isaaclab.app import AppLauncher

# 命令行参数
parser = argparse.ArgumentParser(description="Fourier GR1T1 Pick-Place 场景加载")
parser.add_argument("--num_envs", type=int, default=1, help="环境数量")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# 启动 Omniverse
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym
from tasks.fourier_gr1t1_tasks.pick_place_cylinder_gr1t1.pickplace_cylinder_fourier_gr1t1_env_cfg import PickPlaceFourierGR1T1BaseFixEnvCfg
from subscribe.fourier_gr1_t1_controller import FourierGR1T1Controller

def main():
    print("=" * 60)
    print("Fourier GR1T1 Pick-Place 场景加载")
    print("=" * 60)
    
    # 创建环境配置
    try:
        env_cfg = PickPlaceFourierGR1T1BaseFixEnvCfg()
        print(f"[INFO] 环境配置加载成功")
    except Exception as e:
        print(f"[ERROR] 环境配置加载失败: {e}")
        import traceback
        traceback.print_exc()
        return
    
    controller = FourierGR1T1Controller("")
    controller.Start()
    # 创建环境
    try:
        env = gym.make("Isaac-PickPlace-Cylinder-Fourier-GR1T1-Joint", cfg=env_cfg)
        print(f"[INFO] 环境创建成功")
        # all_joint_names = env.scene["robot"].data.joint_names
        # last_action = torch.zeros(len(all_joint_names), device=env.device)
        # print(all_joint_names)
        while(True):
            import time
            time.sleep(0.5)
            # ik = controller.GetIKMsg()
            # left_qpos  = ik[:7]
            # right_qpos = ik[7:]

            # left_joint_names  = ["left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
            #                     "left_elbow_pitch_joint", "left_wrist_yaw_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint"]
            # right_joint_names = ["right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
            #                     "right_elbow_pitch_joint", "right_wrist_yaw_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint"]

            # for i, name in enumerate(left_joint_names):
            #     idx = all_joint_names.index(name)
            #     last_action[idx] = left_qpos[i]

            # for i, name in enumerate(right_joint_names):
            #     idx = all_joint_names.index(name)
            #     last_action[idx] = right_qpos[i]

            # env_state = env.unwrapped.scene.get_state()
            # env_state_json = sim_state_to_json(env_state)

            # env.step(last_action)
            print(f"[INFO] 环境创建成功")

    except Exception as e:
        print(f"[ERROR] 环境创建失败: {e}")
        import traceback
        traceback.print_exc()
        return
    
    print(f"\n[SUCCESS] 场景加载完成！")
    print(f"  • 环境数量: {env.num_envs}")
    print(f"  • 观测空间: {env.observation_space}")
    print(f"  • 动作空间: {env.action_space}")
    
    env.close()
    print("[INFO] 环境已关闭")
    controller.stop()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
        print("[INFO] 应用已关闭")

