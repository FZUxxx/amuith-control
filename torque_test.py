import odrive
from odrive.enums import *
import time
import sys
import threading
import queue

# 用于存储用户输入的队列
torque_queue = queue.Queue()
# 连接到 ODrive
print("正在连接 ODrive...")
try:
    my_drive = odrive.find_any()
    print("已连接到 ODrive")
except Exception as e:
    print(f"连接失败: {e}")
    exit()

# 配置力矩控制模式
my_drive.axis0.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL  # 改为力矩控制
my_drive.axis0.motor.config.current_lim = 17  # 电流限制，单位：安培，确保足够扭矩
#设置转矩常数
my_drive.axis0.motor.config.torque_constant = 0.0147  # 转矩常数，单位：N·m/A
my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
time.sleep(1)

# 输入线程函数：接收用户输入的扭矩
def input_thread():
    while True:
        try:
            torque = float(input("请输入目标扭矩 (N·m，正数顺转，负数反转，0 停止): "))
            torque_queue.put(torque)
        except ValueError:
            print("请输入有效的数字！")

# 启动输入线程
threading.Thread(target=input_thread, daemon=True).start()

# 主循环：控制电机并显示状态
current_target_torque = 0.01  # 初始扭矩，单位：N·m
my_drive.axis0.controller.input_torque = current_target_torque
print(f"初始扭矩: {current_target_torque} N·m")
print("电机运行中，随时输入新扭矩，按 Ctrl+C 退出")

try:
    while True:
        # 检查是否有新扭矩指令
        try:
            new_torque = torque_queue.get_nowait()
            current_target_torque = new_torque
            my_drive.axis0.controller.input_torque = current_target_torque
            #print(f"\n设置新扭矩: {current_target_torque:.4f} N·m")
        except queue.Empty:
            pass  # 没有新输入，继续显示当前状态

        # 获取并显示当前速度和电流
        current_velocity = my_drive.axis0.encoder.vel_estimate
        current_current = my_drive.axis0.motor.current_control.Iq_measured
        # print(f"当前速度: {current_velocity:.2f} 转/秒，当前电流: {current_current:.2f} A")
        time.sleep(0.05)  # 20Hz 刷新率

except KeyboardInterrupt:
    my_drive.axis0.controller.input_torque = 0
    my_drive.axis0.requested_state = AXIS_STATE_IDLE
    print("\n电机已停止")
except Exception as e:
    print(f"错误: {e}")
    my_drive.axis0.requested_state = AXIS_STATE_IDLE