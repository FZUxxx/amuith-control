import odrive
from odrive.enums import *
import time
import sys
import threading
import queue

# 用于存储用户输入的队列
speed_queue = queue.Queue()

# 连接到 ODrive
print("正在连接 ODrive...")
try:
    my_drive = odrive.find_any()
    print("已连接到 ODrive")
except Exception as e:
    print(f"连接失败: {e}")
    exit()

# 配置速度控制模式
my_drive.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
my_drive.axis0.motor.config.current_lim = 12  # 确保足够扭矩
# 设置速度环的增益
#my_drive.axis0.controller.config.vel_gain = 0.03  # 比例增益
#my_drive.axis0.controller.config.vel_integrator_gain = 0.01  # 调低积分增益

#设置加减速限制
my_drive.axis0.trap_traj.config.accel_limit = 1200  # 单位：转/秒²，设为较大值
my_drive.axis0.trap_traj.config.decel_limit = 1200  # 单位：转/秒²，设为较大值
my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
time.sleep(1)

# 输入线程函数：接收用户输入的速度
def input_thread():
    while True:
        try:
            speed = float(input("请输入目标速度 (转/秒，输入负数反转，0 停止): "))
            speed_queue.put(speed)
        except ValueError:
            print("请输入有效的数字！")

# 启动输入线程
threading.Thread(target=input_thread, daemon=True).start()

# 主循环：控制电机并显示速度
current_target_velocity = 1.0
my_drive.axis0.controller.input_vel = current_target_velocity
print(f"初始速度: {current_target_velocity} 转/秒")
print("电机运行中，随时输入新速度，按 Ctrl+C 退出")

try:
    while True:
        # 检查是否有新速度指令
        try:
            new_speed = speed_queue.get_nowait()
            current_target_velocity = new_speed
            my_drive.axis0.controller.input_vel = current_target_velocity
            print(f"\n设置新速度: {current_target_velocity:.2f} 转/秒")
        except queue.Empty:
            pass  # 没有新输入，继续显示当前速度

        # 获取并显示当前速度
        current_velocity = my_drive.axis0.encoder.vel_estimate
        #获取当前的电流
        current_current = my_drive.axis0.motor.current_control.Iq_measured
        #print(f"当前速度: {current_velocity:.2f} 转/秒， 当前电流: {current_current:.2f} A")
        time.sleep(0.05)  # 20Hz 刷新率

except KeyboardInterrupt:
    my_drive.axis0.controller.input_vel = 0
    my_drive.axis0.requested_state = AXIS_STATE_IDLE
    print("\n电机已停止")
except Exception as e:
    print(f"错误: {e}")
    my_drive.axis0.requested_state = AXIS_STATE_IDLE