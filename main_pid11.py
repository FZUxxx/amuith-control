import serial
import odrive
from odrive.enums import *
import time
import threading
import sys
import csv
import numpy as np

# PID 控制器类（增强 Anti-Windup）
class PIDController:
    def __init__(self, kp, ki, kd, integral_limit=10.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.prev_error = 0
        self.integral = 0
        self.last_time = time.time()
        self.saturated = False
        self.saturation_time = 0.0
        self.saturation_threshold = 1.0

    def update_params(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def compute(self, setpoint, measured_value):
        current_time = time.time()
        dt = current_time - self.last_time
        if dt <= 0:
            dt = 1e-6
        error = setpoint - measured_value
        p_term = self.kp * error

        if self.saturated:
            self.saturation_time += dt
            if self.saturation_time > self.saturation_threshold:
                self.integral *= 0.5
                self.saturation_time = 0.0
        else:
            self.integral += error * dt
            self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)
            self.saturation_time = 0.0

        i_term = self.ki * self.integral
        derivative = (error - self.prev_error) / dt
        d_term = self.kd * derivative
        output = p_term + i_term + d_term
        self.prev_error = error
        self.last_time = current_time
        return output

    def set_saturated(self, saturated):
        self.saturated = saturated

# 低通滤波器类
class LowPassFilter:
    def __init__(self, alpha=0.1):
        self.alpha = alpha
        self.filtered_value = None

    def filter(self, value):
        if self.filtered_value is None:
            self.filtered_value = value
        else:
            self.filtered_value = self.alpha * value + (1 - self.alpha) * self.filtered_value
        return self.filtered_value

# 数据结构
imu_data = []
angle_filter = LowPassFilter(alpha=0.01)
gyro_filter = LowPassFilter(alpha=0.05)

# IMU 串口配置
try:
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    print("已连接到 IMU（/dev/ttyACM0）")
except Exception as e:
    print(f"IMU 连接失败: {e}")
    sys.exit(1)

# 连接 ODrive
print("正在连接 ODrive...")
try:
    my_drive = odrive.find_any()
    print("已连接到 ODrive")
except Exception as e:
    print(f"ODrive 连接失败: {e}")
    sys.exit(1)

# 配置 ODrive 为速度控制模式
my_drive.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
my_drive.axis0.motor.config.current_lim = 10
my_drive.axis0.trap_traj.config.accel_limit = 20
my_drive.axis0.trap_traj.config.decel_limit = 20
my_drive.axis0.controller.config.vel_limit = 60
my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
time.sleep(1)

# 初始化 PID 控制器
angle_pid = PIDController(kp=0, ki=0, kd=0, integral_limit=15.0)  # 增大 kp
gyro_pid = PIDController(kp=0.3, ki=0.5, kd=0.1, integral_limit=10.0)

# 读取 IMU 数据
def read_imu_data():
    while True:
        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8').strip()
                parts = line.split('|')
                if len(parts) == 2:
                    angle_part = parts[0].split(':')[1].strip()
                    gyro_part = parts[1].split(':')[1].strip()
                    angles = [float(x) for x in angle_part.split(',')]
                    gyros = [float(x) for x in gyro_part.split(',')]
                    if len(angles) == 3 and len(gyros) == 3:
                        filtered_angle_z = angle_filter.filter(angles[2])
                        filtered_gyro_z = gyro_filter.filter(gyros[2])
                        imu_data.append([filtered_angle_z, filtered_gyro_z])
            except (ValueError, IndexError, UnicodeDecodeError):
                print("收到无效的 IMU 数据")
        time.sleep(0.01)

# 主控制循环
def control_loop():
    global imu_data
    current_target_angle = -172.0  # 目标角度
    current_motor_vel = 0.0
    DIRECTION_FACTOR = 1  # 方向因子，根据实际测试调整（可能为 -1）
    my_drive.axis0.controller.input_vel = 0.0
    ANGLE_TOLERANCE = 5.0
    MAX_MOTOR_VELOCITY = 60.0
    MAX_DELTA_VELOCITY = 20.0
    stable = False
    start_time = time.time()

    csv_filename = f"motor_data_{int(time.time())}.csv"
    csv_header = ["Timestamp", "Target_Angle_deg", "Current_Angle_deg", "Target_Gyro_Vel_deg_per_s", 
                  "Current_Gyro_Vel_deg_per_s", "Delta_Motor_Vel_turns_per_s", "Current_Motor_Vel_turns_per_s", 
                  "Motor_Current_A"]

    try:
        with open(csv_filename, 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(csv_header)
            while True:
                if imu_data:
                    current_angle, current_gyro_vel = imu_data[-1]
                else:
                    current_angle = 0.0
                    current_gyro_vel = 0.0

                error = current_target_angle - current_angle
                if abs(error) <= ANGLE_TOLERANCE:
                    if not stable:
                        stable = True
                        print("已稳定")
                    target_gyro_vel = 0.0
                else:
                    target_gyro_vel = angle_pid.compute(current_target_angle, current_angle)
                    stable = False

                max_gyro_vel = 180.0
                target_gyro_vel = max(min(target_gyro_vel, max_gyro_vel), -max_gyro_vel)

                delta_motor_vel = gyro_pid.compute(target_gyro_vel, current_gyro_vel)
                delta_motor_vel = max(min(delta_motor_vel, MAX_DELTA_VELOCITY), -MAX_DELTA_VELOCITY)
                current_motor_vel += delta_motor_vel
                current_motor_vel = max(min(current_motor_vel, MAX_MOTOR_VELOCITY), -MAX_MOTOR_VELOCITY)
                saturated = abs(current_motor_vel) >= MAX_MOTOR_VELOCITY
                angle_pid.set_saturated(saturated)
                gyro_pid.set_saturated(saturated)

                my_drive.axis0.controller.input_vel = current_motor_vel * DIRECTION_FACTOR
                current_motor_vel_actual = my_drive.axis0.encoder.vel_estimate
                current_current = my_drive.axis0.motor.current_control.Iq_measured

                timestamp = time.time() - start_time
                csv_writer.writerow([
                    f"{timestamp:.3f}",
                    f"{current_target_angle:.2f}",
                    f"{current_angle:.2f}",
                    f"{target_gyro_vel:.2f}",
                    f"{current_gyro_vel:.2f}",
                    f"{delta_motor_vel:.2f}",
                    f"{current_motor_vel_actual:.2f}",
                    f"{current_current:.2f}"
                ])

                print(f"目标角度: {current_target_angle:.2f} 度, 当前角度: {current_angle:.2f} 度, "
                      f"目标角速度: {target_gyro_vel:.2f} 度/秒, 当前角速度: {current_gyro_vel:.2f} 度/秒, "
                      f"目标电机转速: {current_motor_vel:.2f} 转/秒, 当前电机转速: {current_motor_vel_actual:.2f} 转/秒, "
                      f"电流: {current_current:.2f} A, 稳定: {stable}")

                time.sleep(0.05)

    except Exception as e:
        print(f"错误: {e}")
    finally:
        my_drive.axis0.controller.input_vel = 0
        my_drive.axis0.requested_state = AXIS_STATE_IDLE
        ser.close()
        print(f"电机已停止，串口已关闭，数据保存至 {csv_filename}")

# 启动程序
if __name__ == "__main__":
    # 启动 IMU 数据读取线程
    threading.Thread(target=read_imu_data, daemon=True).start()

    # 启动控制循环
    control_loop()