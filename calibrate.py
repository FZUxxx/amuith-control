import odrive
from odrive.enums import *
import time

# 连接 ODrive
my_drive = odrive.find_any()
print("已连接到 ODrive")

# 清除错误
my_drive.axis0.error = 0
my_drive.axis0.motor.error = 0
my_drive.axis0.encoder.error = 0
print("错误已清除")

# 校准电机
print("开始电机校准...")
my_drive.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION
while my_drive.axis0.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)
if my_drive.axis0.motor.error == 0:
    my_drive.axis0.motor.config.pre_calibrated = True
    print("电机校准完成")
else:
    print(f"电机校准失败，错误: {my_drive.axis0.motor.error}")
    exit()

# 校准编码器
print("开始编码器校准...")
my_drive.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
while my_drive.axis0.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)
if my_drive.axis0.encoder.error == 0:
    my_drive.axis0.encoder.config.pre_calibrated = True
    print("编码器校准完成")
else:
    print(f"编码器校准失败，错误: {my_drive.axis0.encoder.error}")
    exit()

# 保存配置
my_drive.save_configuration()
print("配置已保存")

# 验证校准结果
print(f"电机预校准: {my_drive.axis0.motor.config.pre_calibrated}")
print(f"编码器预校准: {my_drive.axis0.encoder.config.pre_calibrated}")
print(f"当前状态: {my_drive.axis0.current_state}")