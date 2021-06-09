# 平衡小车+循迹

基于 STM32F103C8T6 OpenMV MPU6050
采用双闭环控制

速度闭环和直立闭环

![ls](image/ControlSystem.png)

MPU6050 采用DMP硬解，四元数， 转欧拉角 得到roll，pitch，yaw

openMV 阈值化