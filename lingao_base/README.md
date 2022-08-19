# TARS v4.0 Mobile Platform README(CH)

当前版本协议：V3.1.0

 **NOTE:**  
* 驱动使用私有通讯协议
* 带线速度和角速度校准调节比例参数
* 带有基于里程计odom坐标发布功能
* 需要安装ROS环境
* 通讯速率高达500HZ
* (使用IMU 融合)需要安装基础包

## 启动文件

启动驱动包
``` linux
roslaunch lingao_base lingao_base_driver.launch
```

# TARS Bringup API
## 发布的话题
/imu/data_raw ([sensor_msgs/Imu](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html))  
- 主题包括基于加速度和陀螺仪传感器的机器人姿态  

/raw_odom ([nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html))  
- 基于编码器的的里程表信息

/lingao_base_driver/battery_state (lingao_msgs/LingAoBmsStatus)
- 电池电压和电量状态

/lingao_base_driver/rc_state (lingao_msgs/LingAoRCStatus)
- 反馈遥控数据（包括遥杆、开关、旋钮数据）

## 订阅的话题
/cmd_vel ([geometry_msgs/Twist](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Twist.html))
- 控制机器人平移线速度(单位:m/s)和旋转角速度(单位:rad/s)

## 参数设置 - Parameters
设置文件所在位置`/lingao_base/launch/lingao_base_driver.launch`

- ~baud (int, default: 230400)  
    设置串口波特率
- ~port (string, default: /dev/tars)  
    设置串口号
- ~freq (int, default: 100)  
    驱动通讯频率

- ~publish_odom_transform (bool, default: false)  
    是否发布基于里程计odom的TF转换
- ~linear_scale (double, default: 1.0)  
    里程计线速度校准比例
- ~angular_scale (double, default: 1.0)  
    里程计角速度校准比例
  
- ~imu_use (string, default: false)  
    是否发布板载imu信息
- ~cmd_vel_sub_timeout (double, default: 1.0)  
    接收cmd_vel超时时间，超时后速度置0
- ~imu_calibrate_gyro (string, default: true)  
    是否校准陀螺仪
- ~imu_cailb_samples (int, default: 300)  
    校准陀螺仪偏差的测量次数（越大精度会提高，相对的启动校准时间也增加）
    
IMU注意事项：首次订阅校准陀螺仪会有延迟并且不要动机器人
    
