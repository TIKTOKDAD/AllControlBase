"""平台配置

定义不同机器人平台的运动学特性。

坐标系说明:
===========

系统使用两个坐标系:

1. base_link (机体坐标系)
   - 原点在机器人中心，X轴朝前，Y轴朝左
   - 网络输出的轨迹在此坐标系下

2. odom (里程计坐标系)
   - 从启动位置开始累积，不需要建图
   - 控制器在此坐标系下工作

控制输出坐标系 (output_frame):
   - 差速车/阿克曼车: base_link (vx, omega) - 机体坐标系
   - 全向车/四旋翼: world (vx, vy, omega) - 世界坐标系

平台特性说明:
=============

velocity_heading_coupled:
   - True: 速度方向与航向耦合（差速车、阿克曼车）
     车辆只能沿车头方向移动，不能横向滑移
   - False: 速度方向与航向解耦（全向车、四旋翼）
     可以在任意方向移动，航向独立控制

is_ground_vehicle:
   - True: 地面车辆，z 方向运动受限
   - False: 空中平台，可三维运动

can_rotate_in_place:
   - True: 可原地旋转（差速车、全向车、四旋翼）
   - False: 不可原地旋转，需要最小转弯半径（阿克曼车）

支持的平台类型:
- ackermann: 阿克曼转向车辆 (如汽车)
- differential: 差速驱动车辆 (如两轮机器人)
- omni: 全向移动平台 (如麦克纳姆轮)
- quadrotor: 四旋翼无人机
"""
from ..core.enums import PlatformType


PLATFORM_CONFIG = {
    # 阿克曼转向车辆 (如汽车)
    "ackermann": {
        "type": PlatformType.ACKERMANN,
        "velocity_heading_coupled": True,   # 速度方向与航向耦合
        "is_ground_vehicle": True,          # 地面车辆
        "can_rotate_in_place": False,       # 不可原地旋转
        "output_type": "differential",      # 输出类型: 差速 (v, omega)
        "output_frame": "base_link",        # 输出坐标系: 机体坐标系
    },
    
    # 差速驱动车辆 (如两轮机器人)
    "differential": {
        "type": PlatformType.DIFFERENTIAL,
        "velocity_heading_coupled": True,   # 速度方向与航向耦合
        "is_ground_vehicle": True,          # 地面车辆
        "can_rotate_in_place": True,        # 可原地旋转
        "output_type": "differential",      # 输出类型: 差速
        "output_frame": "base_link",        # 输出坐标系: 机体坐标系
    },
    
    # 全向移动平台 (如麦克纳姆轮)
    "omni": {
        "type": PlatformType.OMNI,
        "velocity_heading_coupled": False,  # 速度方向与航向解耦
        "is_ground_vehicle": True,          # 地面车辆
        "can_rotate_in_place": True,        # 可原地旋转
        "output_type": "omni",              # 输出类型: 全向
        "output_frame": "world",            # 输出坐标系: 世界坐标系
    },
    
    # 四旋翼无人机
    "quadrotor": {
        "type": PlatformType.QUADROTOR,
        "velocity_heading_coupled": False,  # 速度方向与航向解耦
        "is_ground_vehicle": False,         # 空中平台
        "can_rotate_in_place": True,        # 可原地旋转
        "output_type": "3d",                # 输出类型: 三维
        "output_frame": "world",            # 输出坐标系: 世界坐标系
    },
}
