[TOC]
#  当前版本
```
current version : xfarm-mini-dev-D2.4-102021071301
current_hardware: xfarm-mini-
```
#  程序启动方式
1. 机械臂服务端启动程序:
```
$ roslaunch arm_control arm_server_white.launch 
or 
$ roslaunch arm_control arm_server_black.launch
```

## 1. 控制机械臂末端到达指定位置
服务名称:　`/arm/move_to_target_xyz`
类型:  `arm_control/Move_Target_3d`
```
string direction  # 当为"home"时表示回到初始位置．
float32 x         # x，单位为mm
float32 y         # y,单位为mm
float32 z         # z,单位为mm
float32 s
float32 r
float32 h
bool is_relative   # 是否是相对运动模式,默认False，表示绝对位置模式
float32 vel　　　　# 表示运动速度
string move_mode　# 运动模式，即关节,直线，门型，可选”MOVJ”,”MOVL”,”JUMP”
---
bool success
string message 
``` 

|参数取值(direction) |含义 | 用途 |
|---|---|---|
|“home”|机械臂回到初始位置|复位|
|“xyz”|表示会按照用户给定x,y,z值来执行|控制机械臂末端到达直角坐标系下指定位置
| “x” |小写，表示会按照用户给定的x值来执行,y与z为机械臂当前值|控制机械臂末端到达直角坐标系下指定位置|
| “y” |小写，表示会按照用户给定的y值来执行,x与z为机械臂当前值|控制机械臂末端到达直角坐标系下指定位置|
| “z” |小写，表示会按照用户给定的z值来执行,x与y为机械臂当前值|控制机械臂末端到达直角坐标系下指定位置|
|“X+”|末端向x正向（向前）运动，直到人为停止或到达极限位置才停。此时给定x、y、z将不会生效|开环控制|
|“X-”|末端向x负向（向后）运动，直到人为停止或到达极限位置才停。此时给定x、y、z将不会生效|开环控制|
|“Y+”|末端向y正向（向左）运动，直到人为停止或到达极限位置才停。此时给定x、y、z将不会生效|开环控制|
|“Y-”|末端向y负向（向右）运动，直到人为停止或到达极限位置才停。此时给定x、y、z将不会生效|开环控制|
|“Z+”|末端向z正向（向上）运动，直到人为停止或到达极限位置才停。此时给定x、y、z将不会生效|开环控制|
|“Z-”|末端向z负向（向下）运动，直到人为停止或到达极限位置才停。此时给定x、y、z将不会生效|开环控制|
|“srh”|表示以极坐标形式来控制机械臂，按照给定的s、r、h来执行|控制机械臂末端到达极坐标系下指定位置|

调用示例:
```
$ rosservice call /arm/move_to_target_xyz "{direction: 'xyz', x: 200.0, y: 0.0, z: 100.0, s: 0.0, r: 0.0, h: 0.0, is_relative: false, vel: 0.0, move_mode: 'MOVJ'}"
```

## 2. 控制机械臂各关节到达指定角度,可控制单个，也同时控制多个．
服务名称:　`/arm/move_to_target_servo`
类型: 　`arm_control/Move_Target_Servo`
```
string id_name　　　# 可选"BUTTOM","LEFT","RIGHT","WRIST"以及"ALL"
float32 joint1　　　# "BUTTOM"角度
float32 joint2     # "LEFT"角度
float32 joint3     # "RIGHT"角度
float32 joint4     # "WRIST"角度
float32 vel　　　　　# 表示末端运动速度,会自动计算关节速度
---
bool success
string message
``` 


|参数|含义|用途|
|---|---|---|
|ALL|表示对给定的angles均会执行|闭环控制，同时让四个关节均达到指定位置|
|BUTTOM|表示对给定的angles仅会执行第一个值|闭环控制，仅让第一个关节均达到指定位置|
|LEFT|表示对给定的angles仅会执行第二个值|闭环控制，仅让第二个关节均达到指定位置|
|RIGHT|表示对给定的angles仅会执行第三个值|闭环控制，仅让第三个关节均达到指定位置|
|WRIST|表示对给定的angles仅会执行第四个值|闭环控制，仅让第四个关节均达到指定位置|
|BUTTOM+|表示机械臂将会向着第一个关节的最大位置去移动，直到人为停止或到达极限位置才停。此时给定的angles将不会生效。|开环控制，仅让第一个关节向正向一直运动|
|BUTTOM-|表示机械臂将会向着第一个关节的最小位置去移动，直到人为停止或到达极限位置才停。此时给定的angles将不会生效。|开环控制，仅让第一个关节向负向一直运动|
|LEFT+|表示机械臂将会向着第二个关节的最大位置去移动，直到人为停止或到达极限位置才停。此时给定的angles将不会生效。|开环控制，仅让第二个关节向正向一直运动|
|LEFT-|表示机械臂将会向着第二个关节的最小位置去移动，直到人为停止或到达极限位置才停。此时给定的angles将不会生效。|开环控制，仅让第二个关节向负向一直运动|
|RIGHT+|表示机械臂将会向着第三个关节的最大位置去移动，直到人为停止或到达极限位置才停。此时给定的angles将不会生效。|开环控制，仅让第三个关节向正向一直运动|
|RIGHT-|表示机械臂将会向着第三个关节的最小位置去移动，直到人为停止或到达极限位置才停。此时给定的angles将不会生效。|开环控制，仅让第三个关节向负向一直运动|

 由于第四轴为舵机，其控制时必须给定期望位置，所以不存在开环运动模式。
 以上运动速度范围均为0-200.

调用示例:
```
$ rosservice call /arm/move_to_target_servo "{id_name: 'ALL', angle0: 90.0, angle1: 50.0, angle2: 40.0, angle3: 90.0, vel: 40.0}"
```


## 3. 设置末端执行器类型

设置末端执行器的种类（无末端gongju（none）、气泵（pump）、夹子水平（gripper_h）、夹子竖直（gripper_v）笔夹（holder）,值得注意的是，系统默认为无末端状态（none）。

服务名称: `/arm/set_manipulator_type`
服务类型: `arm_control/Set_Manipulator_Type`
```
string end_type  # 可选"none"/"pump"/"gripper_h"/"gripper_v"/"holder",对应"无末端gongju"/"吸盘"/"夹持器水平安装"/"夹持器竖直安装"/"通用笔夹"
---
bool success
string message

``` 

调用示例:
```
$ rosservice call /arm/set_manipulator_type "pump"  
``` 
说明:开机时默认末端执行器类型为"无末端"


## 4. 设置末端执行器工作/不工作
服务名称: `/arm/control_manipulator`
服务类型: `arm_control/Control_Manipulator`
```
bool  used　　　        # True/False，仅对夹持器和吸盘生效
---
bool success
string message 
```
调用示例:
```
$ rosservice call /arm/control_manipulator "used: false"  
``` 

说明:仅传递是否使用即可，会依据上一次设置的末端执行器来进行控制．

## 5. 设置机械臂停止/暂停/继续运动
服务名称 : `/arm/set_arm_interrupt`
服务类型 : `arm_control/Set_Arm_Interrupt`
```
string type            #"STOP"/"PAUSE"/"CONTINUE"
---
bool success
string message 
```
调用示例:
```
$ rosservice call /arm/set_arm_interrupt "PAUSE"  
```
说明:
|状态|含义|
|---|---|
|PAUSE|暂停当前运动，下达继续命令后，还可继续完成未完成的动作|
|CONTINUE|继续当前运动，与"pause"搭配使用|
|STOP|停止当前运动，下达继续命令之后不会继续运动，仅等待下一次运动命令|

## 6. 获取机械臂末端位置
服务名称 : `/arm/get_current_xyz`
服务类型 : `arm_control/Get_Current_3d`
```
string mode #"xyz"/"srh"
---
bool success
float32 x_s
float32 y_r
float32 z_h
string message
```
调用示例:
```
$ rosservice call /arm/get_current_xyz "xyz" 
```

## 7. 获取各电机状态
服务名称: `/arm/get_servos_status`
服务类型: `arm_control/Get_Servos_Status`
```
---
bool success
string message 
float32 joint1
float32 joint2
float32 joint3
float32 joint4
```
说明:返回值依次为各电机角度．
错误类型:"TIMEOUT","E21","E22","uArm is not connect","other_error".
调用示例:
```
$ rosservice call /arm/get_servos_status 
```

## 8. 获取末端执行器状态
服务名称: `/arm/get_manipulator_status`
服务类型: `arm_control/Get_Manipulator_Status`
type : 
```
---
bool success
string message 
string mode
string status 
```
调用示例:
```
$ rosservice call /arm/get_manipulator_status 
```
## 9. 使能和释能机械臂

1) 使能：使机械臂各电机上电，此时由程序控制机械臂运动，不可人为搬动。
2) 释能：使用示教功能时，释放机械臂电机的锁死状态，此时可以人为掰动机械臂。

服务名称: `/arm/set_servo_attach`
服务类型: `arm_control/Set_Servo_Attach`
```
string servo_name # 传入"ALL",表示全部电机同时使能/释能(false)
string mode   # "true"/"false" 
---
bool success
string message 
``` 
调用示例:
```
$ rosservice call /arm/set_servo_attach "{servo_name: 'ALL', mode: true}"
```
## 10. 获取指定末端位置是否可达
服务名称: `/arm/get_pose_in_limit`
服务类型: `arm_control/Get_Pose_In_Limit``
```
float32 x_s
float32 y_r
float32 z_h
bool is_xyz
---
bool success
bool in_range
string message 
``` 
说明:输入为三维坐标,返回值 是否受限,若位置可达,则返回True．
错误类型:"TIMEOUT","E21","E22","uArm is not connect","other_error".
调用示例:
```
$ rosservice call /arm/get_pose_in_range "{x_s: 200, y_r: 0,z_h: 150,is_xyz: True}"
```


## 11.控制灯光（7.1）
type : `arm_control/Set_Led
```
string type
---
bool success
string message
```
**说明:** : 控制灯光点亮，四种，红，蓝，绿，关闭，type可选["red","green","blue","type:'off'"]

**错误类型**: "E21"，表示输入有误

**调用示例:**
```
$ rosservice call /arm/set_led "red"
```
**返回示例:**
```
success :True
message:''
```

## 12.错误码
|错误码|含义|可能原因|
|---|---|---|
|E19|机械臂未连接|检查数据线是否插好|
|E20|超时|执行的指令运行时间过长|
|E21|参数错误|调用服务所用的参数有误|
|E22|位置超限制|给定的位置超出机械臂的运动范围,或给出的速度超出限制|
|E23|获取末端状态时超时|在相对运动/MVJL/JUMP运动时,获取实时状态.|
|E24|电源未连接|机械臂仅用数据线与主控连接,机械臂本体未供电|
|E27|释能下运动|先使能|
|E28|圆弧模式下未设置中间点|中间点必须与终点配合使用|
|E30|末端执行器类型有误|可选”none”,”pump”,”gripper_h”,”gripper_v”,”holder”|
|E31|控制机械臂运动时侯给出的运动模式有误|可选“MOVJ”,”MOVL”,”CIRCLE_MID”,”CIRCLE_END”|


