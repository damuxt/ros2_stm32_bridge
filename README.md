# ros2_stm32_bridge

适配以stm32为主控的两轮或四轮差速底盘的ROS2驱动。

### 使用准备

#### 1.硬件准备

通过USB数据线将机器人底盘连接到上位机，并打开底盘开关。上位机终端下执行指令：

```
ll /dev/ttyUSB*
```

如正常，将输出类似如下的结果：

```
crw-rw---- 1 root dialout 166, 0  4月 27 20:15 /dev/ttyUSB0
```

PS：如果物理连接无异常，Ubuntu系统显示`无法访问 ‘/dev/ttyUSB‘: 没有那个文件或目录`，那么可能是brltty驱动占用导致的，可以运行`sudo apt remove brltty `然后重新插拔一下设备即可解决问题。

#### 2.软件安装

将此软件包下载到你的ROS2工作空间下的src目录。

#### 3.环境配置

##### 3.1端口映射

终端下进入`ros2_stm32_bridge/scripts`,并执行指令：

```
bash create_udev_rules.sh
```

该指令将为STM32端口绑定一个固定名称，重新将底盘与上位机连接，执行如下指令：

```
ll /dev | grep -i ttyUSB
```

如正常，将输出类似如下的结果：

```
lrwxrwxrwx   1 root root           7  4月 27 20:16 mycar -> ttyUSB0
crwxrwxrwx   1 root dialout 166,   0  4月 27 20:16 ttyUSB0
```

##### 3.2设置车辆类型

修改上位机用户目录下的 .bashrc 文件，在该文件中设置车辆类型，如果你使用的是两轮差速底盘，则添加如下语句：

```
export MYCAR_MODEL=stm32_2w
```

如果你使用的是四轮差速底盘，则添加如下语句：

```
export MYCAR_MODEL=stm32_4w
```

#### 4.构建功能包

工作空间下调用如下指令，构建功能包：

```
colcon build --packages-select ros2_stm32_bridge
```

### 使用流程

#### 1.配置参数

在功能包下提供了机器人底盘相关参数的配置文件`ros2_stm32_bridge/params`，`stm32_2w.yaml`为两轮差速底盘的配置文件,`stm32_4w.yaml`为四轮差速底盘的配置文件，参数内容可以自行修改（如果是初次使用，建议使用默认）。

`stm32_2w.yaml`的文件内容如下：

```
/mini_driver:
  ros__parameters:
    base_frame: base_footprint
    baud_rate: 115200
    control_rate: 10
    encoder_resolution: 44.0
    kd: 30
    ki: 0
    kp: 25
    maximum_encoding: 100.0
    model_param_acw: 0.21
    model_param_cw: 0.21
    odom_frame: odom
    port_name: /dev/mycar
    qos_overrides:
      /parameter_events:
        publisher:
          depth: 1000
          durability: volatile
          history: keep_last
          reliability: reliable
      /tf:
        publisher:
          depth: 100
          durability: volatile
          history: keep_last
          reliability: reliable
    reduction_ratio: 90.0
    use_sim_time: false
    wheel_diameter: 0.065

```

`stm32_4w.yaml`的文件内容如下：

```
/mini_driver:
  ros__parameters:
    base_frame: base_footprint
    baud_rate: 115200
    control_rate: 10
    encoder_resolution: 44.0
    kd: 30
    ki: 0
    kp: 25
    maximum_encoding: 100.0
    model_param_acw: 0.45
    model_param_cw: 0.45
    odom_frame: odom
    port_name: /dev/mycar
    qos_overrides:
      /parameter_events:
        publisher:
          depth: 1000
          durability: volatile
          history: keep_last
          reliability: reliable
      /tf:
        publisher:
          depth: 100
          durability: volatile
          history: keep_last
          reliability: reliable
    reduction_ratio: 90.0
    use_sim_time: false
    wheel_diameter: 0.080
```

#### 2.启动launch文件

工作空间下，调用如下指令启动launch文件：

```
. install/setup.bash
ros2 launch ros2_stm32_bridge driver.launch.py
```

#### 3.控制底盘运动

上位机上，启动键盘控制节点：

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

接下来，就可以通过键盘控制机器人运动了。
