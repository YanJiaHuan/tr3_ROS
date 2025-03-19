# TR3

TR3 (Piper Robot Arm),  [Ros SDK](https://github.com/agilexrobotics/Piper_ros), [Additional info](https://github.com/agilexrobotics/piper_sdk/blob/master/asserts/INTERFACE.MD)
### 0. Pre Step:
12-10-2024
Install ros2 humble, the complete installing steps can be found [here](http://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html). \

Create env, humble only supports python 3.10 
```shell 
mamba create -n tr3 python=3.10
```
Install general dependencies

```shell
mamba install --file requirements
```
Install dependencies for ocluous quest 2

 ```shell
sudo apt install android-tools-adb
pip install pure-python-adb
pip install pyyaml
pip install rich
 ```

Type into `adb devices` and expect output like
```shell
List of devices attached
1WMHHA618C1506	device
```

Install dependencies for Piper

```shell
sudo apt update && sudo apt install ethtool
sudo apt update && sudo apt install can-utils
pip install catkin_pkg==0.4.23
pip install empy==3.3.4
pip install lark-parser==0.12.0
pip3 install python-can
pip3 install piper_sdk
```

If encounter the error of piper continuously using the python from system instead of your created conda env:
```shell
sudo apt purge 'python3-colcon-*' # remove the colcon build and reinstall it inside the conda env
conda activate tr3
pip install --upgrade colcon-common-extensions
which colcon # expected output: /home/zcai/miniforge3/envs/tr3/bin/colcon
```

!注意： 这个机械臂有一个模式切换的问题，指令控制，上位机控制，示教拖拽，can模式等属于不同的模式，模式之前的切换，存在bug，需要先切换到待机模式(重新插拔电源，机械臂的默认模式)再切换到其他模式，否则机械臂会跳变，很危险。以下有几种切换到待机模式的方式：\
	1. 重新插拔电源 \
	2. 在上位机控制系统里，先点‘装载’，在点‘使零’ -- 假如控制后，直接使零，也会跳变 \
	3. 他们的python sdk里封装了复位（待机）功能-->[链接](https://github.com/agilexrobotics/piper_sdk?tab=readme-ov-file#%E6%9C%BA%E6%A2%B0%E8%87%82reset)

### 2. Launch 
```shell
# 连接机械臂
cd /home/zcai/jh_workspace/tr3/src/piper_ros
bash can_activate.sh can0 1000000
sudo ethtool -i can0 | grep bus
bash can_activate.sh can_piper 1000000 "1-7:1.0"
```
```shell
# 切换conda环境
conda activate tr3
```

```shell
# 编译ros包
#conda deactivate && conda deactivate
conda activate tr3
rm -rf log/ build/ install/ # 清除之前的编译记录
colcon build
source /opt/ros/humble/setup.bash
source install/setup.bash # 每个终端都要source 
# conda activate tr3

```

```shell
# 启动机械臂luanch节点
ros2 launch piper start_single_piper.launch.py can_port:=can_piper auto_enable:=true gripper_exist:=false gripper_val_mutiple:=2

# 启动rivz控制launch节点, joint 1->6 分别对应机械臂从根部到末端的每个关节
ros2 launch piper start_single_piper_rviz.launch.py can_port:=can_piper auto_enable:=true gripper_exist:=false gripper_val_mutiple:=2
```

```shell
# 启动vr遥操piper的pub节点
ros2 run vr_quest2_pub vr_pub
```

```shell
# 启动realsense 摄像头 [realsense官方自带的launch方法]
ros2 launch realsense2_camera rs_launch.py camera_name:=camera1 serial_no:="'924322063554'"
ros2 launch realsense2_camera rs_launch.py camera_name:=camera2 serial_no:="'935722063008'"
```

```shell
# 启动末端工具节点 
## 启动esp32 classic bluetooth控制节点
ros2 launch end_tools magnet_bluetooth_esp32_launch.py
#-----------单独发送测试指令----------------#
ros2 topic pub /electromagnet_control std_msgs/msg/Bool '{data: true}' --once
#----------------------------------------#    
```

### 3. Data collection
