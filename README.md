# 2024 CADC 中国国际飞行器设计挑战赛——模拟搜救

### 启动
依赖：  
	- ros-foxy  
	- Eigen3  
	- OpenCV4  
```shell
git clone https://github.com/PEACER-jh/AirCraft.git
cd AirCraft
colcon build
source install/setup.bash
```
```
# 1、正式上场
ros2 launch ac_bringup bringup.launch.py
# 2、视觉调参
ros2 launch ac_bringup test.launch.py
```

### 结构
1. **ac_bringup** ：项目启动工作包
2. **ac_camera**L：USB相机工作包
3. **ac_classify**：魔方、台球识别工作包
4. **ac_solver**：pnp解算工作包
5. **ac_transport**：电控通讯工作包

### 参数
所有的可调整参数都在ac_bringup工作包中的config文件夹下，文件名字对应着相应的工作包中的参数，具体参数意义在对应yaml文件中有相应的注释，注意在调整参数的时候要注意参数的类型，在yaml中的参数修改不需要重新编译即可生效，但修改源代码需要重新编译，注意编译时要在AirCraft工作空间下。  
关于使用OpenCV对图像进行预处理的相关参数，可上网查找相关资料了解其具体含义，进行针对性调参

### 自启动
脚本文件放在ac_bringup工作包中的script文件夹下，其中usb.sh脚本文件用于电控通讯时给通讯的usb接口赋予权重；watchdog.sh脚本文件用于自启动，将其加入到Ubuntu的Startup Applications Preference中（复制文件中的第二行命令，注意删除注释符）  
注意，自启动脚本中的路径需要根据代码实际安装位置进行更改