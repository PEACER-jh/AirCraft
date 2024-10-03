#! /bin/bash
#gnome-terminal -e 'bash -c "/home/nuc12/sh/watchdog.sh" '

alias ovenv_activate='source ~/venv/openvino_env/bin/activate'
alias ovrt_set='source /opt/intel/openvino_2022.3.0/setupvars.sh'
# source /opt/intel/openvino_2022.3.0/setupvars.sh
# echo "nuc12" | sudo -S ifconfig can0 down
# echo "nuc12" | sudo -S ip link set can0 type can bitrate 1000000
# echo "nuc12" | sudo -S ifconfig can0 up
# ip -details link show can0
echo "nuc12" | sudo -S chmod 777 /dev/bus/usb/003/*
echo "nuc12" | sudo -s chmod 777 /dev/bus/usb/001/*
echo "nuc12" | sudo -s chmod 777 /dev/bus/usb/004/*
echo "nuc12" | sudo -s chmod 777 /dev/bus/usb/002/*
gnome-terminal -e 'bash -c "source /opt/ros/foxy/setup.bash;source /opt/intel/openvino_2022.3.0/setupvars.sh;source /home/nuc12/Desktop/Viper_Autoaim_oncar/install/setup.bash;cd /home/nuc12/Desktop/Viper_Autoaim_oncar;ros2 launch rmos_bringup normal_aim.launch.py " '

# sleep 6

# gnome-terminal -e 'bash -c "source /opt/ros/foxy/setup.bash;source /home/nuc12/Desktop/RMOS_1.0-master/install/setup.bash;cd /home/nuc12/Desktop/RMOS_1.0-master;ros2 bag record /image_raw /imu_quaternion " '

sleep 3

while true
do

cam_pid=$(ps -ef | grep "daheng_camera" | grep -v grep | awk '{print $2}')
detector_pid=$(ps -ef | grep "basic_detector" | grep -v grep | awk '{print $2}')
processer_pid=$(ps -ef | grep "processer" | grep -v grep | awk '{print $2}')
comm_pid=$(ps -ef | grep "can_comm" | grep -v grep | awk '{print $2}')

if [ $cam_pid -ne 0 || $detector_pid -ne 0 || $processer_pid -ne 0 || $comm_pid -ne 0]
then                 #如果程序PID在，则程序在运行
        echo `date`
        echo "cam_pid is exitable!"

else
	echo `date`
        echo "cam_pid isnot exitable!"
        {
        	echo "nuc" | sudo -S kill -9 $detector_pid
        } || 
        {
        	echo "$detector_pid isnot find!"
        }
        {
        	echo "nuc" | sudo -S kill -9 $processer_pid
        } || 
        {
        	echo "$processer_pid isnot find!"
        }
        {
        	echo "nuc" | sudo -S kill -9 $comm_pid
        } || 
        {
        	echo "$comm_pid isnot find!"
        }
        {
        	echo "nuc" | sudo -S kill -9 $cam_pid
        } || 
        {
        	echo "$cam_pid isnot find!"
        }
        # echo "nuc" | sudo -S ifconfig can0 down
	# echo "nuc" | sudo -S ip link set can0 type can bitrate 1000000
	# echo "nuc" | sudo -S ifconfig can0 up
        # ip -details link show can0
        echo "nuc12" | sudo -S chmod 777 /dev/bus/usb/003/*
        echo "nuc12" | sudo -s chmod 777 /dev/bus/usb/001/*
        echo "nuc12" | sudo -s chmod 777 /dev/bus/usb/004/*
        echo "nuc12" | sudo -s chmod 777 /dev/bus/usb/002/*
	gnome-terminal -e 'bash -c "source /opt/ros/foxy/setup.bash;source /home/nuc12/Desktop/Viper_Autoaim_oncar/install/setup.bash;cd /home/nuc12/Desktop/Viper_Autoaim_oncar;ros2 launch rmos_bringup normal_aim.launch.py " '

fi

sleep 3

done

exit 0