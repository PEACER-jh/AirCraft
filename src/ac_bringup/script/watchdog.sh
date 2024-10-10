#! /bin/bash
#gnome-terminal -e 'bash -c "/home/nuc/Desktop/AirCraft/src/ac_bringup/script/watchdog.sh" '

echo "nuc" | sudo -S chmod 777 /dev/bus/usb/003/*
echo "nuc" | sudo -s chmod 777 /dev/bus/usb/001/*
echo "nuc" | sudo -s chmod 777 /dev/bus/usb/004/*
echo "nuc" | sudo -s chmod 777 /dev/bus/usb/002/*
gnome-terminal -t "aircraft" -- bash -c "source /opt/ros/foxy/setup.bash;source /home/nuc/Desktop/AirCraft/install/setup.bash;cd /home/nuc/Desktop/AirCraft;ros2 launch ac_bringup bringup.launch.py"

sleep 3

while true
do

camera_pid=$(pgrep -f "usb_cam_node")
classify_pid=$(pgrep -f "classify_node")
solver_pid=$(pgrep -f "solver_node")
usb_pid=$(pgrep -f "usb_node")

# camera_pid=$(ps -f | grep "usb_cam_node" | grep -v grep | awk '{print $2}')
# classify_pid=$(ps -f | grep "classify_node" | grep -v grep | awk '{print $2}')
# solver_pid=$(ps -f | grep "solver_node" | grep -v grep | awk '{print $2}')
# usb_pid=$(ps -f | grep "usb_node" | grep -v grep | awk '{print $2}')

if [ $camera_pid -ne 0 ] && [ $classify_pid -ne 0 ] && [ $solver_pid -ne 0 ] && [ $usb_pid -ne 0 ];
then                 #如果程序PID在，则程序在运行
        echo `date`
        echo "all_pid is exitable!"

else
	echo `date`
        echo "camera_pid isnot exitable!"
        {
        	echo "nuc" | sudo -S kill -9 $classify_pid
        } || 
        {
        	echo "$classify_pid isnot find!"
        }
        {
        	echo "nuc" | sudo -S kill -9 $solver_pid
        } || 
        {
        	echo "$solver_pid isnot find!"
        }
        {
        	echo "nuc" | sudo -S kill -9 $usb_pid
        } || 
        {
        	echo "$usb_pid isnot find!"
        }
        {
        	echo "nuc" | sudo -S kill -9 $camera_pid
        } || 
        {
        	echo "$camera_pid isnot find!"
        }
        echo "nuc" | sudo -S chmod 777 /dev/bus/usb/001/*
        echo "nuc" | sudo -s chmod 777 /dev/bus/usb/002/*
        echo "nuc" | sudo -s chmod 777 /dev/bus/usb/003/*
        echo "nuc" | sudo -s chmod 777 /dev/bus/usb/004/*
        gnome-terminal -t "aircraft" -- bash -c "source /opt/ros/foxy/setup.bash;source /home/nuc/Desktop/AirCraft/install/setup.bash;cd /home/nuc/Desktop/AirCraft;ros2 launch ac_bringup bringup.launch.py"

fi

sleep 3

done

exit 0