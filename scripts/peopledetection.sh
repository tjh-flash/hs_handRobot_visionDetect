#!/bin/bash
#去到Nite的库路径下
cd ~/catkin_ws/src/NiTE-Linux-x64-2.2/Redist/

#启动行人检测节点
rosrun openni2_tracker vision_detect

wait
exit 0
