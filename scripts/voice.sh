#!/bin/bash
#去到科大迅飞库的路径下
cd /home/fshs/catkin_ws/kdxf/Linux_aiui5.5.1059.0000_5ef07848/samples/aiui_sample/build
#启动麦克风监听
rosrun asr_bridge asr_bridge &
#启动握手语音识别节点
rosrun openni2_tracker hello 

wait
exit 0



