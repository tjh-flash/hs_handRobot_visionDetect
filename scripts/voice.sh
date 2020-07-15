#!/bin/bash
#去到科大迅飞库的路径下
cd ~/catkin_ws/kdxf/Linux_aiui5.5.1059.0000_5ef07848/samples/aiui_sample/build
source ~/.bashrc
#启动麦克风监听
rosrun asr_bridge asr_bridge &
#启动语音过滤节点
rosrun openni2_tracker hello &
#启动语音智能助手
rosrun hsr_robot_voice voice_assistant &

wait
exit 0



