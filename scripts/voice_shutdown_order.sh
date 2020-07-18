#!/bin/bash

sudo kill -9 $(pidof asr_bridge) &
sudo kill -9 $(pidof hello) &
sudo kill -9 $(pidof voice_assistant) &


wait
exit 0



