#!/bin/bash

sudo kill -9 $(pidof asr_bridge) &
sudo kill -9 $(pidof hello)

wait
exit 0



