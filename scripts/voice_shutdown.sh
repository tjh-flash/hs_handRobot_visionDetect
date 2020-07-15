#!/bin/bash

sudo -S sh /home/de/catkin_ws/src/HS_HandGrasp/openni2_tracker/scripts/voice_shutdown_order.sh << EOF
0
EOF

wait
exit 0



