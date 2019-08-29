#!/bin/sh
rosservice call /naoqi_driver/motion/set_breath_enabled "chain_name: 'Body'
enable: false"
echo "Breath Disabled"
rosservice call /naoqi_driver/tracker/stop_tracker "{}"
echo "Tracker Disabled"
expect -c 'spawn ssh nao@mummer6-eth0.laas.fr "qicli call ALMotion.setExternalCollisionProtectionEnabled All 0"; expect "Password:"; send "mummer@LAAS\r"; interact'
echo "Safety Lock Disabled"
