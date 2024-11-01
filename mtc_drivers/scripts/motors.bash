#!/usr/bin/env bash

#MTS_ROBOT_ID="854CAF96103A6853"
#MTS_ROBOT_IP="192.168.68.167"

L_PWM=$1
R_PWM=$2

L_TIME=$3
R_TIME=$4

echo curl -X PUT -H "Content-Type: application/json" -d {"id": "$MTS_ROBOT_ID", "l": "$L_PWM", "r": "$R_PWM", "l_time": "$L_TIME", "r_time": "$R_TIME"} http://$MTS_ROBOT_IP/motor
curl -X PUT -H "Content-Type: application/json" -d '{"id": "$MTS_ROBOT_ID", "l": "$L_PWM", "r": "$R_PWM", "l_time": "$L_TIME", "r_time": "$R_TIME"}' http://$MTS_ROBOT_IP/motor
