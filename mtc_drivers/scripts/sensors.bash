#!/usr/bin/env bash

#MTS_ROBOT_ID="854CAF96103A6853"
#MTS_ROBOT_IP="192.168.68.167"

VAL=$1

echo curl -X PUT -H "Content-Type: application/json" -d {"id": "$MTS_ROBOT_ID", "type": "$VAL"} http://$MTS_ROBOT_IP/sensor
curl -X PUT -H "Content-Type: application/json" -d '{"id": "$MTS_ROBOT_ID","type": "$VAL"}' http://$MTS_ROBOT_IP/sensor
