#!/bin/bash
set -euo pipefail

if ! CANIF=`ip link | grep vcan0`; then
    echo "no vcan adapter found, creating..."
    ip link add dev vcan0 type vcan
    ip link set up vcan0
else
    echo "vcan adapter found"
fi

while true; do
    echo -n "`date -R` "
    if ! CANIF=`ip link | grep ' can0'`; then
        echo "no can adapter found"
    else
        if echo ${CANIF} | grep -q LOWER_UP; then
            echo "can adapter is up"
        else
            echo "can adapter is down, setting up..."
            ip link set up can0 txqueuelen 1000 type can bitrate 500000 restart-ms 100
        fi
    fi
    sleep 1
done
