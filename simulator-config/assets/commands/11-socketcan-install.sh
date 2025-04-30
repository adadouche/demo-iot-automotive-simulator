#!/bin/bash

mkdir -p /usr/bin
mkdir -p /lib/systemd/system/
cp "${SIMULATOR_CONFIG_DESTINATION}/assets/files/setup-socketcan.sh" "/usr/bin/setup-socketcan.sh"
cp "${SIMULATOR_CONFIG_DESTINATION}/assets/files/setup-socketcan.service" "/lib/systemd/system/setup-socketcan.service"

systemctl start setup-socketcan
systemctl enable setup-socketcan