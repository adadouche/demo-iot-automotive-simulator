#!/bin/bash

mkdir -p /etc/cfn/hooks.d
mkdir -p /usr/bin
mkdir -p /lib/systemd/system/
cp "${SIMULATOR_CONFIG_DESTINATION}/assets/files/cfn-hup.conf" "/etc/cfn/cfn-hup.conf"
cp "${SIMULATOR_CONFIG_DESTINATION}/assets/files/cfn-auto-reloader.conf" "/etc/cfn/hooks.d/cfn-auto-reloader.conf"
cp "${SIMULATOR_CONFIG_DESTINATION}/assets/files/setup-socketcan.sh" "/usr/bin/setup-socketcan.sh"
cp "${SIMULATOR_CONFIG_DESTINATION}/assets/files/setup-socketcan.service" "/lib/systemd/system/setup-socketcan.service"