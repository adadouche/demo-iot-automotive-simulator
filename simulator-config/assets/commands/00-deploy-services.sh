#!/bin/bash

mkdir -p /etc/cfn/
cp '../files/cfn-hup.conf' '/etc/cfn/cfn-hup.conf'
cp '../files/cfn-auto-reloader.conf' '/etc/cfn/hooks.d/cfn-auto-reloader.conf'
cp '../files/setup-socketcan.sh' '/usr/bin/setup-socketcan.sh'
cp '../files/setup-socketcan.service' '/lib/systemd/system/setup-socketcan.service'