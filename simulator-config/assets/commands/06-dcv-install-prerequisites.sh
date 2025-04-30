#!/bin/bash

# https://docs.aws.amazon.com/dcv/latest/adminguide/setting-up-installing-linux-prereq.html
systemctl isolate graphical.target
systemctl set-default graphical.target

apt update -qq -y
apt-get -qq -y install ubuntu-desktop \
    gdm3 \
    pulseaudio-utils \
    mesa-utils \
    xserver-xorg-video-dummy

# resolve "/var/lib/dpkg/info/nice-dcv-server.postinst: 8: dpkg-architecture: not found" when installing dcv-server
apt-get -qq -y install dpkg-dev

apt-get -qq -y install crudini
crudini --set /etc/gdm3/custom.conf "daemon" "WaylandEnable" "false"

systemctl isolate multi-user.target && systemctl isolate graphical.target