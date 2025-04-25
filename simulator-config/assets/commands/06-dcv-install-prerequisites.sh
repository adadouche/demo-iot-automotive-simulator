#!/bin/bash

# https://docs.aws.amazon.com/dcv/latest/adminguide/setting-up-installing-linux-prereq.html
systemctl isolate graphical.target
systemctl set-default graphical.target

apt-get -qq -y install ubuntu-desktop \
    gdm3 \
    pulseaudio-utils \
    libssl1.1 \
    mesa-utils \
    xserver-xorg-video-dummy

# resolve "/var/lib/dpkg/info/nice-dcv-server.postinst: 8: dpkg-architecture: not found" when installing dcv-server
apt-get -qq -y install dpkg-dev

pip3 install crudini
crudini --set /etc/gdm3/custom.conf "daemon" "WaylandEnable" "false"

systemctl isolate multi-user.target && systemctl isolate graphical.target