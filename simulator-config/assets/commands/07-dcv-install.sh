#!/bin/bash

cd /tmp

# https://docs.aws.amazon.com/dcv/latest/adminguide/setting-up-installing-linux-server.html
wget -q https://d1uj6qtbmh3dt5.cloudfront.net/NICE-GPG-KEY
gpg --import NICE-GPG-KEY

# https://docs.aws.amazon.com/dcv/latest/adminguide/setting-up-installing-linux-server.html#linux-server-install

rm -f /tmp/nice-dcv-*.tgz
if   ((uname -a | grep x86 1>/dev/null) && (cat /etc/os-release | grep 24.04 1>/dev/null)); then
    wget -q https://d1uj6qtbmh3dt5.cloudfront.net/nice-dcv-ubuntu2404-x86_64.tgz
elif ((uname -a | grep x86 1>/dev/null) && (cat /etc/os-release | grep 22.04 1>/dev/null)); then
    wget -q https://d1uj6qtbmh3dt5.cloudfront.net/nice-dcv-ubuntu2204-x86_64.tgz
elif ((uname -a | grep x86 1>/dev/null) && (cat /etc/os-release | grep 20.04 1>/dev/null)); then
    wget -q https://d1uj6qtbmh3dt5.cloudfront.net/nice-dcv-ubuntu2004-x86_64.tgz
fi

tar -xzf nice-dcv-*.tgz && cd nice-dcv-*-x86_64

apt-get -qq -y install ./nice-dcv-server_*.deb
apt-get -qq -y install ./nice-dcv-web-viewer_*.deb
usermod -aG video dcv
apt-get -qq -y install ./nice-xdcv_*.deb

