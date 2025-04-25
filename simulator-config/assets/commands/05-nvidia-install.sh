#!/bin/bash

# NVIDIA Drivers installation
add-apt-repository ppa:graphics-drivers/ppa -y
apt-get update

apt-get -qq -y install ubuntu-drivers-common
apt-get -qq -y install nvidia-current nvidia-settings
apt-get -qq -y install $(nvidia-detector)
nvidia-xconfig --preserve-busid --enable-all-gpus