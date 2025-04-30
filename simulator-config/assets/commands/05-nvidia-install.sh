#!/bin/bash

# NVIDIA Drivers installation
add-apt-repository ppa:graphics-drivers/ppa -y
apt-get update -qq -y

apt-get -qq -y install ubuntu-drivers-common
apt-get -qq -y install nvidia-settings
apt-get -qq -y install $(nvidia-detector)
nvidia-xconfig --preserve-busid --enable-all-gpus