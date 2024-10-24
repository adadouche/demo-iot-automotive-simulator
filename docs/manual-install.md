# CARLA Simulator Ubuntu 22.04 on physical hardware + Amazon DCV + GPU

You should execute these steps as `root` and expect that you will be using an `ubuntu` user. 

Once you completed the steps below, you will need to reboot for all the changes to take effect.

## Table of Contents

1. [Preparation](#preparation)
1. [Base Ubuntu packages ](#base-uUbuntu-packages )
1. [AWS CLI](#aws-cli)
1. [NVIDIA drivers](#nvidia-drivers)
1. [Amazon DCV](#amazon-dcv)
1. [CARLA Simulator](#cala-simulator)
1. [ROS2](#ros2)
1. [Socket CAN](#socket-can)
1. [Breeze](#breeze)
1. [CAN Interactive Generator (CANIGEN)](#can-interactive-generator---canigen)
1. [Firefox](#firefox)
1. [Cleanup & Reboot](#cleanup--reboot)


## Preparation

First you will need to set some environement variable and clone the Git repository:

```sh
export CARLA_VERSION=0.9.13
export REPO_URL=https://github.com/aws4embeddedlinux/demo-iot-automotive-simulator

mkdir -p /home/ubuntu/
cd /home/ubuntu/Desktop
git clone ${REPO_URL}
```

[Bask to the top](#table-of-contents)

## Base Ubuntu packages 

```sh
add-apt-repository ppa:deadsnakes/ppa -y
apt-get -qq -y update
apt-get -qq -y install python3
apt-get -qq -y install python3-dev
apt-get -qq -y install python3-venv
apt-get -qq -y install python3-distutils
apt-get -qq -y install python3-pip
apt-get -qq -y install python3-setuptools
apt-get -qq -y install python-is-python3
apt-get -qq -y install locales
apt-get -qq -y install software-properties-common
apt-get -qq -y install wget
apt-get -qq -y install tmux
apt-get -qq -y install unzip
apt-get -qq -y install tar
apt-get -qq -y install curl
apt-get -qq -y install sed
```

[Bask to the top](#table-of-contents)

## AWS CLI

For more details about the AWS CLI installation, please check : https://docs.aws.amazon.com/cli/latest/userguide/getting-started-install.html

```sh
cd /tmp
rm -f /tmp/awscliv2.zip
if (uname -a | grep x86 1>/dev/null); then
    curl https://awscli.amazonaws.com/awscli-exe-linux-x86_64.zip -o awscliv2.zip
else
    curl https://awscli.amazonaws.com/awscli-exe-linux-aarch64.zip -o awscliv2.zip
fi
unzip -q -o awscliv2.zip
./aws/install --update -b /usr/bin

rm -f /tmp/awscliv2.zip
echo "export AWS_CLI_AUTO_PROMPT=on-partial" >> /home/ubuntu/.bashrc
```

[Bask to the top](#table-of-contents)

## NVIDIA drivers

For more details about the NVIDIA drivers installation, please check : https://ubuntu.com/server/docs/nvidia-drivers-installation

```sh
apt-get -qq -y install ubuntu-drivers-common
apt-get -qq -y install $(nvidia-detector)

nvidia-xconfig --preserve-busid --enable-all-gpus
```

[Bask to the top](#table-of-contents)

## Amazon DCV

For more details about the Amazon DCV installation, please check : https://docs.aws.amazon.com/dcv/latest/adminguide/setting-up.html

### Prerequisites 

```sh
systemctl isolate graphical.target
systemctl set-default graphical.target

apt-get -qq -y install ubuntu-desktop
apt-get -qq -y install gdm3
apt-get -qq -y install pulseaudio-utils
apt-get -qq -y install libssl1.1
apt-get -qq -y install mesa-utils
apt-get -qq -y install xserver-xorg-video-dummy

# resolve "/var/lib/dpkg/info/nice-dcv-server.postinst: 8: dpkg-architecture: not found" when installing dcv-server
apt-get -qq -y install dpkg-dev

pip3 install crudini
crudini --set /etc/gdm3/custom.conf "daemon" "WaylandEnable" "false"

# systemctl stop gdm3
# systemctl start gdm3

# rm -rf /etc/X11/XF86Config*

systemctl isolate multi-user.target && systemctl isolate graphical.target
```

### Installation

```sh
cd /tmp

# https://docs.aws.amazon.com/dcv/latest/adminguide/setting-up-installing-linux-server.html
wget -nv https://d1uj6qtbmh3dt5.cloudfront.net/NICE-GPG-KEY
gpg --import NICE-GPG-KEY  

# https://docs.aws.amazon.com/dcv/latest/adminguide/setting-up-installing-linux-server.html#linux-server-install

rm -f /tmp/nice-dcv-*.tgz
if ((uname -a | grep x86 1>/dev/null) && (cat /etc/os-release | grep 22.04 1>/dev/null)); then
    wget -nv https://d1uj6qtbmh3dt5.cloudfront.net/nice-dcv-ubuntu2204-x86_64.tgz
    tar -xvzf nice-dcv-ubuntu*.tgz && cd nice-dcv-*-x86_64
elif ((uname -a | grep x86 1>/dev/null) && (cat /etc/os-release | grep 18.04 1>/dev/null)); then
    wget -nv https://d1uj6qtbmh3dt5.cloudfront.net/nice-dcv-ubuntu1804-x86_64.tgz
    tar -xvzf nice-dcv-ubuntu*.tgz && cd nice-dcv-*-x86_64
elif (cat /etc/os-release | grep 18.04 1>/dev/null); then
    wget -nv https://d1uj6qtbmh3dt5.cloudfront.net/nice-dcv-ubuntu1804-aarch64.tgz
    tar -xvzf nice-dcv-ubuntu*.tgz && cd nice-dcv-*-aarch64
else
    wget -nv https://d1uj6qtbmh3dt5.cloudfront.net/nice-dcv-ubuntu2004-x86_64.tgz
    tar -xvzf nice-dcv-ubuntu*.tgz && cd nice-dcv-*-x86_64
fi

apt-get -qq -y install ./nice-dcv-server_*.deb
apt-get -qq -y install ./nice-dcv-web-viewer_*.deb
usermod -aG video dcv
apt-get -qq -y install ./nice-xdcv_*.deb

# dcvstartx &
# pkill dcvstartx
```

### Configuration

```sh
pip3 install crudini

systemctl stop dcvserver

cp /etc/dcv/dcv.conf /etc/dcv/dcv.conf.original

# https://docs.aws.amazon.com/dcv/latest/adminguide/enable-quic.html
crudini --set /etc/dcv/dcv.conf "security" "no-tls-strict" "true"

crudini --set /etc/dcv/dcv.conf "connectivity" "enable-quic-frontend" "false"
# crudini --set /etc/dcv/dcv.conf "connectivity" "enable-datagrams-display" "always-off"

crudini --set /etc/dcv/dcv.conf "connectivity" "quic-listen-endpoints" "['0.0.0.0:8443', '[::]:8443']"
crudini --set /etc/dcv/dcv.conf "connectivity" "quic-port" "8443"
crudini --set /etc/dcv/dcv.conf "connectivity" "web-listen-endpoints" "['0.0.0.0:8443', '[::]:8443']"
crudini --set /etc/dcv/dcv.conf "connectivity" "web-port" "8443"

# crudini --set /etc/dcv/dcv.conf "session-management" "create-session" "false"
# crudini --set /etc/dcv/dcv.conf "session-management/automatic-console-session" "owner" "ubuntu"

# session storage: https://docs.aws.amazon.com/dcv/latest/userguide/using-transfer.html
mkdir -p /home/ubuntu/DCV-Storage
chown -R ubuntu:ubuntu /home/ubuntu/DCV-Storage

# https://docs.aws.amazon.com/dcv/latest/adminguide/managing-sessions-start.html#managing-sessions-start-manual
tee /opt/dcv-virtual-session.sh > /dev/null << EOF
#!/bin/bash
dcvUser=ubuntu
while true;
do
    if (/usr/bin/dcv list-sessions | grep \$dcvUser 1>/dev/null)
    then
    sleep 5
    else
    /usr/bin/dcv create-session demo --owner \$dcvUser --storage-root /home/\$dcvUser/DCV-Storage --type=virtual 
    /usr/bin/dcv list-sessions
    fi
done
EOF

tee /etc/systemd/system/dcv-virtual-session.service > /dev/null << EOF
[Unit]
Description=Create DCV virtual session for user ubuntu
After=default.target network.target
[Service]
ExecStart=/opt/dcv-virtual-session.sh
[Install]
WantedBy=default.target
EOF

chmod +x /opt/dcv-virtual-session.sh

# text console: DCV virtual sessions only
systemctl daemon-reload
systemctl enable --now dcvserver
systemctl enable --now dcv-virtual-session

systemctl stop dcvserver
systemctl stop dcv-virtual-session

systemctl restart dcvserver
systemctl restart dcv-virtual-session

systemctl status dcvserver
systemctl status dcv-virtual-session
```


### Verify the setup

#### Processes

You can verify that all processes are running using the following command:

```sh
ps -edf | grep dcv
```

The outpout should look like the following:

```

```

#### DCV Sessions

You can verify that the Amazon DCV:

```sh
dcv list-sessions
```

The outpout should look like the following:

```

```

[Bask to the top](#table-of-contents)

## CARLA Simulator

```sh
apt-get -qq -y install libomp5
apt-get -qq -y install can-utils
apt-get -qq -y install socat
apt-get -qq -y install linux-modules-extra-$(uname -r)

mkdir -p /opt/carla-simulator/
cd /opt/carla-simulator/
wget https://carla-releases.s3.us-east-005.backblazeb2.com/Linux/CARLA_${CARLA_VERSION}.tar.gz
tar -xzvf /opt/carla-simulator/CARLA_*.tar.gz -C /opt/carla-simulator/

python -m pip install --upgrade pip
python -m pip install --ignore-installed carla==${CARLA_VERSION}
python -m pip install --ignore-installed -r /opt/carla-simulator/PythonAPI/examples/requirements.txt

chown -R "ubuntu:ubuntu" /opt/carla-simulator
```

[Bask to the top](#table-of-contents)

## ROS2

```sh
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

add-apt-repository universe
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

apt-get -qq -y update

apt-get -qq -y install ros-galactic-desktop
apt-get -qq -y install ros-dev-tools

apt-get -qq -y install python3-rosdep2
apt-get -qq -y install python3-colcon-common-extensions

export CARLA_ROOT=/opt/carla-simulator/
export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/carla-${CARLA_VERSION}-py3.7-linux-x86_64.egg:$CARLA_ROOT/PythonAPI/carla
source /opt/ros/galactic/setup.bash

mkdir -p /home/ubuntu/ros2_ws/src 
cd /home/ubuntu/ros2_ws/src
git clone https://github.com/astuff/astuff_sensor_msgs.git

cd /home/ubuntu/ros2_ws
colcon build --symlink-install
source /home/ubuntu/ros2_ws/install/setup.bash
rosdep install --from-paths src --ignore-src -r -y

cd /home/ubuntu/ros2_ws/src
git clone https://github.com/carla-simulator/ros-bridge.git ros-bridge
cd /home/ubuntu/ros2_ws/src/ros-bridge
git submodule update --init --recursive

rosdep update

cd /home/ubuntu/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install

chown -R "ubuntu:ubuntu" /home/ubuntu/ros2_ws
```

[Bask to the top](#table-of-contents)

## Socket CAN

```sh
# cloning the repository
mkdir -p /home/ubuntu/
cd /home/ubuntu/Desktop
git clone ${REPO_URL}

cd /home/ubuntu/Desktop/demo-iot-automotive-simulator/socket-can-setup
sudo cp setup-socketcan.sh /usr/bin
sudo cp setup-socketcan.service /lib/systemd/system

systemctl start setup-socketcan
systemctl enable setup-socketcan

chown -R "ubuntu:ubuntu" /home/ubuntu/Desktop
```

[Bask to the top](#table-of-contents)

## Breeze

```sh
cd /home/ubuntu/Desktop/demo-iot-automotive-simulator/socket-can-setup
sudo cp setup-socketcan.sh /usr/bin
sudo cp setup-socketcan.service /lib/systemd/system

systemctl start setup-socketcan
systemctl enable setup-socketcan

chown -R "ubuntu:ubuntu" /home/ubuntu/Desktop
```

[Bask to the top](#table-of-contents)

## CAN Interactive Generator - CANIGEN

```sh
python -m pip install --ignore-installed cantools                 
python -m pip install --ignore-installed prompt_toolkit
python -m pip install --ignore-installed python-can
python -m pip install --ignore-installed can-isotp  
```

[Bask to the top](#table-of-contents)

## Firefox

```sh
apt-get -qq -y install firefox
```

[Bask to the top](#table-of-contents)

## Cleanup & Reboot

```sh
apt-get -qq -y install firefox
```

[Bask to the top](#table-of-contents)