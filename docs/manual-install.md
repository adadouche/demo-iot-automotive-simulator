# CARLA Simulator Ubuntu 22.04 on physical hardware + Amazon DCV + GPU

You should execute these steps as `root` and expect that you will be using an `ubuntu` user. 

Once you completed the steps below, you will need to reboot for all the changes to take effect.

## Table of Contents

1. [Preparation](#preparation)
1. [APT Install Ubuntu packages](#apt-install-uUbuntu-packages )
1. [AWS CLI](#aws-cli)
1. [Prepare the target user environment](#prepare-the-target-user-environment)
1. [NVIDIA drivers](#nvidia-drivers)
1. [Amazon DCV](#amazon-dcv)
1. [CARLA Simulator](#carla-simulator)
1. [ROS2](#ros2)
1. [Socket CAN](#socket-can)
1. [Breeze](#breeze)
1. [CAN Interactive Generator (CANIGEN)](#can-interactive-generator---canigen)
1. [Firefox](#firefox)
1. [Cleanup & Reboot](#cleanup--reboot)
1. [Verify the setup](#verify-the-setup)


## Preparation

First you will need to set some environement variables and clone the Git repository:

Open a terminal as a **root** user and execute the following commands:

> If you close this terminal, and open anew, you will need to execute these commands again.

> You might need to adjust the OS_USER depending on how you system is configured.

```sh
export CARLA_VERSION=0.9.13
export REPO_URL=https://github.com/adadouche/demo-iot-automotive-simulator
export OS_USER=biga
```

[Bask to the top](#table-of-contents)

## APT Install Ubuntu packages

In the same terminal as a **root** user, execute the following commands:

```sh
add-apt-repository ppa:deadsnakes/ppa -y
apt-get -qq -y update

apt-get -qq -y upgrade

apt-get -qq -y install \
    python3 \
    python3-dev \
    python3-venv \
    python3-distutils \
    python3-pip \
    python3-setuptools \
    python-is-python3 \
    locales \
    software-properties-common \
    git \
    wget \
    tmux \
    unzip \
    tar \
    curl \
    sed \
    jq \
    whois
```

[Bask to the top](#table-of-contents)

## AWS CLI

In the same terminal as a **root** user, execute the following commands:

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
```

For more details about the AWS CLI installation, please check : https://docs.aws.amazon.com/cli/latest/userguide/getting-started-install.html

[Bask to the top](#table-of-contents)

## Prepare the target user environment 

In the same terminal as a **root** user, execute the following commands:

```sh
sudo -H -u ${OS_USER} bash -c "python -m venv ~/.venv-carla"
sudo -H -u ${OS_USER} bash -c "git clone ${REPO_URL} ~/demo-iot-automotive-simulator"
```

[Bask to the top](#table-of-contents)

## NVIDIA drivers

In the same terminal as a **root** user, execute the following commands:

```sh
apt-get -qq -y install ubuntu-drivers-common
apt-get -qq -y install $(nvidia-detector)
nvidia-xconfig --preserve-busid --enable-all-gpus
```

For more details about the NVIDIA drivers installation, please check : https://ubuntu.com/server/docs/nvidia-drivers-installation

[Bask to the top](#table-of-contents)

## Amazon DCV

For more details about the Amazon DCV installation, please check : https://docs.aws.amazon.com/dcv/latest/adminguide/setting-up.html

### Prerequisites 

In the same terminal as a **root** user, execute the following commands:

```sh
systemctl isolate graphical.target
systemctl set-default graphical.target

apt-get -qq -y install \
    ubuntu-desktop \
    gdm3 \
    pulseaudio-utils \
    libssl1.1 \
    mesa-utils \
    xserver-xorg-video-dummy

# resolve "/var/lib/dpkg/info/nice-dcv-server.postinst: 8: dpkg-architecture: not found" when installing dcv-server
apt-get -qq -y install dpkg-dev

python -m pip install crudini
crudini --set /etc/gdm3/custom.conf "daemon" "WaylandEnable" "false"

systemctl isolate multi-user.target && systemctl isolate graphical.target
```

### Installation

In the same terminal as a **root** user, execute the following commands:

```sh
cd /tmp

# https://docs.aws.amazon.com/dcv/latest/adminguide/setting-up-installing-linux-server.html
wget -q https://d1uj6qtbmh3dt5.cloudfront.net/NICE-GPG-KEY
gpg --import NICE-GPG-KEY  

# https://docs.aws.amazon.com/dcv/latest/adminguide/setting-up-installing-linux-server.html#linux-server-install

rm -f /tmp/nice-dcv-*.tgz
if ((uname -a | grep x86 1>/dev/null) && (cat /etc/os-release | grep 22.04 1>/dev/null)); then
    wget -q https://d1uj6qtbmh3dt5.cloudfront.net/nice-dcv-ubuntu2204-x86_64.tgz
    tar -xzf nice-dcv-ubuntu*.tgz && cd nice-dcv-*-x86_64
elif ((uname -a | grep x86 1>/dev/null) && (cat /etc/os-release | grep 18.04 1>/dev/null)); then
    wget -q https://d1uj6qtbmh3dt5.cloudfront.net/nice-dcv-ubuntu1804-x86_64.tgz
    tar -xzf nice-dcv-ubuntu*.tgz && cd nice-dcv-*-x86_64
elif (cat /etc/os-release | grep 18.04 1>/dev/null); then
    wget -q https://d1uj6qtbmh3dt5.cloudfront.net/nice-dcv-ubuntu1804-aarch64.tgz
    tar -xzf nice-dcv-ubuntu*.tgz && cd nice-dcv-*-aarch64
else
    wget -q https://d1uj6qtbmh3dt5.cloudfront.net/nice-dcv-ubuntu2004-x86_64.tgz
    tar -xzf nice-dcv-ubuntu*.tgz && cd nice-dcv-*-x86_64
fi

apt-get -qq -y install ./nice-dcv-server_*.deb
apt-get -qq -y install ./nice-dcv-web-viewer_*.deb
usermod -aG video dcv
apt-get -qq -y install ./nice-xdcv_*.deb
```

### Configuration

In the same terminal as a **root** user, execute the following commands:

```sh
python -m pip install crudini

systemctl stop dcvserver

cp /etc/dcv/dcv.conf /etc/dcv/dcv.conf.original

# https://docs.aws.amazon.com/dcv/latest/adminguide/enable-quic.html
crudini --set /etc/dcv/dcv.conf "security" "no-tls-strict" "true"

crudini --set /etc/dcv/dcv.conf "connectivity" "enable-quic-frontend" "false"

crudini --set /etc/dcv/dcv.conf "connectivity" "quic-listen-endpoints" "['0.0.0.0:8443', '[::]:8443']"
crudini --set /etc/dcv/dcv.conf "connectivity" "quic-port" "8443"
crudini --set /etc/dcv/dcv.conf "connectivity" "web-listen-endpoints" "['0.0.0.0:8443', '[::]:8443']"
crudini --set /etc/dcv/dcv.conf "connectivity" "web-port" "8443"

# session storage: https://docs.aws.amazon.com/dcv/latest/userguide/using-transfer.html
mkdir -p /home/${OS_USER}/DCV-Storage
chown -R ${OS_USER}:${OS_USER} /home/${OS_USER}/DCV-Storage

# https://docs.aws.amazon.com/dcv/latest/adminguide/managing-sessions-start.html#managing-sessions-start-manual
tee /opt/dcv-virtual-session.sh > /dev/null << EOF
#!/bin/bash
dcvUser=${OS_USER}
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

[Bask to the top](#table-of-contents)

## CARLA Simulator

In the same terminal as a **root** user, execute the following commands:

```sh
apt-get -qq -y install \
    libomp5 \
    can-utils \
    socat \
    linux-modules-extra-$(uname -r)

mkdir -p /opt/carla-simulator/
cd /opt/carla-simulator/
wget -q https://carla-releases.s3.us-east-005.backblazeb2.com/Linux/CARLA_${CARLA_VERSION}.tar.gz
tar -xzf /opt/carla-simulator/CARLA_*.tar.gz -C /opt/carla-simulator/
rm /opt/carla-simulator/CARLA_*.tar.gz

chown -R ${OS_USER}:${OS_USER} /opt/carla-simulator

sudo -H -u ${OS_USER} bash <<EOF
source ~/.venv-carla/bin/activate
python -m pip install --upgrade pip
python -m pip install carla==${CARLA_VERSION}
python -m pip install -r /opt/carla-simulator/PythonAPI/examples/requirements.txt

pip install \
    opencv-python \
    evdev
EOF
EOF
```

[Bask to the top](#table-of-contents)

## ROS2

In the same terminal as a **root** user, execute the following commands:

```sh
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

add-apt-repository universe
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

apt-get -qq -y update

apt-get -qq -y -f install ros-galactic-desktop python3-rosdep2 python3-colcon-common-extensions
apt-get -qq -y -f install ros-galactic-ackermann-msgs

sudo -H -u ${OS_USER} bash <<EOF
source ~/.venv-carla/bin/activate

ROS_DISTRO=galactic
pip install  \
    colcon-core \
    colcon-common-extensions \
    colcon-clean \
    empy==3.3.4 \
    catkin_pkg \
    lark \
    transforms3d 
    
export CARLA_ROOT=/opt/carla-simulator/
export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/carla-${CarlaVersion}-py3.7-linux-x86_64.egg:$CARLA_ROOT/PythonAPI/carla
export PYTHONWARNINGS=ignore:::setuptools.command.install,ignore:::setuptools.command.easy_install,ignore:::pkg_resources
source /opt/ros/galactic/setup.bash

mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone --recurse-submodules  https://github.com/astuff/astuff_sensor_msgs.git

cd ~/ros2_ws
colcon build --symlink-install

source ~/ros2_ws/install/setup.bash
rosdep update --include-eol-distros -q
rosdep install --from-paths src --ignore-src -r -y -q

git clone --recurse-submodules  https://github.com/carla-simulator/ros-bridge.git ~/ros2_ws/src/ros-bridge

rosdep update --include-eol-distros -q

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y -q
colcon build --symlink-install

source ~/ros2_ws/install/setup.bash
rosdep install --from-paths src --ignore-src -r -y -q

rosdep update --include-eol-distros -q

EOF
```

[Bask to the top](#table-of-contents)

## Socket CAN

In the same terminal as a **root** user, execute the following commands:

```sh
cat > /usr/bin/setup-socketcan.sh <<'EOF'
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

EOF

cat > /lib/systemd/system/setup-socketcan.service  <<EOF
[Unit]
Description=Setup SocketCAN Service
After=multi-user.target
[Service]
Type=simple
Restart=always
RestartSec=1
ExecStart=/bin/bash /usr/bin/setup-socketcan.sh

[Install]
WantedBy=multi-user.target
EOF

systemctl start setup-socketcan
systemctl enable setup-socketcan
```

[Bask to the top](#table-of-contents)

## Breeze

In the same terminal as a **root** user, execute the following commands:

```sh
apt-get -qq -y install \
    python3-pyqt5 \
    pyqt5-dev-tools\
    qttools5-dev-tools

sudo -H -u ${OS_USER} bash <<EOF
source ~/.venv-carla/bin/activate
pip install pyqt5
EOF
```

[Bask to the top](#table-of-contents)

## CAN Interactive Generator - CANIGEN

In the same terminal as a **root** user, execute the following commands:

```sh
sudo -H -u ${OS_USER} bash <<EOF
source ~/.venv-carla/bin/activate
pip install \
    cantools \
    prompt_toolkit \
    python-can \
    can-isotp
EOF
```

[Bask to the top](#table-of-contents)

## Firefox

In the same terminal as a **root** user, execute the following commands:

```sh
apt-get -qq -y install firefox
```

[Bask to the top](#table-of-contents)

## Cleanup & Reboot

In the same terminal as a **root** user, execute the following commands:

```sh
apt-get -qq -y autoremove
reboot
```

[Bask to the top](#table-of-contents)

## Verify the setup

You can now verify that the setup has been completed sucessfully.

### Amazon DCV

You can verify that all processes are running using the following command:

```sh
ps -edf | grep dcv
```

The outpout should look like the following:

```
dcv        57827       1  0 12:05 ?        00:00:00 /bin/bash /usr/bin/dcvserver -d --service
dcv        57828   57827  0 12:05 ?        00:00:01 /usr/lib/x86_64-linux-gnu/dcv/dcvserver --service
root       57833       1  0 12:05 ?        00:00:00 /bin/bash /opt/dcv-virtual-session.sh
root       57856       1  0 12:05 ?        00:00:00 /bin/bash /usr/bin/dcvsessionlauncher -d
root       57859   57856  0 12:05 ?        00:00:00 /usr/lib/x86_64-linux-gnu/dcv/dcvsessionlauncher
root       57863   57859  0 12:05 ?        00:00:00 /sbin/runuser -l -s /usr/lib/x86_64-linux-gnu/dcv/dcvbash -c /usr/lib/x86_64-linux-gnu/dcv/dcvsessionstarter -w XDG_SESSION_TYPE biga
biga       57884   57863  0 12:05 ?        00:00:00 bash /usr/lib/x86_64-linux-gnu/dcv/dcvsessionstarter
biga       57914   57884  0 12:05 ?        00:00:00 /usr/bin/Xdcv -sessionid demo -auth /run/user/1001/dcv/demo.xauth -displayfd 4 -logfile /var/log/dcv/Xdcv.biga.demo.log -output 800x600+0+0 -output 800x600+800+0 -output 800x600+1600+0 -output 800x600+2400+0 -enabledoutputs 1 -logverbose 3 -dpi 96 -nolisten tcp
biga       57926   57884  0 12:05 ?        00:00:03 /usr/lib/x86_64-linux-gnu/dcv/dcvagent --mode full --session-id demo --display :1 --settings-path /etc/dcv/dcv.conf --log-level info --log-dir /var/log/dcv --log-rotate-at-startup
biga       57933   57884  0 12:05 ?        00:00:00 /bin/sh /etc/dcv/dcvsessioninit demo
ssm-user   70918   11085  0 12:11 pts/1    00:00:00 grep dcv
```

You can also execute the following command to verify that a DCV sessions has been created (as a root user):

```sh
dcv list-sessions
```

The outpout should look like the following:

```
Session: 'demo' (owner:biga type:virtual)
```

### Run the CARLA Simulator with manual control and no CAN integration

In a new terminal as your target user, execute the following commands:

```sh
source ~/.venv-carla/bin/activate
/opt/carla-simulator/CarlaUE4.sh -no-rendering -quality-level=Epic -prefernvidia
```

In a new terminal as your target user, execute the following commands:

```sh
source ~/.venv-carla/bin/activate
cd /opt/carla-simulator/PythonAPI/examples
python manual_control.py
```

![CARLA PythonAPI](../images/carla-manual-conrtrol.png "CARLA PythonAPI")

You can use the arrow to control the car.