#!/bin/bash

locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

add-apt-repository universe
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

export DEBIAN_FRONTEND=noninteractive
while apt-get upgrade -y | grep -q "Could not get lock" ; do echo "Waiting for other apt-get instances to exit"; sleep 1; done
while sudo fuser /var/{lib/{dpkg,apt/lists},cache/apt/archives}/lock >/dev/null 2>&1; do echo "Waiting for other apt-get instances to exit"; sleep 1; done
apt-get -qq -y update

apt-get -qq -y -f install ros-galactic-desktop python3-rosdep2 python3-colcon-common-extensions
apt-get -qq -y -f install ros-galactic-ackermann-msgs

sudo -H -u ${CARLA_OS_USER_NAME} bash <<EOF
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
export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/carla-${CARLA_VERSION}-py3.7-linux-x86_64.egg:$CARLA_ROOT/PythonAPI/carla
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