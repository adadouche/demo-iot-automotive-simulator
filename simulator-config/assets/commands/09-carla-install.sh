#!/bin/bash

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

chown -R ${CARLA_OS_USER_NAME}:${CARLA_OS_USER_NAME} /opt/carla-simulator

sudo -H -u ${CARLA_OS_USER_NAME} bash <<EOF
source ~/.venv-carla/bin/activate
python -m pip install --upgrade pip
python -m pip install carla==${CARLA_VERSION}
python -m pip install -r /opt/carla-simulator/PythonAPI/examples/requirements.txt

python -m pip install \
    opencv-python \
    evdev \
    boto3 \
    webcolors
EOF