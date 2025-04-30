#!/bin/bash

apt-get update -qq -y
apt-get -qq -y install \
    libomp5 \
    can-utils \
    socat \
    linux-modules-extra-$(uname -r)

mkdir -p /opt/carla-simulator/
cd /opt/carla-simulator/
wget -q -O CARLA.tar.gz https://tiny.carla.org/carla-${CARLA_VERSION//./-}-linux 
tar -xzf /opt/carla-simulator/CARLA.tar.gz -C /opt/carla-simulator/
rm /opt/carla-simulator/CARLA.tar.gz

chown -R ${CARLA_OS_USER_NAME}:${CARLA_OS_USER_NAME} /opt/carla-simulator

sudo -H -u ${CARLA_OS_USER_NAME} bash <<EOF
source ~/.venv-carla/bin/activate
python -m pip install --upgrade pip -q -q -q
python -m pip install carla==${CARLA_VERSION} -q -q -q
python -m pip install -r /opt/carla-simulator/PythonAPI/examples/requirements.txt -q -q -q

python -m pip install \
    opencv-python \
    evdev \
    boto3 \
    webcolors \
    -q -q -q
EOF