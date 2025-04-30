#!/bin/bash

apt-get -qq -y install \
    python3-pyqt5 \
    pyqt5-dev-tools\
    qttools5-dev-tools
sudo -H -u ${CARLA_OS_USER_NAME} bash <<EOF
source ~/.venv-carla/bin/activate
pip install pyqt5 -q -q -q
EOF