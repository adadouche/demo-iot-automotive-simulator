#!/bin/bash

sudo -H -u ${CARLA_OS_USER_NAME} bash <<EOF
source ~/.venv-carla/bin/activate
pip install \
    cantools==37.2.0 \
    prompt-toolkit==3.0.31 \
    python-can==4.0.0 \
    can-isotp==1.8
EOF