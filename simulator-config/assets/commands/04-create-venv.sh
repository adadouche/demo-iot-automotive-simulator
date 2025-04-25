#!/bin/bash

sudo -H -u ${CARLA_OS_USER_NAME} bash -c "python -m venv ~/.venv-carla"
sudo -H -u ${CARLA_OS_USER_NAME} bash -c "git clone ${SIMULATOR_REPOSITORY_URL} ~/demo-iot-automotive-simulator"