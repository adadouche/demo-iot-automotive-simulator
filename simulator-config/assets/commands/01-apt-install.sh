#!/bin/bash

export DEBIAN_FRONTEND=noninteractive
while apt-get upgrade -y | grep -q "Could not get lock" ; do echo "Waiting for other apt-get instances to exit"; sleep 1; done
while sudo fuser /var/{lib/{dpkg,apt/lists},cache/apt/archives}/lock >/dev/null 2>&1; do echo "Waiting for other apt-get instances to exit"; sleep 1; done              
add-apt-repository ppa:deadsnakes/ppa -y
apt-get -qq -y update

apt-get -qq -y install \
    python3 \
    python3-venv \
    python3-pip \
    python-is-python3 \
    locales \
    software-properties-common \
    wget \
    tmux \
    unzip \
    tar \
    curl \
    sed \
    jq \
    whois