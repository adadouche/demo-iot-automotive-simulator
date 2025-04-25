#!/bin/bash

cd /tmp
rm -f /tmp/awscliv2.zip
if (uname -a | grep x86 1>/dev/null); then
    curl -s https://awscli.amazonaws.com/awscli-exe-linux-x86_64.zip -o awscliv2.zip
else
    curl -s https://awscli.amazonaws.com/awscli-exe-linux-aarch64.zip -o awscliv2.zip
fi
unzip -q -o awscliv2.zip
./aws/install --update -b /usr/bin

rm -f /tmp/awscliv2.zip