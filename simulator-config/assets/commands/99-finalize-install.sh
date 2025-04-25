#!/bin/bash

# remove unsued package
apt-get -qq -y autoremove

# check all process are running
ps -edf | grep dcv