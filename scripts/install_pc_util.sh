#!/bin/bash
sudo apt update && sudo apt upgrade -y && \
sudo apt install xsel \
                 x11-apps \
                 tmux \
                 vim \
                 v4l-utils \
                 net-tools \
                 netdiscover \
                 libboost-all-dev \
                 libyaml-cpp-dev \
                 libasio-dev \
                 nlohmann-json3-dev \
                 iputils-ping\
                 ptpd \
                 gstreamer1.0-plugins-good \
                 git -y

                 
