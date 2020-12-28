#!/bin/bash

echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections
apt-get update -y > /dev/null
apt-get install -y apt-utils > /dev/null
apt-get install -y build-essential > /dev/null
apt-get install -y pkg-config > /dev/null
apt-get install -y libeigen3-dev > /dev/null
apt-get install -y libboost-all-dev
wget https://github.com/Kitware/CMake/releases/download/v3.19.2/cmake-3.19.2.tar.gz
tar -zxvf cmake-3.19.2.tar.gz
safe_cd cmake-3.19.2
./bootsrap
make
make install
apt-get install -y gcovr > /dev/null
