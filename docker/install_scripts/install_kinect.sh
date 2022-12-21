#!/bin/bash

set -euxo pipefail

wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/k/k4a-tools/k4a-tools_1.4.1_amd64.deb
wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.4-dev/libk4a1.4-dev_1.4.1_amd64.deb
wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.4/libk4a1.4_1.4.1_amd64.deb
wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4abt1.1/libk4abt1.1_1.1.0_amd64.deb
wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4abt1.1-dev/libk4abt1.1-dev_1.1.0_amd64.deb


export ACCEPT_EULA=Y
dpkg -i libk4a1.4_1.4.1_amd64.deb
dpkg -i libk4a1.4-dev_1.4.1_amd64.deb
apt-get update && apt-get install -q -y --no-install-recommends libsoundio1 && rm -rf /var/lib/apt/lists/*
echo 'libk4abt1.1 libk4abt1.1/accepted-eula-hash string 03a13b63730639eeb6626d24fd45cf25131ee8e8e0df3f1b63f552269b176e38' | debconf-set-selections
dpkg -i libk4abt1.1_1.1.0_amd64.deb
dpkg -i libk4abt1.1-dev_1.1.0_amd64.deb
dpkg -i k4a-tools_1.4.1_amd64.deb
pip install --upgrade pyparsing
pip install pyk4a