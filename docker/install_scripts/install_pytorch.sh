#!/bin/bash

set -euxo pipefail

apt-get update 
apt-get install -y --no-install-recommends libcudnn8=8.2.4.15-1+cuda11.4
apt-mark hold libcudnn8

rm -rf /var/lib/apt/lists/*

# install pytorch
pip install torch torchvision torchaudio
