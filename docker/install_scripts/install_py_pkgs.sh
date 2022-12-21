#!/bin/bash

set -euxo pipefail
apt-get update && apt-get install -y python3-tk libopenblas-dev && \
     rm -rf /var/lib/apt/lists/*
pip install --ignore-installed PyYAML
pip install open3d ipython PyOpenGL freetype-py imageio networkx pyglet trimesh
pip install --upgrade scipy
pip install numpy pyyaml scipy ipython cython matplotlib gym \
    opencv-python tensorboard gitpython pillow cloudpickle \
    colorlog gputil imageio ninja h5py
pip install trimesh iopath fvcore hydra-core pytorch-lightning open3d
TORCH_CUDA_ARCH_LIST="7.5 8.0 8.6+PTX" pip install -U git+https://github.com/NVIDIA/MinkowskiEngine --install-option="--blas=openblas" --install-option="--force_cuda" -v --no-deps
pip install "git+https://github.com/facebookresearch/pytorch3d.git@stable"
pip install wandb hydra-core hydra_colorlog python-dotenv
pip install rl-games==1.1.3
