#!/bin/bash

# Adapted from: https://medium.com/@exesse/cuda-10-1-installation-on-ubuntu-18-04-lts-d04f89287130

# ------------- CUDA -------------

# Remove any NVIDIA traces you may have on your machine
sudo rm /etc/apt/sources.list.d/cuda*
sudo apt remove --autoremove nvidia-cuda-toolkit
sudo apt remove --autoremove nvidia-*

# Setup the CUDA PPA on your system
sudo apt update
sudo add-apt-repository ppa:graphics-drivers

sudo apt-key adv --fetch-keys http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64/7fa2af80.pub
sudo bash -c 'echo "deb http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64 /" > /etc/apt/sources.list.d/cuda.list'
sudo bash -c 'echo "deb http://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1604/x86_64 /" > /etc/apt/sources.list.d/cuda_learn.list'

# Install CUDA 10.2 packages
sudo apt update
sudo apt install -y cuda-10-2

# Specify PATH to CUDA in '.profile' file
sudo bash -c 'echo "
# set PATH for cuda 10.2 installation
if [ -d "/usr/local/cuda-10.2/bin/" ]; then
    export PATH=/usr/local/cuda-10.2/bin\${PATH:+:\${PATH}}
    export LD_LIBRARY_PATH=/usr/local/cuda-10.2/lib64\${LD_LIBRARY_PATH:+:\${LD_LIBRARY_PATH}}
fi
" >> ~/.profile'

# Restart
echo "Need restart"
