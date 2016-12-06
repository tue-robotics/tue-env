#!/bin/bash

# Make sure python-pip is installed
dpkg -s python-pip &> /dev/null || sudo apt-get install python-pip -y

if [ ! $(pip freeze | grep tensorflow) ]
then
    export TF_BINARY_URL=https://storage.googleapis.com/tensorflow/linux/cpu/tensorflow-0.11.0rc0-cp27-none-linux_x86_64.whl

    # ToDo: use pip target
    # ToDo: sudo not necessary for 16.04
    echo -e "\nInstalling tensorflow using pip. If you're using Ubuntu 16.04, this bash script probably needs to be updated\n"

    sudo pip install -U $TF_BINARY_URL
else
    echo -e "Tensorflow already installed"
fi

