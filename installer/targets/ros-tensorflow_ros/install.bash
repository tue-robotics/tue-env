#!/bin/bash

# Make sure python-pip is installed
dpkg -s python-pip &> /dev/null || sudo apt-get install python-pip -y

if [ ! $(pip freeze | grep tensorflow) ]
then
    export TF_BINARY_URL=https://storage.googleapis.com/tensorflow/linux/cpu/tensorflow-0.11.0rc0-cp27-none-linux_x86_64.whl

    # ToDo: use pip target
    echo -e "\nInstalling tensorflow using pip.\n"

    if [[ $(lsb_release -rs) > "16.03" ]] 
    then
        pip install -U $TF_BINARY_URL --user
    else
        sudo pip install -U $TF_BINARY_URL
    fi
else
    echo -e "Tensorflow already installed"
fi

