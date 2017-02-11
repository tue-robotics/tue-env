#!/bin/bash

if [ ! -f /etc/apt/sources.list.d/cpick-ubuntu-hub-xenial.list ]
then
    sudo add-apt-repository ppa:cpick/hub
    sudo apt-get update
fi

tue-install-system hub
