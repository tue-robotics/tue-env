#!/bin/bash

if ! hash bower &> /dev/null
then
    # We need the nodejs package manager
    tue-install-target nodejs

    sudo -H npm install -g bower
fi



