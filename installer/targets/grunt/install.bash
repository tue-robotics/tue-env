#!/bin/bash

if ! hash grunt &> /dev/null
then
    # We need the nodejs package manager
    tue-install-target nodejs

    sudo -H npm install -g grunt-cli
fi



