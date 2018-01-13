#!/bin/bash

if [ ! -d ~/src/GPSRCmdGen ]
then
    tue-install-debug "GPSRCmdGen not yet installed"
    echo "Installing GPSRCmdGen"

    echo "Installing dependency of GPSRCmdGen: mono-complete"
    sudo apt-get install mono-complete

    tue-install-git http://github.com/kyordhel/GPSRCmdGen.git ~/src/GPSRCmdGen

    echo "Making GPSRCmdGen"
    cd ~/src/GPSRCmdGen
    make
else
    tue-install-debug "GPSRCmdGen already installed"
fi
