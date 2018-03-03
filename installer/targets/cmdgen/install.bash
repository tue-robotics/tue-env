#!/bin/bash

if [ ! -d ~/src/GPSRCmdGen ]
then
    tue-install-debug "GPSRCmdGen not yet installed"
    echo "Installing GPSRCmdGen to ~/src/GPSRCmdGen"

    echo "Installing dependency of GPSRCmdGen: mono-complete"
    sudo apt-get install --assume-yes mono-complete

    tue-install-git http://github.com/kyordhel/GPSRCmdGen.git ~/src/GPSRCmdGen

    echo "Making GPSRCmdGen"
    cd ~/src/GPSRCmdGen
    make

    tue-install-info "GPSRCmdGen is only pulled and maked once. In the future you have to do this yourself.
               cd ~/src/GPSRCmdGen
               make"
else
    tue-install-debug "GPSRCmdGen already installed"
fi
