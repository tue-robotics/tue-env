#!/bin/bash

local SIP_version="4.15.5"

if [ $(lsb_release -sc) == "trusty" ]
then
    tue-install-system python-sip-dev
else

    if command -v sip >/dev/null 2>&1
    then
        # We have SIP, let check is version
        SIP_installed=$(sip -V)
        if [[ "$SIP_installed" == "$SIP_version" ]]
        then
            # SIP is correct version, so don't need to install anything
            return
        fi
    fi
    # We don't have SIP or not the correct version
    SIP_file=~/.tue/installer/targets/ros-python-sip/sip-$SIP_version.tar.gz
    if [ ! -f "$SIP_file" ]
    then
        url="https://downloads.sourceforge.net/project/pyqt/sip/sip-$SIP_version/sip-$SIP_version.tar.gz"
        tue-install-error "Download $url and place it in ~/.tue/installer/targets/ros-python-sip/"
        return -1
    fi
    
    tue-install-info "Installing SIP version: $SIP_version"
    cp $SIP_file /tmp/sip.tar.gz

    tar xvzf /tmp/sip.tar.gz -C /tmp
    cd /tmp/sip-$SIP_version
    python ./configure.py
    make
    sudo make install
fi
