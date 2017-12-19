#!/bin/bash

pycharm="pycharm-community"
if dpkg-query -W -f='${Status}' $pycharm 2>/dev/null | grep -q "ok installed"
then
    tue-install-debug "Pycharm was installed  by apt, removing it now"
    sudo apt-get remove $pycharm
else
    tue-install-debug "Pycharm was not installed by apt"
fi
