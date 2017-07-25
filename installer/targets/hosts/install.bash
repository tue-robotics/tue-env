#!/bin/bash

sed s/hostname/$(hostname)/g $TUE_INSTALL_CURRENT_TARGET_DIR/hosts > /tmp/hosts
if ! cmp /tmp/hosts /etc/hosts
then
    sudo install --verbose --compare --backup /tmp/hosts /etc/hosts
fi

