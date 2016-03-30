#!/bin/bash

if [ ! -f /etc/apt/sources.list.d/nodesource.list ]
then
    # increase the amount of file watchers
    echo fs.inotify.max_user_watches=524288 | sudo tee -a /etc/sysctl.conf && sudo sysctl -p

	wget -qO- https://deb.nodesource.com/setup_4.x | sudo bash -
    sudo apt-get install nodejs
fi

tue-install-system nodejs
