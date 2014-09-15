#!/bin/bash

if [ ! -f /etc/apt/sources.list.d/webupd8team-sublime-text-3-precise.list ]
then
    sudo add-apt-repository ppa:webupd8team/sublime-text-3
    sudo apt-get update
	~/.tue/installer/targets/sublime3/sublime-package-control.py
fi

tue-install-system sublime-text-installer
