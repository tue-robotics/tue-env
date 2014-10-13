#!/bin/bash

if [ ! -f /etc/apt/sources.list.d/blahota-texstudio-precise.list ]
then
	sudo apt-add-repository ppa:blahota/texstudio
    sudo apt-get update
fi

tue-install-system texstudio
