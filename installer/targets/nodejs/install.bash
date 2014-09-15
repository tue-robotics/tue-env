#!/bin/bash

if [ ! -f /etc/apt/sources.list.d/chris-lea-node_js-precise.list ]
then
	sudo add-apt-repository ppa:chris-lea/node.js
	sudo apt-get update
fi

tue-install-system nodejs
