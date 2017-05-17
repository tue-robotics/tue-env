#!/bin/bash

if [ ! -f /etc/apt/sources.list.d/vscode.list ]
then
	wget "https://go.microsoft.com/fwlink/?LinkID=760868" -O /tmp/vscode.deb
	sudo dpkg -i /tmp/vscode.deb
	sudo apt-get install -f # Install dependencies
fi