#!/bin/bash

if [ ! hash -q bower &> /dev/null ]
then
	sudo -H npm install -g bower
fi



