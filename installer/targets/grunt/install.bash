#!/bin/bash

if [ ! hash -q grunt &> /dev/null ]
then
	sudo -H npm install -g grunt
fi



