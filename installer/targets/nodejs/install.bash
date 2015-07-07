#!/bin/bash

if [ ! -f /etc/apt/sources.list.d/nodesource.list ]
then
	wget -qO- https://deb.nodesource.com/setup | bash -
fi

tue-install-system nodejs
