#!/bin/bash

if [ ! -f /usr/local/bin/spot ]
then
	sudo bash -c "wget https://raw.githubusercontent.com/guille/spot/master/spot.sh -O /usr/local/bin/spot ; chmod +x /usr/local/bin/spot"
fi
