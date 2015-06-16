#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
RELEASES=`$DIR/get-releases linux64`
LATEST=`echo $RELEASES | head -n 1`
TAG=`echo $LATEST | awk '{ print $1 }'`
URL=`echo $LATEST | awk '{ print $2 }'`

#echo -e "found these versions:\n\n$RELEASES\n"
#echo "latest: $TAG"

if [ -f $TUE_BIN/dashboard/$TAG ]
then
    echo "dashboard: no updates found"
else	
    mkdir -p $TUE_BIN/dashboard
    wget $URL -O- | tar -xzv --directory $TUE_BIN/dashboard
	ln -s $TUE_BIN/dashboard/dashboard $TUE_BIN/tue-dashboard
    touch $TUE_BIN/dashboard/$TAG
fi

VER=$(uname -r)
if [ $VER = "14.04" ]
then
	if ! [ -L /usr/lib/libudev.so.0 ];
	then
		echo "I'm going to symlink libudev.so.1.3.5 to libudev.so.0"
		sudo ln -s /lib/x86_64-linux-gnu/libudev.so.1.3.5 /usr/lib/libudev.so.0
	fi
fi
