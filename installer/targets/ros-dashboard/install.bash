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

