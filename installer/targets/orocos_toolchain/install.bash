#!/bin/sh

REPOSRC=https://github.com/orocos-toolchain/orocos_toolchain.git
LOCALREPO=~/ros/$TUE_ROS_DISTRO/system/src/orocos_toolchain

# We do it this way so that we can abstract if from just git later on
LOCALREPO_VC_DIR=$LOCALREPO/.git

if [ ! -d $LOCALREPO_VC_DIR ]
then
    git clone --recursive $REPOSRC $LOCALREPO
else
    cd $LOCALREPO
    git pull $REPOSRC
    cd -
fi

# End
