#!/bin/bash
# This script can only be ran after the install-package.sh script

for i in "$@"
do
case $i in
    -p=*|--package=*)
    PACKAGE="${i#*=}"
    shift # past argument=value
    ;;
    *)
            # unknown option
    ;;
esac
done

echo "PACKAGE     = ${PACKAGE}"

# Compile the package
docker exec tue-env bash -c "export CI='true'; source /home/amigo/.bashrc; cd ~/ros/kinetic/system/src/$REPO_NAME && catkin build --this --no-status"
