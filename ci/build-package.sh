#!/bin/bash
# This script can only be ran after the install-package.sh script
set -o errexit #Stop on errors

# Standard argument parsing, example: build-package --package=ros_robot
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

echo -e "\e[35m\e[1m PACKAGE     = ${PACKAGE} \e[0m"

# Compile the package
echo -e "\e[35m\e[1m Compile the package \e[0m"
docker exec tue-env bash -c "export CI='true'; source /home/amigo/.bashrc; cd ~/ros/kinetic/system/src/$PACKAGE && catkin build --this --no-status"

# Run unit tests
echo -e "\e[35m\e[1m Run unit test on this package (catkin run_test --this --no-deps) \e[0m"
docker exec tue-env bash -c "export CI='true'; source /home/amigo/.bashrc; cd ~/ros/kinetic/system/src/$PACKAGE && catkin run_tests --this --no-status --no-deps"
