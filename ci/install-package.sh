#!/bin/bash
set -o errexit #Stop on errors

for i in "$@"
do
case $i in
    -p=*|--package=*)
    PACKAGE="${i#*=}"
    shift # past argument=value
    ;;
    -b=*|--branch=*)
    BRANCH="${i#*=}"
    shift # past argument=value
    ;;
    -c=*|--commit=*)
    COMMIT="${i#*=}"
    shift # past argument=value
    ;;
    *)
            # unknown option
    ;;
esac
done

echo -e "\e[35m\e[1m PACKAGE     = ${PACKAGE} \e[0m"
echo -e "\e[35m\e[1m BRANCH      = ${BRANCH} \e[0m"
echo -e "\e[35m\e[1m COMMIT PATH = ${COMMIT} \e[0m"

# Determine docker tag if the same branch exists there
IMAGE_BRANCH_NAME=tuerobotics/tue-env:`echo "$BRANCH" | tr '[:upper:]' '[:lower:]' | sed -e 's:/:_:g'`

# Set the default fallback branch to master
IMAGE_MASTER_NAME=tuerobotics/tue-env:master

# Remove any previously started containers
docker stop tue-env
docker rm tue-env

# Run the docker image (user master as fallback)
echo -e "\e[35m\e[1m Trying to fetch docker image: $IMAGE_BRANCH_NAME \e[0m"
if ! docker run --detach --interactive --name tue-env $IMAGE_BRANCH_NAME
then
    echo -e "\e[35m\e[1m Not found, fall back to master branch: $IMAGE_MASTER_NAME \e[0m"
    docker run --detach --interactive --name tue-env $IMAGE_MASTER_NAME 
fi

# Install the pacakge
echo -e "\e[35m\e[1m tue-get install ros-$PACKAGE \e[0m"
docker exec tue-env bash -c "export CI='true'; source /home/amigo/.bashrc; tue-get install ros-$PACKAGE"

echo -e "\e[35m\e[1m Reset package to this commit \e[0m"
docker exec tue-env bash -c "cd ~/ros/kinetic/system/src/$PACKAGE && git reset --hard $COMMIT"
