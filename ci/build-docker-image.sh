#!/bin/bash
#
# Docker image builder
# This script builds the docker image and pushes it to the Docker registry with
# image tag as the branch name of the repository

# Stop on errors
set -o errexit

# create tag based on branch name
IMAGE_NAME=tueroboticsamigo/tue-env:$(echo "$TRAVIS_BRANCH" | tr '[:upper:]' '[:lower:]' | sed -e 's:/:_:g')

echo -e "\e[35m\e[1m Creating docker $IMAGE_NAME \e[0m"

if [ "$TRAVIS_PULL_REQUEST" != "false" ]
then
    end_tag="##TUE_END"
    begin_tag="##TUE_BEGIN"
    target_file=Dockerfile
    source_text="COPY / ./.tue/"

    # Dynamically modify Dockerfile
    sed -e "/^$end_tag/a $source_text" -e "/^$begin_tag/,/^$end_tag/d" $target_file | tee $target_file.tmp
    mv $target_file.tmp $target_file
fi

# build the Docker image (this will use the Dockerfile in the root of the repo)
docker build --build-arg CI_BUILD_BRANCH="$TRAVIS_BRANCH" --build-arg \
CI_PULL_REQUEST="$TRAVIS_PULL_REQUEST" -t $IMAGE_NAME .

# push the new Docker image to the Docker registry only after acceptance of pull request
if [[ $TRAVIS_PULL_REQUEST == "false" ]]
then
    # authenticate with the Docker Hub registry
    docker login -u="$DOCKER_HUB_USERNAME" -p="$DOCKER_HUB_PASSWORD"

    echo -e "\e[35m\e[1m docker push $IMAGE_NAME \e[0m"
    docker push $IMAGE_NAME

    echo -e "\e[35m\e[1m Succeeded, see: \e[0m"
    docker images
fi
