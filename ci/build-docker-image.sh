#! /usr/bin/env bash
#
# Docker image builder
# This script builds the docker image and pushes it to the Docker registry with
# image tag as the branch name of the repository

# Stop on errors
set -o errexit

# Execute script only in a CI environment
if [ "$CI" != "true" ]
then
    echo -e "\e[35m\e[1m Error!\e[0m Trying to execute a CI script in a non-CI environment. Exiting script."
    exit 1
fi

if [ -z "$1" ]
then
    echo -e "Error! Need image tag subname as input."
    exit 1
fi

IMAGE_NAME_SUBSTRING="$1"
case $IMAGE_NAME_SUBSTRING in
    tue-env )
        BASE_IMAGE="ubuntu:18.04" ;;
    tue-env-cuda )
        BASE_IMAGE="nvidia/cuda:10.0-cudnn7-devel-ubuntu18.04" ;;
    * )
        echo -e "Error! Unknown image tag subname provided."
        echo -e "Supported names are 'tue-env' or 'tue-env-cuda'"
        exit 1
esac

# Create tag based on branch name
IMAGE_NAME=tuerobotics/"$IMAGE_NAME_SUBSTRING":$(echo "$TRAVIS_BRANCH" | tr '[:upper:]' '[:lower:]' | sed -e 's:/:_:g')

echo -e "\e[35m\e[1m Creating docker $IMAGE_NAME \e[0m"

# build the Docker image (this will use the Dockerfile in the root of the repo)
docker build --build-arg BRANCH="${TRAVIS_PULL_REQUEST_BRANCH:-$TRAVIS_BRANCH}" --build-arg \
PULL_REQUEST="$TRAVIS_PULL_REQUEST" --build-arg COMMIT="$TRAVIS_COMMIT" --build-arg \
CI="$CI" --build-arg BASE_IMAGE="$BASE_IMAGE" -t "$IMAGE_NAME" .

# push the new Docker image to the Docker registry only after acceptance of pull request
if [ "$TRAVIS_PULL_REQUEST" == "false" ]
then
    # authenticate with the Docker Hub registry
    echo "$DOCKER_HUB_PASSWORD" | docker login -u "$DOCKER_HUB_USERNAME" --password-stdin

    echo -e "\e[35m\e[1m docker push $IMAGE_NAME \e[0m"
    docker push "$IMAGE_NAME"

    echo -e "\e[35m\e[1m Succeeded, see: \e[0m"
    docker images
fi
