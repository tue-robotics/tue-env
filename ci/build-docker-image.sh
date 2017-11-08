#!/bin/bash
set -o errexit #Stop on errors

# create tag based on branch name
export IMAGE_NAME=tuerobotics/tue-env:`echo "$TRAVIS_BRANCH" | tr '[:upper:]' '[:lower:]' | sed -e 's:/:_:g'`

echo -e "\e[35m\e[1m Creating docker $IMAGE_NAME \e[0m"

# build the Docker image (this will use the Dockerfile in the root of the repo)
docker build -t $IMAGE_NAME .

# authenticate with the Docker Hub registry
docker login -u="$DOCKER_HUB_USERNAME" -p="$DOCKER_HUB_PASSWORD"

# push the new Docker image to the Docker registry only after acceptance of pull request
if [[ $TRAVIS_PULL_REQUEST == "false" ]]; 
then
    echo -e "\e[35m\e[1m docker push $IMAGE_NAME \e[0m"
    docker push $IMAGE_NAME
    
    echo -e "\e[35m\e[1m Succeeded, see: \e[0m"
    docker images
fi
