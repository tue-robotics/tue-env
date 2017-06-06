#!/bin/bash

# create tag based on branch name
export IMAGE_NAME=tuerobotics/tue-env:`echo "$TRAVIS_BRANCH" | tr '[:upper:]' '[:lower:]' | sed -e 's:/:_:g'`

echo "$IMAGE_NAME"

# build the Docker image (this will use the Dockerfile in the root of the repo)
docker build -t $IMAGE_NAME .

# authenticate with the Docker Hub registry
docker login -u="$DOCKER_HUB_USERNAME" -p="$DOCKER_HUB_PASSWORD"

# push the new Docker image to the Docker registry only after acceptance of pull request
if [[ $TRAVIS_PULL_REQUEST == “false” ]]; 
then
    docker push $IMAGE_NAME
fi
