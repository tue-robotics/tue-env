#! /usr/bin/env bash
#
# Script to select the Docker build

# Execute only inside the Docker
if [ -z "$DOCKER" ]
then
    echo "Error! Script to be sourced inside a Docker image only"
    return 1
fi

# If no PR is being tested, build the standard image
if [ "$CI_PULL_REQUEST" == "false" ]
then
    source <(wget -q -O - https://raw.githubusercontent.com/tue-robotics/tue-env/"$BRANCH"/installer/bootstrap.bash)
else
    [ -d ~/.tue ] && sudo chown -R "$USER":"$USER" ~/.tue && ~/.tue/installer/bootstrap.bash
fi

