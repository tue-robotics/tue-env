#! /usr/bin/env bash

# Stop on errors
set -o errexit

# Execute script only in a CI environment
if [ "$CI" != "true" ]
then
    echo -e "\e[35m\e[1mError!\e[0m Trying to execute a CI script in a non-CI environment. Exiting script."
    exit 1
fi

# Download latest stable docker version
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add - > /dev/null
sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
sudo apt-get update -qq
sudo apt-get -y -qq -o Dpkg::Options::="--force-confnew" install docker-ce > /dev/null

# This is needed on travis because of https://github.com/moby/moby/issues/39120 and https://github.com/travis-ci/travis-ci/issues/6418
sudo rm /etc/docker/daemon.json

sudo systemctl restart docker

docker --version
