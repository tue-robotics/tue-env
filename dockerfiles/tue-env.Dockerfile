# syntax = docker/dockerfile:experimental
# ----------------------------------------------------------------
#       Dockerfile to build working Ubuntu image with tue-env
# ----------------------------------------------------------------

# Set default base image to Ubuntu 20.04
ARG BASE_IMAGE=ubuntu:20.04
FROM $BASE_IMAGE

# Build time arguments
# BRANCH is the target branch if in PULL_REQUEST mode else it is the test branch
ARG CI=false
ARG BRANCH=
ARG PULL_REQUEST=false
ARG COMMIT=
ARG REF_NAME=
# Default is empty and gives ROS1, for ROS2 use --build-arg ROS_VERSION=2
ARG ROS_VERSION=
ARG ROS_DISTRO=

# Inform scripts that no questions should be asked and set some environment
# variables to prevent warnings and errors
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    DOCKER=true \
    USER=docker \
    TERM=xterm-256color

# Set default shell to be bash
SHELL ["/bin/bash", "-c"]

# Install commands used in our scripts and standard present on a clean ubuntu
# installation and setup a user with sudo priviledges
RUN apt-get update -qq && \
    echo "resolvconf resolvconf/linkify-resolvconf boolean false" | debconf-set-selections && \
    apt-get install -qq --assume-yes --no-install-recommends apt-transport-https apt-utils bash-completion ca-certificates curl dbus debconf-utils dialog git lsb-release iproute2 iputils-ping net-tools openssh-client psmisc python3-pip resolvconf sudo systemd tzdata wget > /dev/null && \
    # Add defined user
    adduser -u 1000 --disabled-password --gecos "" $USER && \
    usermod -aG sudo $USER && \
    usermod -aG adm $USER && \
    usermod -aG dialout $USER && \
    usermod -aG video $USER && \
    usermod -aG audio $USER && \
    echo "%sudo ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/"$USER"

# Setup the current user and its home directory
USER "$USER"
WORKDIR /home/"$USER"

ADD bootstrap.bash ./bootstrap.bash

RUN mkdir -p -m 0700 ~/.ssh
ADD ./known_hosts ./.ssh/known_hosts
RUN sudo chown 1000:1000 ~/.ssh/known_hosts && sudo chmod 644 ~/.ssh/known_hosts

# Setup tue-env and install target ros
    # Remove interactive check from bashrc, otherwise bashrc refuses to execute
RUN --mount=type=ssh,uid=1000 sed -e s/return//g -i ~/.bashrc && \
    # Set the CI args in the container as docker currently provides no method to
    # remove the environment variables
    # NOTE: The following exports will exist only in this container
    export CI=$CI && \
    export BRANCH=$BRANCH && \
    export PULL_REQUEST=$PULL_REQUEST && \
    export COMMIT=$COMMIT && \
    export REF_NAME=$REF_NAME && \
    # Run the standard installation script
    source bootstrap.bash --ros-version="$ROS_VERSION" --ros-distro="$ROS_DISTRO" && \
    # Make tue-env to be available to the environment
    source ~/.bashrc && \
    # Install Avular Common
    tue-get install avular-common && \
    # Install target ros
    tue-get install ros${ROS_VERSION} --test-depend --branch="$BRANCH" && \
    # Remove temp tue files
    (rm -rf /tmp/tue* > /dev/null || true) && \
    # Show ownership of .tue
    namei -l ~/.tue && \
    # Check git remote origin
    git -C ~/.tue remote -v && \
    # Show the branches of tue-env
    git -C ~/.tue branch && \
    # Remove docker-clean. APT will be able to autocomplete packages now
    sudo rm /etc/apt/apt.conf.d/docker-clean
