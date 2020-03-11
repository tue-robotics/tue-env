# syntax = docker/dockerfile:experimental
# ----------------------------------------------------------------
#       Dockerfile to build working Ubuntu image with tue-env
# ----------------------------------------------------------------

# Set default base image to Ubuntu 16.04
ARG BASE_IMAGE=ubuntu:16.04
FROM $BASE_IMAGE

# Build time arguments
# BRANCH is the PULL_REQUEST_BRANCH if in PULL_REQUEST mode else it is the
# BUILD_BRANCH
ARG CI=false
ARG BRANCH=master
ARG PULL_REQUEST=false
ARG COMMIT=

# Inform scripts that no questions should be asked and set some environment
# variables to prevent warnings and errors
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    DOCKER=true \
    USER=amigo \
    TERM=xterm

# Set default shell to be bash
SHELL ["/bin/bash", "-c"]

# Install commands used in our scripts and standard present on a clean ubuntu
# installation and setup a user with sudo priviledges
RUN apt-get update -qq && \
    apt-get install -qq --assume-yes --no-install-recommends apt-transport-https apt-utils ca-certificates curl dbus dialog git lsb-release openssh-client sudo tzdata wget > /dev/null && \
    # Add amigo user
    adduser -u 1000 --disabled-password --gecos "" $USER && \
    usermod -aG sudo $USER && \
    usermod -aG adm $USER && \
    echo "%sudo ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/"$USER"

# Setup the current user and its home directory
USER "$USER"
WORKDIR /home/"$USER"

ADD installer/bootstrap.bash ./bootstrap.bash

RUN mkdir -p -m 0700 ~/.ssh && ssh-keyscan gitlab.com >> ~/.ssh/known_hosts && chmod 644 ~/.ssh/known_hosts

# Setup tue-env and install target ros
    # Remove interactive check from bashrc, otherwise bashrc refuses to execute
RUN sed -e s/return//g -i ~/.bashrc && \
    # Set the CI args in the container as docker currently provides no method to
    # remove the environment variables
    # NOTE: The following exports will exist only in this container
    export CI=$CI && \
    export BRANCH=$BRANCH && \
    export PULL_REQUEST=$PULL_REQUEST && \
    export COMMIT=$COMMIT && \
    # Run the standard installation script
    source bootstrap.bash && \
    # Make tue-env to be available to the environment
    source ~/.bashrc && \
    # Set all git repositories to use HTTPS urls (Needed for local image builds)
    tue-env config ros-"$TUE_ROS_DISTRO" use-https && \
    # Install target ros
    tue-get install ros --test-depend && \
    # Show ownership of .tue
    namei -l ~/.tue && \
    # Check git remote origin
    git -C ~/.tue remote -v && \
    # Show the branches of tue-env
    git -C ~/.tue branch
