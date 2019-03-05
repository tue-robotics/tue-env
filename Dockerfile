# ----------------------------------------------------------------
#       Dockerfile to build working Ubuntu image with tue-env
# ----------------------------------------------------------------

# Set the base image to Ubuntu 16.04
FROM ubuntu:16.04

ARG CI_BUILD_BRANCH=

# Inform scripts that no questions should be asked and set some environment
# variables to prevent warnings and errors
ENV DEBIAN_FRONTEND=noninteractive \
    CI=true \
    LANG=C.UTF-8 \
    DOCKER=true \
    USER=amigo \
    TERM=xterm \
    CI_BRANCH=$CI_BUILD_BRANCH

# Set default shell to be bash
SHELL ["/bin/bash", "-c"]

# Test
RUN echo $CI_BRANCH

# Install commands used in our scripts and standard present on a clean ubuntu
# installation and setup a user with sudo priviledges
RUN apt-get update -qq && \
    apt-get install -qq --assume-yes --no-install-recommends apt-transport-https apt-utils ca-certificates curl dbus dialog git lsb-release sudo wget && \
    # Add amigo user
    adduser --disabled-password --gecos "" $USER && \
    adduser $USER sudo && \
    echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# Setup the current user and its home directory
USER "$USER"
WORKDIR /home/"$USER"

# Setup tue-env and install target ros
    # Remove interactive check from bashrc, otherwise bashrc refuses to execute
RUN sed -e s/return//g -i ~/.bashrc && \
    # Run the standard installation script
    source <(wget -O - https://raw.githubusercontent.com/tue-robotics/tue-env/master/installer/bootstrap.bash) && \
    # Make tue-env to be available to the environment
    source ~/.bashrc && \
    # Set all git repositories to use HTTPS urls (Needed for local image builds)
    tue-env config ros-"$TUE_ROS_DISTRO" use-https && \
    # Install target ros
    tue-get install ros && \
    # Show ownership of .tue
    namei -l ~/.tue && \
    # Check git remote origin
    git -C ~/.tue remote -v

