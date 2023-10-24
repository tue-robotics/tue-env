# syntax = docker/dockerfile:experimental
# ----------------------------------------------------------------
#       Dockerfile to build working Ubuntu image with tue-env
# ----------------------------------------------------------------

# Set default base image to Ubuntu 22.04
ARG BASE_IMAGE=ubuntu:22.04
FROM $BASE_IMAGE as base

# ----------------------------------------------------------------
#                           STAGE 1
# ----------------------------------------------------------------
FROM base as builder

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
# Default is empty and will result in the default targets repo
ARG TARGETS_REPO=
ARG CREATE_VENV=false
ARG OAUTH2_TOKEN=

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
    apt-get install -qq --assume-yes --no-install-recommends apt-transport-https apt-utils bash-completion ca-certificates curl dbus debconf-utils dialog git keyboard-configuration lsb-release iproute2 iputils-ping mesa-utils net-tools openssh-client psmisc resolvconf sudo tzdata wget > /dev/null && \
    # Add defined user
    adduser -u 1000 --disabled-password --gecos "" $USER && \
    groupadd -g 109 render && \
    usermod -aG sudo $USER && \
    usermod -aG adm $USER && \
    usermod -aG dialout $USER && \
    usermod -aG video $USER && \
    usermod -aG audio $USER && \
    usermod -aG render $USER && \
    echo "%sudo ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/"$USER"

# Setup the current user and its home directory
USER "$USER"
WORKDIR /home/"$USER"

ADD installer/bootstrap.bash ./bootstrap.bash

RUN mkdir -p -m 0700 ~/.ssh
ADD ./known_hosts ./.ssh/known_hosts
RUN sudo chown 1000:1000 ~/.ssh/known_hosts && sudo chmod 644 ~/.ssh/known_hosts

# Setup Git HTTPS token authentication
RUN { [[ -n "$OAUTH2_TOKEN" ]] && git config --global credential.helper '!f() { printf "%s\n" "username=oauth2" "password=$OAUTH2_TOKEN"; };f'; } || exit 0

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
    source bootstrap.bash --ros-version="${ROS_VERSION}" --ros-distro="${ROS_DISTRO}" --create-virtualenv="${CREATE_VENV}"--targets-repo="${TARGETS_REPO}" && \
    # Make tue-env to be available to the environment
    source ~/.bashrc && \
    # Install target ros
    tue-get install ros${ROS_VERSION} --test-depend --branch="$BRANCH" && \
    # Install target ccache
    tue-get install ccache --test-depend && \
    # Remove temp tue files
    (rm -rf /tmp/tue_* > /dev/null || true) && \
    # Show ownership of .tue
    namei -l ~/.tue && \
    # Check git remote origin
    git -C ~/.tue remote -v && \
    # Show the branches of tue-env
    git -C ~/.tue branch && \
    # Remove docker-clean. APT will be able to autocomplete packages now
    sudo rm /etc/apt/apt.conf.d/docker-clean && \
    # Remove apt cache
    sudo rm -rf /var/lib/apt/lists/*

RUN { [[ -n "$OAUTH2_TOKEN" ]] && git config --global --unset credential.helper; } || exit 0

# ----------------------------------------------------------------
#                           STAGE 2
# ----------------------------------------------------------------
FROM base
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    DOCKER=true \
    USER=docker \
    TERM=xterm-256color
USER "$USER"
WORKDIR /home/"$USER"
COPY --from=builder / /
