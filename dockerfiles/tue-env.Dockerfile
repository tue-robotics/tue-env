# syntax = docker/dockerfile:latest
# ----------------------------------------------------------------
#       Dockerfile to build working Ubuntu image with tue-env
# ----------------------------------------------------------------

# Set default base image to Ubuntu 24.04
ARG BASE_IMAGE=ubuntu:24.04

# ----------------------------------------------------------------
#                           STAGE 1
# ----------------------------------------------------------------
# hadolint ignore=DL3006
FROM ${BASE_IMAGE} AS builder

# Build time arguments
# BRANCH is the target branch if in PULL_REQUEST mode else it is the test branch
ARG CI=false
ARG BRANCH
ARG PULL_REQUEST=false
ARG COMMIT
ARG REF_NAME
# Default is empty and gives ROS1, for ROS2 use --build-arg ROS_VERSION=2
ARG ROS_VERSION
ARG ROS_DISTRO
# Default is empty and will result in the default targets repo
ARG TARGETS_REPO
ARG CREATE_VENV=false
ARG VENV_INCLUDE_SYSTEM_SITE=true
ARG OAUTH2_TOKEN
ARG GITHUB_TOKEN
ENV GITHUB_TOKEN=${GITHUB_TOKEN}

ARG DOCKER_USER=docker
ARG DOCKER_USER_ID=1000

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    DOCKER=true \
    USER=${DOCKER_USER} \
    USER_ID=${DOCKER_USER_ID} \
    TERM=xterm-256color

# Set default shell to be bash
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Install commands used in our scripts and standard present on a clean ubuntu
# installation and setup a user with sudo priviledges
# hadolint ignore=DL3008
RUN apt-get update -qq && \
    echo "resolvconf resolvconf/linkify-resolvconf boolean false" | debconf-set-selections && \
    apt-get install -qq --assume-yes --no-install-recommends apt-transport-https apt-utils bash-completion ca-certificates curl dbus debconf-utils dialog git keyboard-configuration lsb-release iproute2 iputils-ping mesa-utils net-tools openssh-client psmisc resolvconf sudo tzdata wget > /dev/null && \
    # Add defined user
    adduser -u "${USER_ID}" --disabled-password --gecos "" "${USER}" && \
    groupadd -g 109 render && \
    usermod -aG sudo "${USER}" && \
    usermod -aG adm "${USER}" && \
    usermod -aG dialout "${USER}" && \
    usermod -aG video "${USER}" && \
    usermod -aG audio "${USER}" && \
    usermod -aG render "${USER}" && \
    echo "%sudo ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/"${USER}"

# Setup the current user and its home directory
USER "${USER}"
WORKDIR /home/"${USER}"

# hadolint ignore=SC2174
RUN mkdir -p -m 0700 ~/.ssh
COPY --chown=${USER_ID}:${USER_ID} --chmod=644 ./known_hosts ./.ssh/known_hosts
RUN cp ~/.ssh/known_hosts ~/.ssh/known_hosts.bak

# Setup Git HTTPS token authentication
# hadolint ignore=SC2016
RUN { [[ -n "$OAUTH2_TOKEN" ]] && git config --global credential.helper '!f() { printf "%s\n" "username=oauth2" "password=$OAUTH2_TOKEN"; };f'; } || exit 0

# Setup tue-env and install target ros
# hadolint ignore=DL3004,SC1091
RUN --mount=type=ssh,uid=$USER_ID --mount=type=bind,source=installer/bootstrap.bash,target=bootstrap.bash \
    # Remove interactive check from bashrc, otherwise bashrc refuses to execute
    sed -e s/return//g -i ~/.bashrc && \
    # Set the CI args in the container as docker currently provides no method to
    # remove the environment variables
    # NOTE: The following exports will exist only in this container
    export CI=$CI && \
    export BRANCH=$BRANCH && \
    export PULL_REQUEST=$PULL_REQUEST && \
    export COMMIT=$COMMIT && \
    export REF_NAME=$REF_NAME && \
    # Run the standard installation script
    source bootstrap.bash \
    --ros-version="${ROS_VERSION}" \
    --ros-distro="${ROS_DISTRO}" \
    --create-virtualenv="${CREATE_VENV}" \
    --virtualenv-include-system-site-packages="${VENV_INCLUDE_SYSTEM_SITE}" \
    --targets-repo="${TARGETS_REPO}" && \
    # Make tue-env to be available to the environment
    source ~/.bashrc && \
    # Install optional git target
    { tue-get install git --test-depend || true; } && \
    # Install target ccache
    tue-get install ccache --test-depend && \
    # Install target ros
    tue-get install ros${ROS_VERSION} --test-depend --branch="$BRANCH" && \
    # Remove temp tue files
    (rm -rf /tmp/tue_* > /dev/null || true) && \
    # Show ownership of ~/.tue
    echo -e "\e[35mOwner of ~/.tue\e[0m" && \
    namei -l ~/.tue && \
    # Check git remote origin
    echo -e "\e[35mgit -C ~/.tue remote -v\e[0m" && \
    git -C ~/.tue remote -v && \
    # Show the branches of tue-env
    echo -e "\e[35mgit -C ~/.tue branch\e[0m" && \
    git -C ~/.tue branch && \
    # Remove docker-clean. APT will be able to autocomplete packages now
    sudo rm /etc/apt/apt.conf.d/docker-clean && \
    # Remove apt cache
    sudo apt-get clean && \
    sudo rm -rf /var/lib/apt/lists/*

# Restore known_hosts to one provided by the user
RUN mv -f ~/.ssh/known_hosts.bak ~/.ssh/known_hosts

# Remove Git HTTPS token authentication
RUN { [[ -n "$OAUTH2_TOKEN" ]] && git config --global --unset credential.helper; } || exit 0

# ----------------------------------------------------------------
#                           STAGE 2
# ----------------------------------------------------------------
# hadolint ignore=DL3006
FROM ${BASE_IMAGE} as final

ARG DOCKER_USER=docker
ARG DOCKER_USER_ID=1000

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    DOCKER=true \
    USER=${DOCKER_USER} \
    USER_ID=${DOCKER_USER_ID} \
    TERM=xterm-256color

SHELL ["/bin/bash", "-c"]

# Setup the current user and its home directory
USER "${USER}"
WORKDIR /home/"${USER}"

COPY --from=builder / /
