# syntax = docker/dockerfile:latest
# ----------------------------------------------------------------
#       Dockerfile to build working Ubuntu image with tue-env
# ----------------------------------------------------------------

# Set default base image to Ubuntu 26.04
ARG BASE_IMAGE=ubuntu:26.04

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

ARG DOCKER_USER=docker
ARG DOCKER_USER_ID=1000
# Host of an Ubuntu archive mirror used during the build, e.g. azure.archive.ubuntu.com on GitHub Actions.
# Default is empty and uses the sources of the base image.
ARG APT_MIRROR

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    DOCKER=true \
    USER=${DOCKER_USER} \
    USER_ID=${DOCKER_USER_ID} \
    TERM=xterm-256color

# Set default shell to be bash
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Use the apt mirror for archive.ubuntu.com and security.ubuntu.com during the build. The original sources are backed
# up and restored at the end of this stage, so the image uses the sources of the base image.
RUN [[ -z "${APT_MIRROR}" ]] || \
    for sources in /etc/apt/sources.list /etc/apt/sources.list.d/ubuntu.sources; \
    do \
        [[ -f "${sources}" ]] || continue; \
        install -D -m 644 "${sources}" /var/backups/apt-mirror/original"${sources}" && \
        sed -i -E "s#http://(archive|security)\.ubuntu\.com/#http://${APT_MIRROR}/#g" "${sources}" && \
        install -D -m 644 "${sources}" /var/backups/apt-mirror/mirrored"${sources}" || exit 1; \
    done

# Install commands used in our scripts and standard present on a clean ubuntu
# installation and setup a user with sudo priviledges
# The apt cache mounts only work when downloaded packages are kept: docker-clean deletes them after every install.
# Only /var/lib/apt/lists is cached, the rest of /var/lib/apt (e.g. extended_states) belongs in the image.
# The cache only saves downloads, `apt-get update` still runs every build, so the installed versions are always current.
# hadolint ignore=DL3008
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    rm -f /etc/apt/apt.conf.d/docker-clean && \
    echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache && \
    # Retry failed downloads, apt before 2.3 (Ubuntu 20.04) doesn't retry by default, newer versions retry 3 times
    echo 'Acquire::Retries "3";' > /etc/apt/apt.conf.d/80-retries && \
    apt-get update -qq && \
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
# hadolint ignore=DL3066
USER "${DOCKER_USER_ID}"
WORKDIR /home/"${USER}"

# hadolint ignore=SC2174
RUN mkdir -p -m 0700 ~/.ssh
COPY --chown=${USER_ID}:${USER_ID} --chmod=644 ./known_hosts ./.ssh/known_hosts
RUN cp ~/.ssh/known_hosts ~/.ssh/known_hosts.bak

# Setup Git HTTPS token authentication
# hadolint ignore=SC2016
RUN --mount=type=secret,id=oauth2_token,env=OAUTH2_TOKEN \
    { [[ -n "$OAUTH2_TOKEN" ]] && git config --global credential.helper '!f() { printf "%s\n" "username=oauth2" "password=$OAUTH2_TOKEN"; };f'; } || exit 0

# Setup tue-env and install target ros
# hadolint ignore=DL3004,SC1091
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    --mount=type=secret,id=oauth2_token,env=OAUTH2_TOKEN \
    --mount=type=secret,id=github_token,env=GITHUB_TOKEN \
    --mount=type=ssh,uid=$USER_ID --mount=type=bind,source=installer/bootstrap.bash,target=bootstrap.bash \
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
    # Drop packages from the apt cache that can no longer be downloaded, so a persisted cache does not keep growing
    sudo apt-get autoclean -qq && \
    # Restore the default of 'apt' to not keep downloaded packages in the final image.
    # docker-clean was already removed in the first stage. APT will be able to autocomplete packages now
    sudo rm /etc/apt/apt.conf.d/keep-cache

# Restore the original apt sources when the apt mirror was used. Fail when the sources were changed in another way
# during the build, as restoring them would undo that change.
# hadolint ignore=DL3004
RUN [[ -z "${APT_MIRROR}" ]] || { \
        for sources in /etc/apt/sources.list /etc/apt/sources.list.d/ubuntu.sources; \
        do \
            [[ -f /var/backups/apt-mirror/original"${sources}" ]] || continue; \
            if ! cmp -s /var/backups/apt-mirror/mirrored"${sources}" "${sources}"; \
            then \
                echo -e "\e[31mError! ${sources} was changed during the build, restoring the original sources would undo that:\e[0m"; \
                diff /var/backups/apt-mirror/mirrored"${sources}" "${sources}"; \
                exit 1; \
            fi; \
            sudo cp /var/backups/apt-mirror/original"${sources}" "${sources}" || exit 1; \
        done && \
        sudo rm -rf /var/backups/apt-mirror; \
    }

# Restore known_hosts to one provided by the user
RUN mv -f ~/.ssh/known_hosts.bak ~/.ssh/known_hosts

# Remove Git HTTPS token authentication
RUN --mount=type=secret,id=oauth2_token,env=OAUTH2_TOKEN \
    { [[ -n "$OAUTH2_TOKEN" ]] && git config --global --unset credential.helper; } || exit 0

# ----------------------------------------------------------------
#                           STAGE 2
# ----------------------------------------------------------------
# hadolint ignore=DL3006
FROM ${BASE_IMAGE} AS final

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
# hadolint ignore=DL3066
USER "${DOCKER_USER_ID}"
WORKDIR /home/"${USER}"

# hadolint ignore=DL3067
COPY --from=builder / /
