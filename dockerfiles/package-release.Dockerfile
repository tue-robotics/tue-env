# syntax = docker/dockerfile:experimental
ARG BASE_IMAGE=
FROM $BASE_IMAGE

# Build time arguments and their default values
ARG CI=false
ARG BRANCH=
ARG PULL_REQUEST=false
ARG COMMIT=
ARG PACKAGE=
ARG BUILD_WS=false

# Add gitlab.com to trusted hosts for SSH
RUN mkdir -p -m 0700 ~/.ssh && ssh-keyscan gitlab.com >> ~/.ssh/known_hosts
# Remove interactive check from bashrc, otherwise bashrc refuses to execute
RUN --mount=type=ssh,uid=1000 \
    --mount=type=bind,source=.tmp/repos,target=/home/docker/avular/repos,readwrite \
    --mount=type=bind,source=.tmp/release,target=/home/docker/avular/system/release,readwrite \
    sudo chown 1000:1000 -R /home/docker/avular/repos && \
    sudo chown 1000:1000 -R /home/docker/avular/system/release && \
    # Remove return statement from .bashrc
    sed -e s/return//g -i ~/.bashrc && \
    # Set the CI args in the container as docker currently provides no method to
    # remove the environment variables
    # NOTE: The following exports will exist only in this layer
    export CI=$CI && \
    export BRANCH=$BRANCH && \
    export PULL_REQUEST=$PULL_REQUEST && \
    export COMMIT=$COMMIT && \
    # Make tue-env available to the intermediate image
    # This step needs to be executed at every RUN step
    source ~/.bashrc && \
    # Install the target
    tue-get install ${PACKAGE} --branch="$BRANCH" && \
    # Make workspace
    if [[ "$BUILD_WS" == "true" ]]; then CI_INSTALL=true tue-make --cmake-args -DBUILD_TESTING=OFF; tue-deb-generate; fi && \
    source ~/.bashrc
