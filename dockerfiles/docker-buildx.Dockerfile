# The Dockerfile creates an image based on docker:latest with the docker-buildx
# plugin in it.
#
# Necessary argument:
# - TARGET_PLATFORM
#       This is either of amd64, arm64
# hadolint ignore=DL3006
FROM alpine AS fetcher

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# hadolint ignore=DL3018
RUN apk --no-cache add curl jq

ARG TARGET_PLATFORM
RUN tag_name=$(curl -s https://api.github.com/repos/docker/buildx/releases/latest | jq -r ".tag_name") && \
    filename="buildx-${tag_name}.linux-${TARGET_PLATFORM}" && \
    echo "Going to run: curl -L --output /docker-buildx 'https://github.com/docker/buildx/releases/download/${tag_name}/${filename}'" && \
    curl -L \
    --output /docker-buildx \
    "https://github.com/docker/buildx/releases/download/${tag_name}/${filename}"

RUN chmod a+x /docker-buildx

# hadolint ignore=DL3007
FROM docker:latest

COPY --from=fetcher /docker-buildx /usr/lib/docker/cli-plugins/docker-buildx

RUN docker buildx version
