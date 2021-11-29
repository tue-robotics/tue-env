# The Dockerfile creates an image based on docker:latest with the docker-buildx
# plugin in it.
#
# Necessary argument:
# - TARGET_PLATFORM
#       This is either of amd64, arm64
FROM alpine AS fetcher

RUN apk --update add curl jq

ARG TARGET_PLATFORM
RUN tag_name=$(curl -s https://api.github.com/repos/docker/buildx/releases/latest | jq -r ".tag_name") && \
    filename="buildx-${tag_name}.linux-${TARGET_PLATFORM}" && \
    echo "Going to run: curl -L --output /docker-buildx 'https://github.com/docker/buildx/releases/download/${tag_name}/${filename}'" && \
    curl -L \
    --output /docker-buildx \
    "https://github.com/docker/buildx/releases/download/${tag_name}/${filename}"

RUN chmod a+x /docker-buildx

FROM docker:latest

COPY --from=fetcher /docker-buildx /usr/lib/docker/cli-plugins/docker-buildx

RUN docker buildx version
