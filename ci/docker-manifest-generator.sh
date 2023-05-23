#! /usr/bin/env bash
#
# Docker manifest generator
# This script combines the docker images into a new manifest file and pushes it to
# the Docker registry with the architecture stripped from the image tag

# Stop on errors
set -o errexit

# Execute script only in a CI environment
if [[ "$CI" != "true" ]]
then
    echo -e "\e[35;1mError!\e[0m Trying to execute a CI script in a non-CI environment. Exiting script."
    exit 1
fi

for i in "$@"
do
    case $i in
        -i=* | --images-json=* )
            CI_DOCKER_IMAGES_JSON="${i#*=}" ;;

        -t=* | --tag=* )
            CI_DOCKER_IMAGE_TAG="${i#*=}" ;;

        * )
            # unknown option
            if [[ -n "$i" ]]  # Ignore empty arguments
            then
                echo -e "\e[35;1mUnknown input argument '$i'. Check CI .yml file\e[0m"
                exit 1
            fi ;;
    esac
done

[[ -z "${CI_DOCKER_IMAGES_JSON}" ]] && { echo -e "\e[35;1mError! Mandatory arg '--images-json' not set.\e[0m"; exit 1; }
[[ -z "${CI_DOCKER_IMAGE_TAG}" ]] && { echo -e "\e[35;1mError! Mandatory arg '--tag' not set.\e[0m"; exit 1; }

for i in $(jq -rc 'fromjson | .image | to_entries | .[]' <<< "${CI_DOCKER_IMAGES_JSON}")
do
    image_name=$(jq -r '.key'  <<< "${i}")

    TARGET_IMAGE="${image_name}:${CI_DOCKER_IMAGE_TAG}"
    IMAGES_TO_COMBINE=""

    for et in $(jq -r '.value | .[]' <<< "${i}")
    do
        IMAGES_TO_COMBINE="${image_name}:${et} ${IMAGES_TO_COMBINE}"
    done

    echo -e "\e[35;1mExecuting: docker buildx imagetools create -t ${TARGET_IMAGE} ${IMAGES_TO_COMBINE}\e[0m"
    # shellcheck disable=SC2086
    docker buildx imagetools create -t "${TARGET_IMAGE}" ${IMAGES_TO_COMBINE}

    echo -e "\e[35;1mExecuting: docker buildx imagetools inspect ${TARGET_IMAGE}\e[0m"
    docker buildx imagetools inspect "${TARGET_IMAGE}"

done
