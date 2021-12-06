#! /usr/bin/env bash
#
# Docker image builder
# This script builds the docker image and pushes it to the Docker registry with
# image tag as the branch name of the repository

# Stop on errors
set -o errexit

# Execute script only in a CI environment
if [ "$CI" != "true" ]
then
    echo -e "\e[35m\e[1m Error!\e[0m Trying to execute a CI script in a non-CI environment. Exiting script."
    exit 1
fi

for i in "$@"
do
    case $i in
        -i=* | --image=* )
            CI_DOCKER_IMAGE_NAME="${i#*=}" ;;

        -b=* | --branch=* )
        # branch should always be target branch
            CI_BRANCH="${i#*=}" ;;

        -pr=* | --pull_request=* )
            CI_PULL_REQUEST="${i#*=}" ;;

        -c=* | --commit=* )
            CI_COMMIT="${i#*=}" ;;

        -u=* | --user=* )
            CI_DOCKER_USER="${i#*=}" ;;

        -p=* | --password=* )
            CI_DOCKER_PASSWORD="${i#*=}" ;;

        -r=* | --registry=* )
            CI_DOCKER_REGISTRY="${i#*=}" ;;

        --ssh )
            CI_DOCKER_SSH=true ;;

        --ref-name=* )
            CI_REF_NAME="${i#*=}" ;;

        --platforms=* )
            CI_DOCKER_PLATFORMS="${i#*=}" ;;

        --push_image=* )
            CI_DOCKER_PUSH_IMAGE="${i#*=}" ;;

        --ros_version=* )
            CI_ROS_VERSION="${i#*=}" ;;

        --docker_login=* )
            CI_DOCKER_LOGIN="${i#*=}" ;;

        --docker_file=* )
            CI_DOCKER_FILE="${i#*=}" ;;

        * )
            echo -e "Error! Unknown input variable '$i'"
            exit 1 ;;
    esac
done

echo -e "\e[35m\e[1m CI_BRANCH              = ${CI_BRANCH} \e[0m"
echo -e "\e[35m\e[1m CI_PULL_REQUEST        = ${CI_PULL_REQUEST} \e[0m"
echo -e "\e[35m\e[1m CI_COMMIT              = ${CI_COMMIT} \e[0m"

[ -z "$CI_REF_NAME" ] && CI_REF_NAME="pull"
echo -e "\e[35m\e[1m CI_REF_NAME            = ${CI_REF_NAME} \e[0m"
echo -e "\e[35m\e[1m CI_DOCKER_PLATFORMS    = ${CI_DOCKER_PLATFORMS} \e[0m"
echo -e "\e[35m\e[1m CI_ROS_VERSION         = ${CI_ROS_VERSION} \e[0m"

[ -z "$CI_DOCKER_LOGIN" ] && CI_DOCKER_LOGIN="false"
echo -e "\e[35m\e[1m CI_DOCKER_LOGIN        = ${CI_DOCKER_LOGIN} \e[0m"

# Declare arrays for storing the constructed docker build arguments
CI_DOCKER_BUILD_ARGS=()
CI_DOCKER_BUILDX_ARGS=()

# Construction of Docker build arguments begins here
image_dirname="$(dirname "${CI_DOCKER_IMAGE_NAME}")"
image_substring=$(basename "$CI_DOCKER_IMAGE_NAME")
image_name="${image_substring%%:*}"
image_name_tag="${image_substring#*:}"
case $image_name in
    tue-env | ros-* )
        BASE_IMAGE="ubuntu:20.04" ;;
    tue-env-cuda )
        BASE_IMAGE="nvidia/cuda:10.0-cudnn7-devel-ubuntu20.04" ;;
    * )
        echo -e "Error! Unknown image tag subname provided."
        echo -e "Supported names are 'tue-env', 'tue-env-cuda' or 'ros-*'"
        exit 1
esac

# Create tag based on branch name
default_branch=$(git symbolic-ref refs/remotes/origin/HEAD --short 2>/dev/null | sed 's@^origin/@@')
[ -z "$default_branch" ] && default_branch=$(git remote show origin 2>/dev/null | grep HEAD | awk '{print $3}')

if [ "$default_branch" == "$CI_BRANCH" ]
then
    CI_DOCKER_IMAGE_TAG="latest"
    CI_BRANCH=
else
    CI_DOCKER_IMAGE_TAG=$(echo "$CI_BRANCH" | tr '[:upper:]' '[:lower:]' | sed -e 's:/:_:g')
fi

CI_DOCKER_BUILD_ARGS+=("--build-arg=BRANCH=$CI_BRANCH" "--build-arg=PULL_REQUEST=$CI_PULL_REQUEST" "--build-arg=COMMIT=$CI_COMMIT" "--build-arg=CI=$CI" \
    "--build-arg=REF_NAME=$CI_REF_NAME" "--build-arg=BASE_IMAGE=$BASE_IMAGE" "--build-arg=ROS_VERSION=$CI_ROS_VERSION")

# Check the constructed Docker image name against the input
image_name_expected="${image_dirname}/${image_name}:${CI_DOCKER_IMAGE_TAG}-${CI_DOCKER_PLATFORMS}"

if [[ -n "${image_name_tag}" ]] && [[ "${image_name_tag}" != "${CI_DOCKER_IMAGE_TAG}-${CI_DOCKER_PLATFORMS}" ]]; then
    echo -e "\e[35;1mError! Docker image name provided as input is not the same as the one generated.\e[0m"
    echo -e "\e[35;1mInput:     ${CI_DOCKER_IMAGE_NAME}\e[0m"
    echo -e "\e[35;1mExpected:  ${image_name_expected}\e[0m"
    exit 1
fi

CI_DOCKER_IMAGE_NAME="${image_name_expected}"
CI_DOCKER_BUILD_ARGS+=("--tag=$CI_DOCKER_IMAGE_NAME")

# Make sure a known hosts file exists on the host in the workingdir
if [ -f ~/.ssh/known_hosts ]
then
    cp ~/.ssh/known_hosts ./known_hosts
else
    touch ./known_hosts
fi

# Forward ssh-agent of the host
if [ "$CI_DOCKER_SSH" == "true" ]
then
    CI_DOCKER_BUILD_ARGS+=("--ssh=default")
fi

# Check if the specified or default Dockerfile exists
if [[ -n "$CI_DOCKER_FILE" ]] && [[ -f "${CI_DOCKER_FILE}" ]]; then
    CI_DOCKER_BUILD_ARGS+=("--file=${CI_DOCKER_FILE}")
elif [[ ! -f "Dockerfile" ]]; then
    echo -e "\e[35;1mError! No Dockerfile found.\e[0m"
    echo -e "\e[35;1mEither provide the filepath to a Dockerfile using the arg --docker_file=/location/to/Dockerfile\e[0m"
    echo -e "\e[35;1mor place a Dockerfile named 'Dockerfile' in the context directory.\e[0m"
    exit 1
fi

# Construction of Docker buildx arguments begins here
if [ -n "$CI_DOCKER_PLATFORMS" ]
then
    CI_DOCKER_BUILDX_ARGS+=("--platform=linux/${CI_DOCKER_PLATFORMS}")

    echo -e "\e[35m\e[1m Creating a new docker context for multi arch builds \e[0m"
    docker context create multiarch-environment
    echo -e "\e[35m\e[1m Creating a new buildx builder for multi arch builds \e[0m"
    docker buildx create --name multiarchbuilder --driver docker-container --use multiarch-environment
    docker buildx ls
fi

# Push the new Docker image to the Docker registry only after acceptance of pull/merge request
DOCKER_PUSH=false
if [ "$CI_PULL_REQUEST" == "false" ]
then
    if [ "$CI_DOCKER_PUSH_IMAGE" == "true" ]
    then
        if [[ "${CI_DOCKER_LOGIN}" == "true" ]]; then
            # Authenticate to the Docker registry
            echo -e "\e[35m\e[1m Authenticating docker registry $CI_DOCKER_REGISTRY \e[0m"
            echo "$CI_DOCKER_PASSWORD" | docker login "$CI_DOCKER_REGISTRY" -u "$CI_DOCKER_USER" --password-stdin
        fi
        DOCKER_PUSH=true
    fi
else
    if [[ "${CI_DOCKER_PUSH_IMAGE}" == "true" ]]; then
        echo -e "\e[33;1m--push_image=true with --pull_request=true is not allowed. Docker images are not pushed in a non-branch pipeline\e[0m"
    fi
fi
CI_DOCKER_BUILDX_ARGS+=("--output=type=image,push=${DOCKER_PUSH}")

# Build the Docker image
echo -e "\e[35m\e[1m Building docker image: ${CI_DOCKER_IMAGE_NAME} with push=${DOCKER_PUSH} \e[0m"
echo -e "\e[36mExecuting the command: docker buildx build ${CI_DOCKER_BUILDX_ARGS[@]} ${CI_DOCKER_BUILD_ARGS[@]} . \e[0m"
docker buildx build "${CI_DOCKER_BUILDX_ARGS[@]}" "${CI_DOCKER_BUILD_ARGS[@]}" .

# Inspect the manifest file of pushed docker image
if [ "$DOCKER_PUSH" == "true" ]
then
    echo -e "\e[35m\e[1m Inspecting the pushed image ${CI_DOCKER_IMAGE_NAME} \e[0m"
    docker buildx imagetools inspect "${CI_DOCKER_IMAGE_NAME}"
fi
