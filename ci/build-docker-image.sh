#! /usr/bin/env bash
#
# Docker image builder
# This script builds the docker image and pushes it to the Docker registry with
# image tag as the branch name of the repository

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

        --ssh-key=* )
            CI_DOCKER_SSH_KEY_PATH="${i#*=}" ;;

        --oauth2_token=* )
            CI_OAUTH2_TOKEN="${i#*=}" ;;

        --ref-name=* )
        # Variable to set the ref name of a git PR/MR: The value is pull for GitHub, BitBucket and merge for GitLab
            CI_REF_NAME="${i#*=}" ;;

        --platforms=* )
        # Docker buildx argument for specifying build architecture: Commonly used architectures are amd64 and arm64
            CI_DOCKER_PLATFORMS="${i#*=}" ;;

        --push_image=* )
        # Docker buildx argument to specify if the built image should be pushed to a registry
            CI_DOCKER_PUSH_IMAGE="${i#*=}" ;;

        --ros_version=* )
            CI_ROS_VERSION="${i#*=}" ;;

        --ros_distro=* )
            CI_ROS_DISTRO="${i#*=}" ;;

        --targets_repo=* )
            CI_TARGETS_REPO="${i#*=}" ;;

        --docker_login=* )
            CI_DOCKER_LOGIN="${i#*=}" ;;

        --docker_file=* )
            CI_DOCKER_FILE="${i#*=}" ;;

        --base_image=* )
            CI_DOCKER_BASE_IMAGE="${i#*=}" ;;

        * )
            # unknown option
            if [[ -n "$i" ]]  # Ignore empty arguments
            then
                echo -e "\e[35;1mUnknown input argument '$i'. Check CI .yml file\e[0m"
                exit 1
            fi ;;
    esac
done

echo -e "\e[35;1mCI_BRANCH             = ${CI_BRANCH}\e[0m"
echo -e "\e[35;1mCI_PULL_REQUEST       = ${CI_PULL_REQUEST}\e[0m"
echo -e "\e[35;1mCI_COMMIT             = ${CI_COMMIT}\e[0m"

[[ -z "$CI_REF_NAME" ]] && CI_REF_NAME="pull"
echo -e "\e[35;1mCI_REF_NAME           = ${CI_REF_NAME}\e[0m"
echo -e "\e[35;1mCI_DOCKER_PLATFORMS   = ${CI_DOCKER_PLATFORMS}\e[0m"
echo -e "\e[35;1mCI_ROS_VERSION        = ${CI_ROS_VERSION}\e[0m"
echo -e "\e[35;1mCI_ROS_DISTRO         = ${CI_ROS_DISTRO}\e[0m"
echo -e "\e[35;1mCI_TARGETS_REPO       = ${CI_TARGETS_REPO}\e[0m"

[[ -z "$CI_DOCKER_LOGIN" ]] && CI_DOCKER_LOGIN="false"
echo -e "\e[35;1mCI_DOCKER_LOGIN       = ${CI_DOCKER_LOGIN}\e[0m"

[[ -z "$CI_DOCKER_BASE_IMAGE" ]] && CI_DOCKER_BASE_IMAGE="ubuntu:22.04"
echo -e "\e[35;1mCI_DOCKER_BASE_IMAGE  = ${CI_DOCKER_BASE_IMAGE}\e[0m"

echo -e "\e[35;1mCI_DOCKER_REGISTRY    = ${CI_DOCKER_REGISTRY}\e[0m"

# Declare arrays for storing the constructed docker build arguments
CI_DOCKER_BUILD_ARGS=()
CI_DOCKER_BUILDX_ARGS=()

# Construction of Docker build arguments begins here
image_dirname="$(dirname "${CI_DOCKER_IMAGE_NAME}")"
image_substring=$(basename "$CI_DOCKER_IMAGE_NAME")
image_name="${image_substring%%:*}"
image_name_tag="${image_substring#*:}"

# Create tag based on branch name
default_branch=$(git symbolic-ref refs/remotes/origin/HEAD --short 2>/dev/null | sed 's@^origin/@@')
[[ -z "$default_branch" ]] && default_branch=$(git remote show origin 2>/dev/null | grep HEAD | awk '{print $3}')

if [[ "$default_branch" == "$CI_BRANCH" ]]
then
    CI_DOCKER_IMAGE_TAG="latest"
    CI_BRANCH=
else
    CI_DOCKER_IMAGE_TAG=$(echo "$CI_BRANCH" | tr '[:upper:]' '[:lower:]' | sed -e 's:/:_:g')
fi

CI_DOCKER_BUILD_ARGS+=("--build-arg=BRANCH=$CI_BRANCH" "--build-arg=PULL_REQUEST=$CI_PULL_REQUEST" "--build-arg=COMMIT=$CI_COMMIT" "--build-arg=CI=$CI" \
    "--build-arg=REF_NAME=$CI_REF_NAME" "--build-arg=BASE_IMAGE=$CI_DOCKER_BASE_IMAGE" "--build-arg=ROS_VERSION=$CI_ROS_VERSION" \
    "--build-arg=ROS_DISTRO=$CI_ROS_DISTRO" "--build-arg=TARGETS_REPO=${CI_TARGETS_REPO}" "--provenance=false")

# Check the constructed Docker image name against the input
image_name_expected="${image_dirname}/${image_name}:${CI_DOCKER_IMAGE_TAG}-${CI_DOCKER_PLATFORMS}"

if [[ -n "${image_name_tag}" ]] && [[ "${image_name_tag}" != "${CI_DOCKER_IMAGE_TAG}-${CI_DOCKER_PLATFORMS}" ]]
then
    echo -e "\e[35;1mError! Docker image name provided as input is not the same as the one generated.\e[0m"
    echo -e "\e[35;1mInput:     ${CI_DOCKER_IMAGE_NAME}\e[0m"
    echo -e "\e[35;1mExpected:  ${image_name_expected}\e[0m"

    # This check of Docker image name to be constructed must not break the pull request pipeline as the created
    # docker image in never pushed in this pipeline
    if [[ "$CI_PULL_REQUEST" == "false" ]]
    then
        exit 1
    fi
fi

CI_DOCKER_IMAGE_NAME="${image_name_expected}"
CI_DOCKER_BUILD_ARGS+=("--tag=${CI_DOCKER_IMAGE_NAME}")

# Make sure a known hosts file exists on the host in the workingdir
if [[ -f ~/.ssh/known_hosts ]]
then
    cp ~/.ssh/known_hosts ./known_hosts
else
    touch ./known_hosts
fi

# Forward ssh-agent of the host
if [[ "$CI_DOCKER_SSH" == "true" ]]
then
    CI_DOCKER_BUILD_ARGS+=("--ssh=default=${CI_DOCKER_SSH_KEY_PATH}")
fi

if [[ -n "${CI_OAUTH2_TOKEN}" ]]
then
    CI_DOCKER_BUILD_ARGS+=("--build-arg=OAUTH2_TOKEN=${CI_OAUTH2_TOKEN}")
fi

# Check if the specified or default Dockerfile exists
if [[ -n "$CI_DOCKER_FILE" ]] && [[ -f "${CI_DOCKER_FILE}" ]]
then
    CI_DOCKER_BUILD_ARGS+=("--file=${CI_DOCKER_FILE}")
elif [[ ! -f "Dockerfile" ]]
then
    echo -e "\e[35;1mError! No Dockerfile found.\e[0m"
    echo -e "\e[35;1mEither provide the filepath to a Dockerfile using the arg --docker_file=/location/to/Dockerfile\e[0m"
    echo -e "\e[35;1mor place a Dockerfile named 'Dockerfile' in the context directory.\e[0m"
    exit 1
fi

# Construction of Docker buildx arguments begins here
if [[ -n "$CI_DOCKER_PLATFORMS" ]]
then
    CI_DOCKER_BUILDX_ARGS+=("--platform=linux/${CI_DOCKER_PLATFORMS}")

    echo -e "\e[35;1mCreating a new docker context for multi-arch builds\e[0m"
    docker context create multiarch-environment
    echo -e "\e[35;1mCreating a new buildx builder for multi-arch builds\e[0m"
    docker buildx create --name multiarchbuilder --driver docker-container --use multiarch-environment
    docker buildx ls
fi

# Push the new Docker image to the Docker registry only after acceptance of pull/merge request
DOCKER_PUSH=false
if [[ "$CI_PULL_REQUEST" == "false" ]]
then
    if [[ "$CI_DOCKER_PUSH_IMAGE" == "true" ]]
    then
        if [[ "${CI_DOCKER_LOGIN}" == "true" ]]
        then
            # Authenticate to the Docker registry
            echo -e "\e[35;1mAuthenticating docker registry $CI_DOCKER_REGISTRY\e[0m"
            echo "$CI_DOCKER_PASSWORD" | docker login "$CI_DOCKER_REGISTRY" -u "$CI_DOCKER_USER" --password-stdin
        fi
        DOCKER_PUSH=true
    fi
else
    if [[ "${CI_DOCKER_PUSH_IMAGE}" == "true" ]]
    then
        echo -e "\e[33;1m--push_image=true with --pull_request=true is not allowed. Docker images are not pushed in a non-branch pipeline\e[0m"
    fi
fi
CI_DOCKER_BUILDX_ARGS+=("--output=type=image,push=${DOCKER_PUSH}")

# Build the Docker image
echo -e "\e[35;1mBuilding docker image: ${CI_DOCKER_IMAGE_NAME} with push=${DOCKER_PUSH}\e[0m"
echo -e "\e[36mExecuting the command: docker buildx build ${CI_DOCKER_BUILDX_ARGS[*]} ${CI_DOCKER_BUILD_ARGS[*]} .\e[0m"
docker buildx build "${CI_DOCKER_BUILDX_ARGS[@]}" "${CI_DOCKER_BUILD_ARGS[@]}" .

# Inspect the manifest file of pushed docker image
if [[ "$DOCKER_PUSH" == "true" ]]
then
    echo -e "\e[35;1mInspecting the pushed image ${CI_DOCKER_IMAGE_NAME}\e[0m"
    docker buildx imagetools inspect "${CI_DOCKER_IMAGE_NAME}"
fi
