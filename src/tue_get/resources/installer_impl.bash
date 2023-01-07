#! /usr/bin/env bash

function tue-install-error
{
    echo -e "tue-install-error: $*"
    local return_value
    read -r return_value
    return $(("$return_value"))
}

function tue-install-warning
{
    echo -e "tue-install-warning: $*"
    local return_value
    read -r return_value
    echo -e "return_value: ${return_value}"
    return $(("$return_value"))
}

function tue-install-info
{
    echo -e "tue-install-info: $*"
    local return_value
    read -r return_value
    return $(("$return_value"))
}

function tue-install-debug
{
    echo -e "tue-install-info: $*"
    local return_value
    read -r return_value
    return $(("$return_value"))
}

function tue-install-echo
{
    echo -e "tue-install-echo: $*"
    local return_value
    read -r return_value
    return $(("$return_value"))
}

function tue-install-tee
{
    echo -e "tue-install-tee: $*"
    local return_value
    read -r return_value
    return $(("$return_value"))
}

function tue-install-pipe
{
    local pipefail_old return_code
    pipefail_old=$(set -o | grep pipefail | awk '{printf $2}')
    [ "$pipefail_old" != "on" ] && set -o pipefail # set pipefail if not yet set
    tue-install-echo "$*"
    # Executes the command (all arguments), catch stdout and stderr, red styled, print them directly and to file
    {
        IFS=$'\n' read -r -d '' CAPTURED_STDERR;
        IFS=$'\n' read -r -d '' CAPTURED_STDOUT;
    } < <((printf '\0%s\0' "$("$@")" 1>&2) 2>&1)
    return_code=$?
    [ "$pipefail_old" != "on" ] && set +o pipefail # restore old pipefail setting
    echo -e "tue-install-pipe: ${CAPTURED_STDOUT}^^^${CAPTURED_STDERR}"
    read -r return_value
    if [ "$return_value" != "0" ]
    then
        return $(("$return_value"))
    fi

    return $return_code
}

function tue-install-target-now
{
    echo -e "tue-install-target-now: $*"
    local return_value
    read -r return_value
    return $(("$return_value"))
}

function tue-install-target
{
    echo -e "tue-install-target: $*"
    local return_value
    read -r return_value
    return $(("$return_value"))
}

function tue-install-git
{
    echo -e "tue-install-git: $*"
    local return_value
    read -r return_value
    return $(("$return_value"))
}

function tue-install-apply-patch
{
    echo -e "tue-install-apply-patch: $*"
    local return_value
    read -r return_value
    return $(("$return_value"))
}

function tue-install-cp
{
    echo -e "tue-install-cp: $*"
    local return_value
    read -r return_value
    return $(("$return_value"))
}

function tue-install-add-text
{
    echo -e "tue-install-add-text: $*"
    local return_value
    read -r return_value
    return $(("$return_value"))
}

function tue-install-get-releases
{
    echo -e "tue-install-get-releases: $*"
    local return_value
    read -r return_value
    return $(("$return_value"))
}

function tue-install-system
{
    echo -e "tue-install-system: $*"
    local return_value
    read -r return_value
    return $(("$return_value"))
}

function tue-install-system-now
{
    echo -e "tue-install-system-now: $*"
    local return_value
    read -r return_value
    return $(("$return_value"))
}

function tue-install-apt-get-update
{
    echo -e "tue-install-apt-get-update: $*"
    local return_value
    read -r return_value
    return $(("$return_value"))
}

function tue-install-ppa
{
    echo -e "tue-install-ppa: $*"
    local return_value
    read -r return_value
    return $(("$return_value"))
}

function tue-install-ppa-now
{
    echo -e "tue-install-ppa-now: $*"
    local return_value
    read -r return_value
    return $(("$return_value"))
}

function tue-install-pip
{
    echo -e "tue-install-pip: $*"
    local return_value
    read -r return_value
    return $(("$return_value"))
}

function tue-install-pip-now
{
    echo -e "tue-install-pip-now: $*"
    local return_value
    read -r return_value
    return $(("$return_value"))
}

function tue-install-snap
{
    echo -e "tue-install-snap: $*"
    local return_value
    read -r return_value
    return $(("$return_value"))
}

function tue-install-snap-now
{
    echo -e "tue-install-snap-now: $*"
    local return_value
    read -r return_value
    return $(("$return_value"))
}

function tue-install-gem
{
    echo -e "tue-install-gem: $*"
    local return_value
    read -r return_value
    return $(("$return_value"))
}

function tue-install-gem-now
{
    echo -e "tue-install-gem-now: $*"
    local return_value
    read -r return_value
    return $(("$return_value"))
}
