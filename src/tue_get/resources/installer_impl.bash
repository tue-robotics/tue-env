#! /usr/bin/env bash

function tue-install-error
{
    echo -e "tue-install-error: $(echo -e "$*" | tr '\n' '^')"
    local return_value
    read -r return_value
    return $(("$return_value"))
}

function tue-install-warning
{
    echo -e "tue-install-warning: $(echo -e "$*" | tr '\n' '^')"
    local return_value
    read -r return_value
    return $(("$return_value"))
}

function tue-install-info
{
    echo -e "tue-install-info: $(echo -e "$*" | tr '\n' '^')"
    local return_value
    read -r return_value
    return $(("$return_value"))
}

function tue-install-debug
{
    echo -e "tue-install-debug: $(echo -e "$*" | tr '\n' '^')"
    local return_value
    read -r return_value
    return $(("$return_value"))
}

function tue-install-echo
{
    echo -e "tue-install-echo: $(echo -e "$*" | tr '\n' '^')"
    local return_value
    read -r return_value
    return $(("$return_value"))
}

function tue-install-tee
{
    echo -e "tue-install-tee: $(echo -e "$*" | tr '\n' '^')}"
    local return_value
    read -r return_value
    return $(("$return_value"))
}

function tue-install-pipe
{
    local return_code
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
    # shellcheck disable=SC2034
    TUE_INSTALL_PIPE_STDOUT=$CAPTURED_STDOUT

    CAPTURED_STDOUT=$(echo -e "$CAPTURED_STDOUT" | tr '\n' '^')
    CAPTURED_STDERR=$(echo -e "$CAPTURED_STDERR" | tr '\n' '^')
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
    local url targetdir version
    url=$1
    shift
    for i in "$@"
        do
            case $i in
                --target-dir=* )
                    targetdir="${i#*=}"
                    ;;
                --version=* )
                    version="${i#*=}" ;;
                * )
                    tue-install-error "Unknown input variable ${i}" ;;
            esac
        done
    echo -e "tue-install-git: ${url} ${targetdir} ${version}"
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
    local ppa_list
    for ppa in "$@"
    do
        ppa_list="${ppa_list:+${ppa_list} }${ppa// /^}"
    done
    echo -e "tue-install-ppa: ${ppa_list}"
    local return_value
    read -r return_value
    return $(("$return_value"))
}

function tue-install-ppa-now
{
    local ppa_list
    for ppa in "$@"
    do
        ppa_list="${ppa_list:+${ppa_list} }${ppa// /^}"
    done
    echo -e "tue-install-ppa-now: $ppa_list"
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

# TEMP for backward compatibility
function tue-install-pip3
{
    tue-install-pip "$*"
    return $?
}

function tue-install-pip-now
{
    echo -e "tue-install-pip-now: $*"
    local return_value
    read -r return_value
    return $(("$return_value"))
}

# TEMP for backward compatibility
function tue-install-pip3-now
{
    tue-install-pip-now "$*"
    return $?
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
