#! /usr/bin/env bash

#  - TUE-GET: tool voor het weergeven van dependencies (a la rospack depends*)
#     - Extra handig: aangeven via welke weg A en B van elkaar afhangen
#     - Bijv: tue-get dep A B --level 1       (Waarbij A en B ook ? of * mogen zijn)

hash xmlstarlet 2> /dev/null || sudo apt-get install --assume-yes -qq xmlstarlet

function _show_dep
{
    [[ "$1" == "ros" ]] && return 0

    [[ "$ROS_ONLY" = "true" && "$1" != ros-* ]] && return 0

    local indent=$3
    local tmp=$4

    if [ -z "$indent" ]
    then
        indent=0
    fi

    local indent_str
    indent_str=$(perl -E 'say "--" x '$indent)

    if [ -n "$2" ]
    then
        tmp="$tmp$indent_str$1\n"
        if [[ "$1" == "$2" ]]
        then
            echo -e "$tmp"
            return
        fi
    else
        if [ -n "$LEVEL" ]
        then
            if [[ $indent > $LEVEL ]]
            then
                return
            fi
        fi

        if [[ "$VERBOSE" = "true" ]]
        then
            #get package version
            pkgdir="$(rospack find "${1//ros-}" 2> /dev/null)"
            packagexml="${pkgdir}/package.xml"
            if [ -f "$packagexml" ]
            then
                version=$(xmlstarlet sel -t -m '//version[1]' -v . -n <"$packagexml")
            else
                version=""
            fi

            if [ -z "$pkgdir" ] || ! githash=$(git -C "$pkgdir" rev-parse --short HEAD 2>&1)
            then
                githash=""
            fi
        fi

        #echo output
        outstr=""
        if [[ "$PLAIN" != "true" ]]
        then
            outstr="$indent_str"  # Add indentation
        fi
        outstr="${outstr}$1"  # Add package
        if [[ "$VERBOSE" = "true" ]]
        then
            outstr="${outstr} ${version} ${githash}"  # Add version
        fi
        echo "$outstr"
    fi

    [ ! -f "$TUE_ENV_DIR"/.env/dependencies/"$1" ] && echo -e "\e[38;1mNo dependency file exists of $1\e[0m" && return 1
    while read -r t
    do
        _show_dep "$t" "$2" "$((indent + 1))" "$tmp" || return 1
    done < "$TUE_ENV_DIR"/.env/dependencies/"$1"
}

# idiomatic parameter and option handling in sh
targets=""
while test $# -gt 0
do
    case "$1" in
        --help|-h)
            # shellcheck disable=SC1078,SC1079
            echo """[tue-get dep] shows dependencies of one target (to another target)

    Usage: tue-get dep TARGET_FROM [ TARGET_TO ]

    Possible options:
        --plain        - Show flat output, usefull for parsing later
        --verbose      - Also show the versions of every package
        --all          - Show the dependencies of all installed targets
        --level        - Show the dependencies with a max depth(ignored, if TARGET_TO is provided)
        --ros-only     - Only show ROS dependencies
"""
            exit 0
            ;;
        --plain) PLAIN=true
            ;;
        --verbose) VERBOSE=true
            ;;
        --all) ALL=true
            ;;
        --level*)
            # shellcheck disable=SC2001
            LEVEL=$(echo "$1" | sed -e 's/^[^=]*=//g')
            ;;
        --ros-only) ROS_ONLY=true
            ;;
        --*) echo "unknown option $1"; exit 1;
            ;;
        *) targets="$targets $1"
            ;;
    esac
    shift
done

# shellcheck disable=SC2086
set -- $targets
if [[ -z "$targets" || "$ALL" = "true" ]]
then
    for t in "$TUE_ENV_DIR"/.env/dependencies/*
    do
        _show_dep "$(basename "$t")" "$2"
    done
else
    _show_dep "$1" "$2"
fi
