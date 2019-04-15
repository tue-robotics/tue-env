#! /bin/bash

#  - TUE-GET: tool voor het weergeven van dependencies (a la rospack depends*)
#     - Extra handig: aangeven via welke weg A en B van elkaar afhangen
#     - Bijv: tue-get dep A B --level 1       (Waarbij A en B ook ? of * mogen zijn)

function _show_dep
{
    if [[ "$1" == "ros" ]]
    then
        return
    fi

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
            packagexml="$(rospack find "${1//ros-}" 2> /dev/null)/package.xml"
            if [ -f "$packagexml" ]
            then
                version=$(xmlstarlet sel -t -m '//version[1]' -v . -n <"$packagexml")
                #xmlstarlet ed --inplace --update '//version[1]' -v 0.4.0 $packagexml
            else
                version=""
            fi
        fi

        #echo output
        if [[ "$PLAIN" = "true" ]]
        then
            echo "$1"
        elif [[ "$VERBOSE" = "true" ]]
        then
            echo "$indent_str""$1" "$version"
        else
            echo "$indent_str""$1"
        fi
    fi

    while read -r t
    do
        _show_dep "$t" "$2" "$((indent + 1))" "$tmp"
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
