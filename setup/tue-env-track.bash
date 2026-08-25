#! /usr/bin/env bash

# ----------------------------------------------------------------------------------------------------
#                                  ENVIRONMENT CHANGE TRACKING
# ----------------------------------------------------------------------------------------------------
# Records what loading a tue-env environment does to this shell, so that unloading it undoes exactly
# that and nothing else.
# Design: docs/superpowers/specs/2026-08-21-env-change-tracking-design.md
#
# Naming rule: every global here is named __TUE_ENV_*, every local __tue_env_*. A snapshot taken from
# inside a function sees the locals of every enclosing function, because bash scopes dynamically, so
# both prefixes are excluded from tracking. A local without the prefix ends up in the ledger.
#
# Never use `(( x++ ))` and never end a function with a bare `cond && action`: both can return 1, and
# a caller running under `set -e` would abort.
# ----------------------------------------------------------------------------------------------------

# Record, field and pair separators. Newlines cannot be used, `declare -p` emits literal newlines
# inside values; NUL cannot be used, command substitution discards it.
__TUE_ENV_RS=$'\x1e'
__TUE_ENV_FS=$'\x1f'
__TUE_ENV_PS=$'\x1d'

# Extra glob patterns of names never to track. Empty in production; the test harness uses it to hide
# its own bookkeeping.
if ! declare -p __TUE_ENV_TRACK_EXTRA_EXCLUDE > /dev/null 2>&1
then
    declare -ga __TUE_ENV_TRACK_EXTRA_EXCLUDE=()
fi

# The ledger. Guarded, because setup.bash is re-sourced on every `tue-env switch` and on every
# `source ~/.bashrc`, and a second load has to merge into the ledger the first one built.
if ! declare -p __TUE_ENV_LEDGER_VAR > /dev/null 2>&1
then
    declare -gA __TUE_ENV_LEDGER_VAR=() __TUE_ENV_LEDGER_VAR_PRE=()
    declare -gA __TUE_ENV_LEDGER_VAR_POST=() __TUE_ENV_LEDGER_VAR_ADD=()
    declare -gA __TUE_ENV_LEDGER_FUNC=() __TUE_ENV_LEDGER_FUNC_PRE=()
    declare -gA __TUE_ENV_LEDGER_FUNC_POST=() __TUE_ENV_LEDGER_FUNC_XPRE=()
    declare -gA __TUE_ENV_LEDGER_ALIAS=() __TUE_ENV_LEDGER_ALIAS_PRE=()
    declare -gA __TUE_ENV_LEDGER_ALIAS_POST=()
    declare -gA __TUE_ENV_LEDGER_COMPLETE=() __TUE_ENV_LEDGER_COMPLETE_PRE=()
    declare -gA __TUE_ENV_LEDGER_COMPLETE_POST=()
    declare -gi __TUE_ENV_TRACK_DEPTH=0
fi

# Transient snapshot targets, cleared at the end of every commit.
declare -gA __TUE_ENV_PRE_VAR=() __TUE_ENV_PRE_FUNC=() __TUE_ENV_PRE_FUNCX=()
declare -gA __TUE_ENV_PRE_ALIAS=() __TUE_ENV_PRE_COMPLETE=()
declare -gA __TUE_ENV_POST_VAR=() __TUE_ENV_POST_FUNC=() __TUE_ENV_POST_FUNCX=()
declare -gA __TUE_ENV_POST_ALIAS=() __TUE_ENV_POST_COMPLETE=()

# ----------------------------------------------------------------------------------------------------
#                                          CAPTURE
# ----------------------------------------------------------------------------------------------------

function __tue_env_track_excluded
{
    # $1: name. Returns 0 when the name must not be tracked.
    local __tue_env_p
    case "$1" in
        BASH* | COLUMNS | COMP_* | DIRSTACK | EPOCH* | FUNCNAME | GROUPS | HISTCMD | LINENO | LINES | \
        OLDPWD | PIPESTATUS | PWD | RANDOM | SECONDS | SHELLOPTS | SHLVL | SRANDOM | _ | \
        __TUE_ENV_* | __tue_env_* )
            return 0 ;;
    esac
    for __tue_env_p in "${__TUE_ENV_TRACK_EXTRA_EXCLUDE[@]}"
    do
        # shellcheck disable=SC2053
        if [[ "$1" == ${__tue_env_p} ]]
        then
            return 0
        fi
    done
    return 1
}

function __tue_env_track_dump
{
    # Writes a snapshot of this shell to stdout, framed as described at the top of the plan. Meant to
    # be called inside a command substitution; the few substitutions below are per category, never per
    # name, so the cost of a snapshot does not grow with the size of the environment.
    #
    # The local IFS pins field-splitting to its default regardless of what the caller set it to, which
    # is what keeps the `declare -Fx` read loop below correct; the two `compgen` name lists are read
    # with `mapfile` instead of a split, so they are immune to IFS and to globbing either way.
    local IFS=$' \t\n'
    local __tue_env_n __tue_env_names __tue_env_l __tue_env_d1 __tue_env_d2
    local -a __tue_env_list=()
    local -A __tue_env_xf=()

    # Which functions carry `export -f`; `declare -f` output does not encode it.
    __tue_env_names="$(declare -Fx)"
    while read -r __tue_env_d1 __tue_env_d2 __tue_env_n
    do
        if [[ -n "${__tue_env_n}" ]]
        then
            __tue_env_xf["${__tue_env_n}"]="x"
        fi
    done <<< "${__tue_env_names}"

    mapfile -t __tue_env_list < <(compgen -v)
    for __tue_env_n in "${__tue_env_list[@]}"
    do
        __tue_env_track_excluded "${__tue_env_n}" && continue
        printf 'V%s%s%s' "${__TUE_ENV_FS}" "${__tue_env_n}" "${__TUE_ENV_FS}"
        declare -p "${__tue_env_n}" 2> /dev/null
        printf '%s' "${__TUE_ENV_RS}"
    done

    mapfile -t __tue_env_list < <(compgen -A function)
    for __tue_env_n in "${__tue_env_list[@]}"
    do
        __tue_env_track_excluded "${__tue_env_n}" && continue
        printf 'F%s%s%s%s%s' "${__TUE_ENV_FS}" "${__tue_env_n}" "${__TUE_ENV_FS}" \
               "${__tue_env_xf[${__tue_env_n}]:-}" "${__TUE_ENV_FS}"
        declare -f "${__tue_env_n}"
        printf '%s' "${__TUE_ENV_RS}"
    done

    # BASH_ALIASES gives both names and values without a fork.
    for __tue_env_n in "${!BASH_ALIASES[@]}"
    do
        __tue_env_track_excluded "${__tue_env_n}" && continue
        printf 'A%s%s%s%s%s' "${__TUE_ENV_FS}" "${__tue_env_n}" "${__TUE_ENV_FS}" \
               "${BASH_ALIASES[${__tue_env_n}]}" "${__TUE_ENV_RS}"
    done

    # `complete -p` prints one registration per line; the command it applies to is the last field.
    # IFS= on the read itself keeps a registration's leading/trailing whitespace intact.
    __tue_env_names="$(complete -p 2> /dev/null)"
    while IFS= read -r __tue_env_l
    do
        [[ -z "${__tue_env_l}" ]] && continue
        __tue_env_n="${__tue_env_l##* }"
        __tue_env_track_excluded "${__tue_env_n}" && continue
        printf 'C%s%s%s%s%s' "${__TUE_ENV_FS}" "${__tue_env_n}" "${__TUE_ENV_FS}" \
               "${__tue_env_l}" "${__TUE_ENV_RS}"
    done <<< "${__tue_env_names}"

    return 0
}

function __tue_env_track_parse
{
    # $1: snapshot stream, $2: PRE or POST. Fills the __TUE_ENV_$2_* arrays, replacing their content.
    local -n __tue_env_rv="__TUE_ENV_$2_VAR"
    local -n __tue_env_rf="__TUE_ENV_$2_FUNC"
    local -n __tue_env_rx="__TUE_ENV_$2_FUNCX"
    local -n __tue_env_ra="__TUE_ENV_$2_ALIAS"
    local -n __tue_env_rc="__TUE_ENV_$2_COMPLETE"
    __tue_env_rv=()
    __tue_env_rf=()
    __tue_env_rx=()
    __tue_env_ra=()
    __tue_env_rc=()

    # One mapfile pass rather than shrinking the stream with ${x%%RS*} / ${x#*RS} per record: those
    # rescan the whole remaining string every iteration, which is quadratic in the snapshot size. On a
    # 77 KB snapshot the parse measured 1032 ms that way against roughly 34 ms here, and
    # _tue-env-track-commit parses two snapshots on every environment load.
    local -a __tue_env_recs
    mapfile -d "${__TUE_ENV_RS}" -t __tue_env_recs < <(printf '%s' "$1")

    local __tue_env_rec __tue_env_k __tue_env_n
    for __tue_env_rec in "${__tue_env_recs[@]}"
    do
        [[ -z "${__tue_env_rec}" ]] && continue
        __tue_env_k="${__tue_env_rec%%"${__TUE_ENV_FS}"*}"
        __tue_env_rec="${__tue_env_rec#*"${__TUE_ENV_FS}"}"
        __tue_env_n="${__tue_env_rec%%"${__TUE_ENV_FS}"*}"
        __tue_env_rec="${__tue_env_rec#*"${__TUE_ENV_FS}"}"
        case "${__tue_env_k}" in
            V )
                __tue_env_rv["${__tue_env_n}"]="${__tue_env_rec%$'\n'}" ;;
            F )
                __tue_env_rx["${__tue_env_n}"]="${__tue_env_rec%%"${__TUE_ENV_FS}"*}"
                __tue_env_rec="${__tue_env_rec#*"${__TUE_ENV_FS}"}"
                __tue_env_rf["${__tue_env_n}"]="${__tue_env_rec%$'\n'}" ;;
            A )
                __tue_env_ra["${__tue_env_n}"]="${__tue_env_rec}" ;;
            C )
                __tue_env_rc["${__tue_env_n}"]="${__tue_env_rec%$'\n'}" ;;
        esac
    done

    return 0
}

# ----------------------------------------------------------------------------------------------------
#                                        CLASSIFICATION
# ----------------------------------------------------------------------------------------------------

function __tue_env_track_attrs
{
    # $1: a `declare -p` line. Result in __TUE_ENV_ATTRS: the attribute letters, "" for none.
    local __tue_env_a="${1#declare }"
    __tue_env_a="${__tue_env_a%% *}"
    __tue_env_a="${__tue_env_a#-}"
    if [[ "${__tue_env_a}" == "-" ]]
    then
        __tue_env_a=""
    fi
    __TUE_ENV_ATTRS="${__tue_env_a}"
    return 0
}

function __tue_env_track_listable
{
    # $1: pre-load declare line, $2: post-load declare line, either may be empty. Returns 0 when
    # neither side is an array, so the value may be treated as a `:`-separated list. Arrays are never
    # `extended`, and __tue_env_track_value must never be evaluated on an array line: `declare -p`
    # renders one as bash array syntax, so eval would turn __TUE_ENV_VALUE into an array, and an
    # associative key can carry an assignment that bash performs in arithmetic context.
    local __tue_env_a
    __tue_env_track_attrs "$1"
    __tue_env_a="${__TUE_ENV_ATTRS}"
    __tue_env_track_attrs "$2"
    __tue_env_a+="${__TUE_ENV_ATTRS}"
    if [[ "${__tue_env_a}" == *a* ]] || [[ "${__tue_env_a}" == *A* ]]
    then
        return 1
    fi
    return 0
}

function __tue_env_track_value
{
    # $1: a `declare -p` line of a scalar. Result in __TUE_ENV_VALUE. `declare -p` output is written
    # to be re-evaluated by bash, so eval is the round trip; it is never applied to array lines.
    __TUE_ENV_VALUE=""
    if [[ "$1" != *=* ]]
    then
        return 0
    fi
    eval "__TUE_ENV_VALUE=${1#*=}"
    return 0
}

function __tue_env_track_entries
{
    # $1: pre-load value, $2: post-load value. Returns 0 when $1's `:`-separated entries are a
    # subsequence, in order, of $2's, and puts the entries of $2 that the match did not consume into
    # __TUE_ENV_ADDED as "index PS entry" pairs joined by RS. This is exactly the shape produced by
    # `export PATH=/usr/lib/ccache:${PATH}` and by /opt/ros/<distro>/setup.bash.
    __TUE_ENV_ADDED=""
    if [[ "$1" == *$'\n'* ]] || [[ "$2" == *$'\n'* ]]
    then
        return 1
    fi

    local -a __tue_env_p __tue_env_q
    IFS=':' read -r -a __tue_env_p <<< "$1"
    IFS=':' read -r -a __tue_env_q <<< "$2"

    local __tue_env_i __tue_env_j=0 __tue_env_o=""
    for (( __tue_env_i = 0; __tue_env_i < ${#__tue_env_q[@]}; __tue_env_i++ ))
    do
        if (( __tue_env_j < ${#__tue_env_p[@]} )) &&
           [[ "${__tue_env_q[__tue_env_i]}" == "${__tue_env_p[__tue_env_j]}" ]]
        then
            __tue_env_j=$(( __tue_env_j + 1 ))
        else
            __tue_env_o+="${__tue_env_i}${__TUE_ENV_PS}${__tue_env_q[__tue_env_i]}${__TUE_ENV_RS}"
        fi
    done

    if (( __tue_env_j != ${#__tue_env_p[@]} ))
    then
        return 1
    fi
    __TUE_ENV_ADDED="${__tue_env_o}"
    return 0
}

function __tue_env_track_diff_vars
{
    # Classifies every variable that differs between the PRE and POST snapshots and hands the result
    # to __tue_env_track_ledger_var.
    local __tue_env_n __tue_env_pre __tue_env_post __tue_env_kind __tue_env_add
    local __tue_env_pv __tue_env_qv
    local -A __tue_env_seen=()

    for __tue_env_n in "${!__TUE_ENV_PRE_VAR[@]}" "${!__TUE_ENV_POST_VAR[@]}"
    do
        if [[ -n "${__tue_env_seen[${__tue_env_n}]:-}" ]]
        then
            continue
        fi
        __tue_env_seen["${__tue_env_n}"]=1

        __tue_env_pre="${__TUE_ENV_PRE_VAR[${__tue_env_n}]:-}"
        __tue_env_post="${__TUE_ENV_POST_VAR[${__tue_env_n}]:-}"
        [[ "${__tue_env_pre}" == "${__tue_env_post}" ]] && continue

        # A readonly variable can be neither restored nor unset, so it is left alone entirely.
        __tue_env_track_attrs "${__tue_env_post:-${__tue_env_pre}}"
        [[ "${__TUE_ENV_ATTRS}" == *r* ]] && continue

        __tue_env_add=""
        if [[ -z "${__tue_env_pre}" ]]
        then
            __tue_env_kind="added"
            # Remember the entries as well: if a later load extends this variable, the merged entry
            # has to be able to fall back to entry-wise removal instead of unsetting it.
            if __tue_env_track_listable "" "${__tue_env_post}"
            then
                __tue_env_track_value "${__tue_env_post}"
                if __tue_env_track_entries "" "${__TUE_ENV_VALUE}"
                then
                    __tue_env_add="${__TUE_ENV_ADDED}"
                fi
            fi
        elif [[ -z "${__tue_env_post}" ]]
        then
            __tue_env_kind="removed"
        else
            __tue_env_kind="replaced"
            if __tue_env_track_listable "${__tue_env_pre}" "${__tue_env_post}"
            then
                __tue_env_track_value "${__tue_env_pre}"
                __tue_env_pv="${__TUE_ENV_VALUE}"
                __tue_env_track_value "${__tue_env_post}"
                __tue_env_qv="${__TUE_ENV_VALUE}"
                if __tue_env_track_entries "${__tue_env_pv}" "${__tue_env_qv}"
                then
                    __tue_env_kind="extended"
                    __tue_env_add="${__TUE_ENV_ADDED}"
                fi
            fi
        fi

        __tue_env_track_ledger_var "${__tue_env_n}" "${__tue_env_kind}" "${__tue_env_pre}" \
                                   "${__tue_env_post}" "${__tue_env_add}"
    done

    return 0
}

function __tue_env_track_ledger_var
{
    # $1: name, $2: kind, $3: pre-load declare line, $4: post-load declare line, $5: added entries.
    # Merging keeps the original pre-load state and takes the new post-load state. A change the user
    # made by hand between two loads is in both the new pre-load and the new post-load snapshot, so it
    # cancels out of the diff and is never attributed to the environment; that is why the ledger
    # accumulates diffs instead of re-baselining.
    local __tue_env_kind="$2" __tue_env_pre="$3" __tue_env_add="$5"

    if [[ -n "${__TUE_ENV_LEDGER_VAR[$1]:-}" ]]
    then
        local __tue_env_ok="${__TUE_ENV_LEDGER_VAR[$1]}"
        local __tue_env_oadd="${__TUE_ENV_LEDGER_VAR_ADD[$1]}"
        __tue_env_pre="${__TUE_ENV_LEDGER_VAR_PRE[$1]}"

        if [[ -z "${__tue_env_pre}" ]] && [[ -z "$4" ]]
        then
            unset "__TUE_ENV_LEDGER_VAR[$1]" "__TUE_ENV_LEDGER_VAR_PRE[$1]" \
                  "__TUE_ENV_LEDGER_VAR_POST[$1]" "__TUE_ENV_LEDGER_VAR_ADD[$1]"
            return 0
        fi

        if [[ -z "$4" ]]
        then
            __tue_env_kind="removed"
            __tue_env_add=""
        elif [[ "${__tue_env_ok}" == "extended" ]] && [[ "${__tue_env_kind}" == "extended" ]]
        then
            __tue_env_add="${__tue_env_oadd}${__tue_env_add}"
        elif [[ -z "${__tue_env_pre}" ]]
        then
            if [[ "${__tue_env_ok}" == "extended" ]] || [[ "${__tue_env_kind}" == "extended" ]]
            then
                __tue_env_kind="extended"
                __tue_env_add="${__tue_env_oadd}${__tue_env_add}"
            else
                __tue_env_kind="added"
            fi
        else
            __tue_env_kind="replaced"
            __tue_env_add=""
        fi
    fi

    __TUE_ENV_LEDGER_VAR["$1"]="${__tue_env_kind}"
    __TUE_ENV_LEDGER_VAR_PRE["$1"]="${__tue_env_pre}"
    __TUE_ENV_LEDGER_VAR_POST["$1"]="$4"
    __TUE_ENV_LEDGER_VAR_ADD["$1"]="${__tue_env_add}"
    return 0
}

function __tue_env_track_diff_funcs
{
    # Functions use the added / removed / replaced triple; the list rule does not apply to them.
    local __tue_env_n __tue_env_pre __tue_env_post __tue_env_kind __tue_env_xp __tue_env_xq
    local -A __tue_env_seen=()

    for __tue_env_n in "${!__TUE_ENV_PRE_FUNC[@]}" "${!__TUE_ENV_POST_FUNC[@]}"
    do
        if [[ -n "${__tue_env_seen[${__tue_env_n}]:-}" ]]
        then
            continue
        fi
        __tue_env_seen["${__tue_env_n}"]=1

        __tue_env_pre="${__TUE_ENV_PRE_FUNC[${__tue_env_n}]:-}"
        __tue_env_post="${__TUE_ENV_POST_FUNC[${__tue_env_n}]:-}"
        __tue_env_xp="${__TUE_ENV_PRE_FUNCX[${__tue_env_n}]:-}"
        __tue_env_xq="${__TUE_ENV_POST_FUNCX[${__tue_env_n}]:-}"
        if [[ "${__tue_env_pre}" == "${__tue_env_post}" ]] && [[ "${__tue_env_xp}" == "${__tue_env_xq}" ]]
        then
            continue
        fi

        if [[ -z "${__tue_env_pre}" ]]
        then
            __tue_env_kind="added"
        elif [[ -z "${__tue_env_post}" ]]
        then
            __tue_env_kind="removed"
        else
            __tue_env_kind="replaced"
        fi

        __tue_env_track_ledger_func "${__tue_env_n}" "${__tue_env_kind}" "${__tue_env_pre}" \
                                    "${__tue_env_post}" "${__tue_env_xp}"
    done

    return 0
}

function __tue_env_track_diff_simple
{
    # $1: ALIAS or COMPLETE. Same triple, one state string per name.
    local -n __tue_env_sp="__TUE_ENV_PRE_$1"
    local -n __tue_env_sq="__TUE_ENV_POST_$1"
    local __tue_env_n __tue_env_pre __tue_env_post __tue_env_kind
    local -A __tue_env_seen=()

    for __tue_env_n in "${!__tue_env_sp[@]}" "${!__tue_env_sq[@]}"
    do
        if [[ -n "${__tue_env_seen[${__tue_env_n}]:-}" ]]
        then
            continue
        fi
        __tue_env_seen["${__tue_env_n}"]=1

        __tue_env_pre="${__tue_env_sp[${__tue_env_n}]:-}"
        __tue_env_post="${__tue_env_sq[${__tue_env_n}]:-}"
        [[ "${__tue_env_pre}" == "${__tue_env_post}" ]] && continue

        if [[ -z "${__tue_env_pre}" ]]
        then
            __tue_env_kind="added"
        elif [[ -z "${__tue_env_post}" ]]
        then
            __tue_env_kind="removed"
        else
            __tue_env_kind="replaced"
        fi

        __tue_env_track_ledger_simple "$1" "${__tue_env_n}" "${__tue_env_kind}" "${__tue_env_pre}" \
                                     "${__tue_env_post}"
    done

    return 0
}

function __tue_env_track_ledger_func
{
    # $1: name, $2: kind, $3: pre-load body, $4: post-load body, $5: pre-load export flag.
    local __tue_env_kind="$2" __tue_env_pre="$3" __tue_env_xp="$5"

    if [[ -n "${__TUE_ENV_LEDGER_FUNC[$1]:-}" ]]
    then
        __tue_env_pre="${__TUE_ENV_LEDGER_FUNC_PRE[$1]}"
        __tue_env_xp="${__TUE_ENV_LEDGER_FUNC_XPRE[$1]}"

        if [[ -z "${__tue_env_pre}" ]] && [[ -z "$4" ]]
        then
            unset "__TUE_ENV_LEDGER_FUNC[$1]" "__TUE_ENV_LEDGER_FUNC_PRE[$1]" \
                  "__TUE_ENV_LEDGER_FUNC_POST[$1]" "__TUE_ENV_LEDGER_FUNC_XPRE[$1]"
            return 0
        fi

        if [[ -z "$4" ]]
        then
            __tue_env_kind="removed"
        elif [[ -z "${__tue_env_pre}" ]]
        then
            __tue_env_kind="added"
        else
            __tue_env_kind="replaced"
        fi
    fi

    __TUE_ENV_LEDGER_FUNC["$1"]="${__tue_env_kind}"
    __TUE_ENV_LEDGER_FUNC_PRE["$1"]="${__tue_env_pre}"
    __TUE_ENV_LEDGER_FUNC_POST["$1"]="$4"
    __TUE_ENV_LEDGER_FUNC_XPRE["$1"]="${__tue_env_xp}"
    return 0
}

function __tue_env_track_ledger_simple
{
    # $1: ALIAS or COMPLETE, $2: name, $3: kind, $4: pre-load state, $5: post-load state.
    local -n __tue_env_lk="__TUE_ENV_LEDGER_$1"
    local -n __tue_env_lp="__TUE_ENV_LEDGER_$1_PRE"
    local -n __tue_env_lq="__TUE_ENV_LEDGER_$1_POST"
    local __tue_env_kind="$3" __tue_env_pre="$4"

    if [[ -n "${__tue_env_lk[$2]:-}" ]]
    then
        __tue_env_pre="${__tue_env_lp[$2]}"

        if [[ -z "${__tue_env_pre}" ]] && [[ -z "$5" ]]
        then
            unset "__tue_env_lk[$2]" "__tue_env_lp[$2]" "__tue_env_lq[$2]"
            return 0
        fi

        if [[ -z "$5" ]]
        then
            __tue_env_kind="removed"
        elif [[ -z "${__tue_env_pre}" ]]
        then
            __tue_env_kind="added"
        else
            __tue_env_kind="replaced"
        fi
    fi

    __tue_env_lk["$2"]="${__tue_env_kind}"
    __tue_env_lp["$2"]="${__tue_env_pre}"
    __tue_env_lq["$2"]="$5"
    return 0
}

# ----------------------------------------------------------------------------------------------------
#                                       ENTRY POINTS
# ----------------------------------------------------------------------------------------------------

function __tue_env_track_empty
{
    # Returns 0 when the ledger holds nothing.
    if (( ${#__TUE_ENV_LEDGER_VAR[@]} + ${#__TUE_ENV_LEDGER_FUNC[@]} +
          ${#__TUE_ENV_LEDGER_ALIAS[@]} + ${#__TUE_ENV_LEDGER_COMPLETE[@]} == 0 ))
    then
        return 0
    fi
    return 1
}

function _tue-env-track-begin
{
    # Takes the transient pre-load snapshot, guarded by a depth counter so that a target setup script
    # that sources setup.bash recursively contributes to the outer load instead of starting its own.
    __TUE_ENV_TRACK_DEPTH=$(( __TUE_ENV_TRACK_DEPTH + 1 ))
    if (( __TUE_ENV_TRACK_DEPTH != 1 ))
    then
        return 0
    fi
    __TUE_ENV_SNAP_PRE="$(__tue_env_track_dump)"
    return 0
}

function _tue-env-track-commit
{
    # Takes the post-load snapshot, diffs it against the pre-load one and merges the diff into the
    # ledger, but only when the depth counter comes back to zero.
    if (( __TUE_ENV_TRACK_DEPTH == 0 ))
    then
        return 0
    fi
    __TUE_ENV_TRACK_DEPTH=$(( __TUE_ENV_TRACK_DEPTH - 1 ))
    if (( __TUE_ENV_TRACK_DEPTH != 0 ))
    then
        return 0
    fi

    local __tue_env_snap
    __tue_env_snap="$(__tue_env_track_dump)"
    __tue_env_track_parse "${__TUE_ENV_SNAP_PRE}" PRE
    __tue_env_track_parse "${__tue_env_snap}" POST
    __TUE_ENV_SNAP_PRE=""

    __tue_env_track_diff_vars
    __tue_env_track_diff_funcs
    __tue_env_track_diff_simple ALIAS
    __tue_env_track_diff_simple COMPLETE

    # The snapshots are transient; they exist only for the duration of a load.
    __TUE_ENV_PRE_VAR=()
    __TUE_ENV_PRE_FUNC=()
    __TUE_ENV_PRE_FUNCX=()
    __TUE_ENV_PRE_ALIAS=()
    __TUE_ENV_PRE_COMPLETE=()
    __TUE_ENV_POST_VAR=()
    __TUE_ENV_POST_FUNC=()
    __TUE_ENV_POST_FUNCX=()
    __TUE_ENV_POST_ALIAS=()
    __TUE_ENV_POST_COMPLETE=()
    return 0
}

# ----------------------------------------------------------------------------------------------------
#                                          REVERTING
# ----------------------------------------------------------------------------------------------------

function __tue_env_track_kept
{
    # $1: what was kept, e.g. "value for PATH" or "version of function tue-make".
    echo "[tue-env](deactivate) kept your $1"
    return 0
}

function __tue_env_track_restore_line
{
    # $1: a captured `declare -p` line. Evaluating it as it stands from inside a function would create
    # a function-local variable and silently do nothing, so the attributes are rewritten to carry -g.
    local __tue_env_rest="${1#declare }"
    local __tue_env_attrs="${__tue_env_rest%% *}"
    __tue_env_rest="${__tue_env_rest#* }"
    local __tue_env_flags="${__tue_env_attrs#-}"
    if [[ "${__tue_env_flags}" == "-" ]]
    then
        __tue_env_flags=""
    fi
    # Unset first: re-declaring over a variable whose current type differs keeps the old data (a
    # scalar restored over an array leaves the array's other elements in place) and can leave a stale
    # attribute, or fail outright between indexed and associative.
    local __tue_env_name="${__tue_env_rest%%=*}"
    unset -v "${__tue_env_name}"
    eval "declare -g${__tue_env_flags} ${__tue_env_rest}"
    return 0
}

function __tue_env_track_split
{
    # $1: a `:`-separated value. Result in __TUE_ENV_SPLIT, empty fields preserved. `IFS=':' read -a`
    # is unusable here twice over: it silently drops a trailing empty field, and it stops at the first
    # newline, which would make the `:.` sentinel below invisible and delete a real field instead. An
    # empty field in PATH means the current directory, so either mistake changes what the shell runs.
    # A wholly empty value yields no entries, which is what __tue_env_track_entries already assumes.
    __TUE_ENV_SPLIT=()
    if [[ -z "$1" ]]
    then
        return 0
    fi
    local -a __tue_env_s=()
    mapfile -d ':' -t __tue_env_s < <(printf '%s:.' "$1")
    unset "__tue_env_s[$(( ${#__tue_env_s[@]} - 1 ))]"
    __TUE_ENV_SPLIT=("${__tue_env_s[@]}")
    return 0
}

function __tue_env_track_strip
{
    # $1: current value, $2: recorded added entries, $3: the pre-load value ("" when the variable did
    # not exist before the load). Result in __TUE_ENV_VALUE.
    #
    # Each recorded entry removes one occurrence: the one closest to the recorded index, ties going to
    # the HIGHER position, because a user prepending to the value shifts the environment's entries
    # rightwards. An occurrence is never removed if that would leave fewer copies of the entry than
    # the value held before the load — the copies the user already had must survive, and when the
    # environment duplicated one of them the two cannot be told apart by value alone. An entry that is
    # no longer present is skipped.
    local -a __tue_env_c __tue_env_p
    __tue_env_track_split "$1"
    __tue_env_c=("${__TUE_ENV_SPLIT[@]}")
    __tue_env_track_split "$3"
    __tue_env_p=("${__TUE_ENV_SPLIT[@]}")

    # An empty entry (an empty PATH field) cannot be used as an associative-array subscript on its
    # own: `a[""]=1` is a bash syntax error ("bad array subscript"), even though the empty string is
    # a perfectly valid array VALUE. Every key below is therefore prefixed with a fixed, non-empty
    # character; concatenating a constant prefix can never make two different entries collide, so the
    # mapping stays one-to-one.
    local -A __tue_env_keep=() __tue_env_live=()
    local __tue_env_x
    for __tue_env_x in ${__tue_env_p[@]+"${__tue_env_p[@]}"}
    do
        __tue_env_keep["k${__tue_env_x}"]=$(( ${__tue_env_keep["k${__tue_env_x}"]:-0} + 1 ))
    done
    for __tue_env_x in ${__tue_env_c[@]+"${__tue_env_c[@]}"}
    do
        __tue_env_live["k${__tue_env_x}"]=$(( ${__tue_env_live["k${__tue_env_x}"]:-0} + 1 ))
    done

    local -A __tue_env_dead=()
    local __tue_env_rest="$2" __tue_env_pair __tue_env_idx __tue_env_e
    local __tue_env_best __tue_env_bestd __tue_env_i __tue_env_d
    while [[ -n "${__tue_env_rest}" ]]
    do
        # Same trap the test helper guards: without this an unterminated tail spins forever.
        [[ "${__tue_env_rest}" == *"${__TUE_ENV_RS}"* ]] || break
        __tue_env_pair="${__tue_env_rest%%"${__TUE_ENV_RS}"*}"
        __tue_env_rest="${__tue_env_rest#*"${__TUE_ENV_RS}"}"
        [[ -z "${__tue_env_pair}" ]] && continue
        __tue_env_idx="${__tue_env_pair%%"${__TUE_ENV_PS}"*}"
        __tue_env_e="${__tue_env_pair#*"${__TUE_ENV_PS}"}"

        if (( ${__tue_env_live["k${__tue_env_e}"]:-0} <= ${__tue_env_keep["k${__tue_env_e}"]:-0} ))
        then
            continue
        fi

        __tue_env_best=""
        __tue_env_bestd=-1
        for (( __tue_env_i = 0; __tue_env_i < ${#__tue_env_c[@]}; __tue_env_i++ ))
        do
            [[ -n "${__tue_env_dead[${__tue_env_i}]:-}" ]] && continue
            [[ "${__tue_env_c[__tue_env_i]}" != "${__tue_env_e}" ]] && continue
            __tue_env_d=$(( __tue_env_i - __tue_env_idx ))
            if (( __tue_env_d < 0 ))
            then
                __tue_env_d=$(( 0 - __tue_env_d ))
            fi
            # `<=`, not `<`: on a tie the higher position wins.
            if (( __tue_env_bestd < 0 )) || (( __tue_env_d <= __tue_env_bestd ))
            then
                __tue_env_bestd="${__tue_env_d}"
                __tue_env_best="${__tue_env_i}"
            fi
        done
        if [[ -n "${__tue_env_best}" ]]
        then
            __tue_env_dead["${__tue_env_best}"]=1
            __tue_env_live["k${__tue_env_e}"]=$(( ${__tue_env_live["k${__tue_env_e}"]} - 1 ))
        fi
    done

    local __tue_env_o="" __tue_env_first="true"
    for (( __tue_env_i = 0; __tue_env_i < ${#__tue_env_c[@]}; __tue_env_i++ ))
    do
        [[ -n "${__tue_env_dead[${__tue_env_i}]:-}" ]] && continue
        if [[ "${__tue_env_first}" == "true" ]]
        then
            __tue_env_o="${__tue_env_c[__tue_env_i]}"
            __tue_env_first="false"
        else
            __tue_env_o+=":${__tue_env_c[__tue_env_i]}"
        fi
    done
    __TUE_ENV_VALUE="${__tue_env_o}"
    return 0
}

function __tue_env_track_revert_vars
{
    # Applies every variable entry in the ledger. Names are sorted so that the notes printed for kept
    # user changes come out in a stable order.
    local __tue_env_n __tue_env_kind __tue_env_pre __tue_env_post __tue_env_cur __tue_env_pv
    local -a __tue_env_names
    mapfile -t __tue_env_names < <(printf '%s\n' "${!__TUE_ENV_LEDGER_VAR[@]}" | LC_ALL=C sort)

    for __tue_env_n in "${__tue_env_names[@]}"
    do
        [[ -z "${__tue_env_n}" ]] && continue
        __tue_env_kind="${__TUE_ENV_LEDGER_VAR[${__tue_env_n}]}"
        __tue_env_pre="${__TUE_ENV_LEDGER_VAR_PRE[${__tue_env_n}]}"
        __tue_env_post="${__TUE_ENV_LEDGER_VAR_POST[${__tue_env_n}]}"
        __tue_env_cur="$(declare -p "${__tue_env_n}" 2> /dev/null)" || __tue_env_cur=""

        if [[ "${__tue_env_kind}" == "extended" ]]
        then
            # Entry-wise removal needs no conflict check: whatever the user added stays by
            # construction.
            [[ -z "${__tue_env_cur}" ]] && continue

            # The current state must still be listable, exactly like __tue_env_track_diff_vars
            # requires before treating a value as `:`-separated: evaluating an associative array's
            # `declare -p` line can execute a command substitution smuggled into a key.
            if ! __tue_env_track_listable "" "${__tue_env_cur}"
            then
                __tue_env_track_kept "value for ${__tue_env_n}"
                continue
            fi

            __tue_env_pv=""
            if [[ -n "${__tue_env_pre}" ]]
            then
                __tue_env_track_value "${__tue_env_pre}"
                __tue_env_pv="${__TUE_ENV_VALUE}"
            fi
            __tue_env_track_value "${__tue_env_cur}"
            __tue_env_track_strip "${__TUE_ENV_VALUE}" \
                                  "${__TUE_ENV_LEDGER_VAR_ADD[${__tue_env_n}]}" "${__tue_env_pv}"
            if [[ -z "${__TUE_ENV_VALUE}" ]] && [[ -z "${__tue_env_pre}" ]]
            then
                unset "${__tue_env_n}"
            else
                printf -v "${__tue_env_n}" '%s' "${__TUE_ENV_VALUE}"
            fi
            continue
        fi

        if [[ "${__tue_env_cur}" != "${__tue_env_post}" ]]
        then
            __tue_env_track_kept "value for ${__tue_env_n}"
            continue
        fi

        if [[ -z "${__tue_env_pre}" ]]
        then
            unset "${__tue_env_n}"
        else
            __tue_env_track_restore_line "${__tue_env_pre}"
        fi
    done

    return 0
}

function __tue_env_track_current
{
    # $1: FUNC, ALIAS or COMPLETE, $2: name. Result in __TUE_ENV_CURRENT, empty when absent.
    # `declare -f` and `complete -p` both return 1 for a name that does not exist, and a bare
    # command-substitution assignment propagates that, so without the `||` a caller running under
    # `set -e` would abort here instead of treating the object as absent.
    case "$1" in
        FUNC )
            __TUE_ENV_CURRENT="$(declare -f "$2" 2> /dev/null)" || __TUE_ENV_CURRENT="" ;;
        ALIAS )
            __TUE_ENV_CURRENT="${BASH_ALIASES[$2]:-}" ;;
        COMPLETE )
            __TUE_ENV_CURRENT="$(complete -p "$2" 2> /dev/null)" || __TUE_ENV_CURRENT="" ;;
    esac
    return 0
}

function __tue_env_track_revert_funcs
{
    local __tue_env_n __tue_env_pre
    local -a __tue_env_names
    mapfile -t __tue_env_names < <(printf '%s\n' "${!__TUE_ENV_LEDGER_FUNC[@]}" | LC_ALL=C sort)

    for __tue_env_n in "${__tue_env_names[@]}"
    do
        [[ -z "${__tue_env_n}" ]] && continue
        __tue_env_track_current FUNC "${__tue_env_n}"
        if [[ "${__TUE_ENV_CURRENT}" != "${__TUE_ENV_LEDGER_FUNC_POST[${__tue_env_n}]}" ]]
        then
            __tue_env_track_kept "version of function ${__tue_env_n}"
            continue
        fi

        unset -f "${__tue_env_n}"
        __tue_env_pre="${__TUE_ENV_LEDGER_FUNC_PRE[${__tue_env_n}]}"
        if [[ -n "${__tue_env_pre}" ]]
        then
            eval "${__tue_env_pre}"
            if [[ "${__TUE_ENV_LEDGER_FUNC_XPRE[${__tue_env_n}]}" == "x" ]]
            then
                # `declare -f` output does not encode `export -f`, so it has to be re-applied. The name
                # to export is held in a variable, not literal, which is exactly what SC2163 flags.
                # shellcheck disable=SC2163
                export -f "${__tue_env_n}"
            fi
        fi
    done

    return 0
}

function __tue_env_track_revert_simple
{
    # $1: ALIAS or COMPLETE. The nameref locals are named distinctly from __tue_env_track_ledger_simple's
    # (which also namerefs the same three globals): shellcheck's SC2178 does not scope nameref type
    # inference per function, so reusing those names here reads, to it, as the same variable switching
    # from array to scalar and it warns; giving these their own names side-steps that false positive.
    local -n __tue_env_rlk="__TUE_ENV_LEDGER_$1"
    local -n __tue_env_rlp="__TUE_ENV_LEDGER_$1_PRE"
    local -n __tue_env_rlq="__TUE_ENV_LEDGER_$1_POST"
    local __tue_env_n __tue_env_pre __tue_env_label
    local -a __tue_env_names
    mapfile -t __tue_env_names < <(printf '%s\n' "${!__tue_env_rlk[@]}" | LC_ALL=C sort)

    if [[ "$1" == "ALIAS" ]]
    then
        __tue_env_label="alias"
    else
        __tue_env_label="completion for"
    fi

    for __tue_env_n in "${__tue_env_names[@]}"
    do
        [[ -z "${__tue_env_n}" ]] && continue
        __tue_env_track_current "$1" "${__tue_env_n}"
        if [[ "${__TUE_ENV_CURRENT}" != "${__tue_env_rlq[${__tue_env_n}]}" ]]
        then
            __tue_env_track_kept "version of ${__tue_env_label} ${__tue_env_n}"
            continue
        fi

        __tue_env_pre="${__tue_env_rlp[${__tue_env_n}]}"
        if [[ "$1" == "ALIAS" ]]
        then
            unalias "${__tue_env_n}" 2> /dev/null || true
            if [[ -n "${__tue_env_pre}" ]]
            then
                # The pre-load alias text is meant to expand right now, into the literal text that
                # `alias` re-registers; it is not a deferred expression, which is what SC2139 flags.
                # shellcheck disable=SC2139
                alias "${__tue_env_n}=${__tue_env_pre}"
            fi
        else
            complete -r "${__tue_env_n}" 2> /dev/null || true
            if [[ -n "${__tue_env_pre}" ]]
            then
                eval "${__tue_env_pre}"
            fi
        fi
    done

    return 0
}

function __tue_env_track_clear
{
    __TUE_ENV_LEDGER_VAR=()
    __TUE_ENV_LEDGER_VAR_PRE=()
    __TUE_ENV_LEDGER_VAR_POST=()
    __TUE_ENV_LEDGER_VAR_ADD=()
    __TUE_ENV_LEDGER_FUNC=()
    __TUE_ENV_LEDGER_FUNC_PRE=()
    __TUE_ENV_LEDGER_FUNC_POST=()
    __TUE_ENV_LEDGER_FUNC_XPRE=()
    __TUE_ENV_LEDGER_ALIAS=()
    __TUE_ENV_LEDGER_ALIAS_PRE=()
    __TUE_ENV_LEDGER_ALIAS_POST=()
    __TUE_ENV_LEDGER_COMPLETE=()
    __TUE_ENV_LEDGER_COMPLETE_PRE=()
    __TUE_ENV_LEDGER_COMPLETE_POST=()
    return 0
}

function _tue-env-track-revert
{
    # Applies the ledger to this shell and clears it. Returns 1 without touching anything when the
    # ledger is empty, which is the signal for the caller to fall back to the old heuristic.
    if __tue_env_track_empty
    then
        return 1
    fi

    __tue_env_track_revert_vars
    __tue_env_track_revert_funcs
    __tue_env_track_revert_simple ALIAS
    __tue_env_track_revert_simple COMPLETE
    __tue_env_track_clear

    # The one non-variable thing the virtual environment's `deactivate` does; the ledger has already
    # taken care of everything else that `deactivate` would have restored, including unsetting the
    # `deactivate` function itself.
    hash -r
    return 0
}

# ----------------------------------------------------------------------------------------------------
#                                          REPORTING
# ----------------------------------------------------------------------------------------------------

function __tue_env_track_display
{
    # $1: a `declare -p` line. Result in __TUE_ENV_VALUE, ready to print. Array values are not
    # rendered: their `declare -p` body is bash source, not something a user wants to read.
    __tue_env_track_attrs "$1"
    if [[ "${__TUE_ENV_ATTRS}" == *a* ]] || [[ "${__TUE_ENV_ATTRS}" == *A* ]]
    then
        __TUE_ENV_VALUE="(array)"
        return 0
    fi
    __tue_env_track_value "$1"
    return 0
}

function __tue_env_track_entry_list
{
    # $1: recorded added entries. Result in __TUE_ENV_LIST: "entry, entry".
    local __tue_env_rest="$1" __tue_env_pair __tue_env_o=""
    while [[ -n "${__tue_env_rest}" ]]
    do
        # Same trap the sibling loops guard against (__tue_env_track_strip, tue_track_added):
        # without this an unterminated tail spins forever.
        [[ "${__tue_env_rest}" == *"${__TUE_ENV_RS}"* ]] || break
        __tue_env_pair="${__tue_env_rest%%"${__TUE_ENV_RS}"*}"
        __tue_env_rest="${__tue_env_rest#*"${__TUE_ENV_RS}"}"
        [[ -z "${__tue_env_pair}" ]] && continue
        __tue_env_o+="${__tue_env_o:+, }${__tue_env_pair#*"${__TUE_ENV_PS}"}"
    done
    __TUE_ENV_LIST="${__tue_env_o}"
    return 0
}

function __tue_env_track_removed
{
    # $1: a `:`-separated value, $2: the same value after __tue_env_track_strip. Result in
    # __TUE_ENV_LIST: the entries of $1 that do not survive in $2, "entry, entry", in the order
    # they appear in $1. $2 is always exactly $1 with some positions dropped, so a left-to-right
    # greedy alignment always finds a valid pairing; which of several equal-valued entries it
    # credits as dropped does not matter here, only the combined text of the dropped ones does.
    local -a __tue_env_c __tue_env_r
    __tue_env_track_split "$1"
    __tue_env_c=("${__TUE_ENV_SPLIT[@]}")
    __tue_env_track_split "$2"
    __tue_env_r=("${__TUE_ENV_SPLIT[@]}")

    local __tue_env_ci=0 __tue_env_ri=0 __tue_env_o=""
    while (( __tue_env_ci < ${#__tue_env_c[@]} ))
    do
        if (( __tue_env_ri < ${#__tue_env_r[@]} )) &&
           [[ "${__tue_env_c[__tue_env_ci]}" == "${__tue_env_r[__tue_env_ri]}" ]]
        then
            __tue_env_ri=$(( __tue_env_ri + 1 ))
        else
            __tue_env_o+="${__tue_env_o:+, }${__tue_env_c[__tue_env_ci]}"
        fi
        __tue_env_ci=$(( __tue_env_ci + 1 ))
    done
    __TUE_ENV_LIST="${__tue_env_o}"
    return 0
}

function __tue_env_track_report_vars
{
    # $1: changes or revert.
    local __tue_env_n __tue_env_k __tue_env_pre __tue_env_post __tue_env_cur __tue_env_pv
    local __tue_env_cv
    local -a __tue_env_names
    mapfile -t __tue_env_names < <(printf '%s\n' "${!__TUE_ENV_LEDGER_VAR[@]}" | LC_ALL=C sort)

    for __tue_env_n in "${__tue_env_names[@]}"
    do
        [[ -z "${__tue_env_n}" ]] && continue
        __tue_env_k="${__TUE_ENV_LEDGER_VAR[${__tue_env_n}]}"
        __tue_env_pre="${__TUE_ENV_LEDGER_VAR_PRE[${__tue_env_n}]}"
        __tue_env_post="${__TUE_ENV_LEDGER_VAR_POST[${__tue_env_n}]}"
        __tue_env_cur="$(declare -p "${__tue_env_n}" 2> /dev/null)" || __tue_env_cur=""

        if [[ "${__tue_env_k}" == "extended" ]]
        then
            if [[ "$1" == "changes" ]]
            then
                __tue_env_track_entry_list "${__TUE_ENV_LEDGER_VAR_ADD[${__tue_env_n}]}"
                echo "added   ${__tue_env_n} entries: ${__TUE_ENV_LIST}"
                continue
            fi

            # Mirror __tue_env_track_revert_vars's extended branch (764-778): a variable the
            # user has since unset gets nothing done to it, and one that is no longer listable
            # is kept whole rather than guessed at.
            if [[ -z "${__tue_env_cur}" ]]
            then
                continue
            fi
            if ! __tue_env_track_listable "" "${__tue_env_cur}"
            then
                __tue_env_track_display "${__tue_env_cur}"
                echo "would keep   ${__tue_env_n}=${__TUE_ENV_VALUE} (changed since load)"
                continue
            fi

            # Ask the real stripper what it would leave behind, then read off which of the
            # current entries would not survive: this is the exact computation
            # __tue_env_track_revert_vars performs before assigning, so the report cannot
            # invent its own answer and drift from what a revert actually does.
            __tue_env_pv=""
            if [[ -n "${__tue_env_pre}" ]]
            then
                __tue_env_track_value "${__tue_env_pre}"
                __tue_env_pv="${__TUE_ENV_VALUE}"
            fi
            __tue_env_track_value "${__tue_env_cur}"
            __tue_env_cv="${__TUE_ENV_VALUE}"
            __tue_env_track_strip "${__tue_env_cv}" "${__TUE_ENV_LEDGER_VAR_ADD[${__tue_env_n}]}" \
                                  "${__tue_env_pv}"
            __tue_env_track_removed "${__tue_env_cv}" "${__TUE_ENV_VALUE}"
            if [[ -n "${__TUE_ENV_LIST}" ]]
            then
                echo "would remove ${__tue_env_n} entries: ${__TUE_ENV_LIST}"
            fi
            continue
        fi

        if [[ "$1" == "revert" ]] && [[ "${__tue_env_cur}" != "${__tue_env_post}" ]]
        then
            __tue_env_track_display "${__tue_env_cur}"
            echo "would keep   ${__tue_env_n}=${__TUE_ENV_VALUE} (changed since load)"
            continue
        fi

        case "${__tue_env_k}" in
            added )
                __tue_env_track_display "${__tue_env_post}"
                if [[ "$1" == "changes" ]]
                then
                    echo "added   ${__tue_env_n}=${__TUE_ENV_VALUE}"
                else
                    echo "would unset  ${__tue_env_n}"
                fi ;;
            removed | replaced )
                __tue_env_track_display "${__tue_env_pre}"
                if [[ "$1" != "changes" ]]
                then
                    echo "would restore ${__tue_env_n} to '${__TUE_ENV_VALUE}'"
                elif [[ "${__tue_env_k}" == "removed" ]]
                then
                    echo "removed ${__tue_env_n} (was '${__TUE_ENV_VALUE}')"
                else
                    echo "changed ${__tue_env_n} (was '${__TUE_ENV_VALUE}')"
                fi ;;
        esac
    done

    return 0
}

function __tue_env_track_report_objects
{
    # $1: changes or revert, $2: FUNC, ALIAS or COMPLETE, $3: label to print before each name.
    # Distinct nameref names from __tue_env_track_ledger_simple's: reusing them makes shellcheck
    # carry an SC2178 across function scopes and fail the file.
    local -n __tue_env_pk="__TUE_ENV_LEDGER_$2"
    local -n __tue_env_pq="__TUE_ENV_LEDGER_$2_POST"
    # __tue_env_pkeep, not __tue_env_keep: __tue_env_track_strip already uses that name for an
    # associative array, and shellcheck carries the type across function scopes (SC2178).
    local __tue_env_n __tue_env_add="" __tue_env_gone="" __tue_env_chg="" __tue_env_pkeep=""
    local -a __tue_env_names
    mapfile -t __tue_env_names < <(printf '%s\n' "${!__tue_env_pk[@]}" | LC_ALL=C sort)

    for __tue_env_n in "${__tue_env_names[@]}"
    do
        [[ -z "${__tue_env_n}" ]] && continue
        __tue_env_track_current "$2" "${__tue_env_n}"
        if [[ "$1" == "revert" ]] && [[ "${__TUE_ENV_CURRENT}" != "${__tue_env_pq[${__tue_env_n}]}" ]]
        then
            __tue_env_pkeep+="${__tue_env_pkeep:+, }$3 ${__tue_env_n}"
            continue
        fi
        case "${__tue_env_pk[${__tue_env_n}]}" in
            added )
                __tue_env_add+="${__tue_env_add:+, }$3 ${__tue_env_n}" ;;
            removed )
                __tue_env_gone+="${__tue_env_gone:+, }$3 ${__tue_env_n}" ;;
            replaced )
                __tue_env_chg+="${__tue_env_chg:+, }$3 ${__tue_env_n}" ;;
        esac
    done

    if [[ "$1" == "changes" ]]
    then
        [[ -n "${__tue_env_add}" ]] && echo "added   ${__tue_env_add}"
        [[ -n "${__tue_env_chg}" ]] && echo "changed ${__tue_env_chg}"
        [[ -n "${__tue_env_gone}" ]] && echo "removed ${__tue_env_gone}"
    else
        [[ -n "${__tue_env_add}" ]] && echo "would unset  ${__tue_env_add}"
        [[ -n "${__tue_env_chg}" ]] && echo "would restore ${__tue_env_chg}"
        [[ -n "${__tue_env_gone}" ]] && echo "would restore ${__tue_env_gone}"
        [[ -n "${__tue_env_pkeep}" ]] && echo "would keep   ${__tue_env_pkeep} (changed since load)"
    fi

    return 0
}

function _tue-env-track-report
{
    # $1: changes or revert. Renders the ledger, or the revert it would perform, without mutating
    # anything. Returns 2 when $1 is neither, 1 when the ledger is empty.
    case "${1-}" in
        changes | revert ) ;;
        * )
            echo "_tue-env-track-report: mode must be 'changes' or 'revert'" >&2
            return 2 ;;
    esac

    if __tue_env_track_empty
    then
        return 1
    fi

    __tue_env_track_report_vars "$1"
    __tue_env_track_report_objects "$1" ALIAS "alias"
    __tue_env_track_report_objects "$1" FUNC "function"
    __tue_env_track_report_objects "$1" COMPLETE "completion for"
    return 0
}
