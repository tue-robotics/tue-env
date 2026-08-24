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
    __TUE_ENV_LEDGER_VAR["$1"]="$2"
    __TUE_ENV_LEDGER_VAR_PRE["$1"]="$3"
    __TUE_ENV_LEDGER_VAR_POST["$1"]="$4"
    __TUE_ENV_LEDGER_VAR_ADD["$1"]="$5"
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
    __TUE_ENV_LEDGER_FUNC["$1"]="$2"
    __TUE_ENV_LEDGER_FUNC_PRE["$1"]="$3"
    __TUE_ENV_LEDGER_FUNC_POST["$1"]="$4"
    __TUE_ENV_LEDGER_FUNC_XPRE["$1"]="$5"
    return 0
}

function __tue_env_track_ledger_simple
{
    # $1: ALIAS or COMPLETE, $2: name, $3: kind, $4: pre-load state, $5: post-load state.
    local -n __tue_env_lk="__TUE_ENV_LEDGER_$1"
    local -n __tue_env_lp="__TUE_ENV_LEDGER_$1_PRE"
    local -n __tue_env_lq="__TUE_ENV_LEDGER_$1_POST"
    __tue_env_lk["$2"]="$3"
    __tue_env_lp["$2"]="$4"
    __tue_env_lq["$2"]="$5"
    return 0
}
