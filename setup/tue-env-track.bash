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
        BASH* | COMP_* | DIRSTACK | EPOCH* | FUNCNAME | GROUPS | HISTCMD | LINENO | OLDPWD | \
        PIPESTATUS | PWD | RANDOM | SECONDS | SHLVL | SRANDOM | _ | __TUE_ENV_* | __tue_env_* )
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
    local __tue_env_n __tue_env_names __tue_env_l __tue_env_d1 __tue_env_d2
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

    __tue_env_names="$(compgen -v)"
    for __tue_env_n in ${__tue_env_names}
    do
        __tue_env_track_excluded "${__tue_env_n}" && continue
        printf 'V%s%s%s' "${__TUE_ENV_FS}" "${__tue_env_n}" "${__TUE_ENV_FS}"
        declare -p "${__tue_env_n}" 2> /dev/null
        printf '%s' "${__TUE_ENV_RS}"
    done

    __tue_env_names="$(compgen -A function)"
    for __tue_env_n in ${__tue_env_names}
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
    __tue_env_names="$(complete -p 2> /dev/null)"
    while read -r __tue_env_l
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

    local __tue_env_rest="$1" __tue_env_rec __tue_env_k __tue_env_n
    while [[ -n "${__tue_env_rest}" ]]
    do
        __tue_env_rec="${__tue_env_rest%%"${__TUE_ENV_RS}"*}"
        __tue_env_rest="${__tue_env_rest#*"${__TUE_ENV_RS}"}"
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
