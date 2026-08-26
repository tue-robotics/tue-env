load helpers/track

setup() {
    tue_track_setup
}

@test "capture: a scalar round-trips with its export attribute" {
    export TUE_TEST_SCALAR="hello world"
    tue_track_snapshot PRE
    [[ "${__TUE_ENV_PRE_VAR[TUE_TEST_SCALAR]}" == 'declare -x TUE_TEST_SCALAR="hello world"' ]]
}

@test "capture: an array round-trips" {
    TUE_TEST_ARR=(x "y z")
    tue_track_snapshot PRE
    [[ "${__TUE_ENV_PRE_VAR[TUE_TEST_ARR]}" == "$(declare -p TUE_TEST_ARR)" ]]
}

@test "capture: newlines, colons, spaces and single quotes survive" {
    TUE_TEST_NASTY=$'a\nb:c d'\''e'
    tue_track_snapshot PRE
    [[ "${__TUE_ENV_PRE_VAR[TUE_TEST_NASTY]}" == "$(declare -p TUE_TEST_NASTY)" ]]
}

@test "capture: volatile shell state is excluded" {
    tue_track_snapshot PRE
    [[ -z "${__TUE_ENV_PRE_VAR[RANDOM]:-}" ]]
    [[ -z "${__TUE_ENV_PRE_VAR[PWD]:-}" ]]
    [[ -z "${__TUE_ENV_PRE_VAR[SECONDS]:-}" ]]
    [[ -z "${__TUE_ENV_PRE_VAR[BASH_SOURCE]:-}" ]]
    [[ -z "${__TUE_ENV_PRE_VAR[__TUE_ENV_TRACK_DEPTH]:-}" ]]
}

@test "capture: function bodies and the export attribute" {
    tue_test_exported() {
        echo "multi
line"
    }
    export -f tue_test_exported
    tue_test_plain() {
        echo plain
    }
    tue_track_snapshot PRE
    [[ "${__TUE_ENV_PRE_FUNC[tue_test_exported]}" == "$(declare -f tue_test_exported)" ]]
    [[ "${__TUE_ENV_PRE_FUNCX[tue_test_exported]}" == "x" ]]
    [[ "${__TUE_ENV_PRE_FUNCX[tue_test_plain]:-}" == "" ]]
}

@test "capture: aliases, including odd names" {
    alias tue_test_alias='echo hi'
    alias ..='cd ..'
    tue_track_snapshot PRE
    [[ "${__TUE_ENV_PRE_ALIAS[tue_test_alias]}" == "echo hi" ]]
    [[ "${__TUE_ENV_PRE_ALIAS[..]}" == "cd .." ]]
}

@test "capture: completion registrations" {
    tue_test_complete() {
        COMPREPLY=()
    }
    complete -o nospace -F tue_test_complete tue-test-cmd
    tue_track_snapshot PRE
    [[ "${__TUE_ENV_PRE_COMPLETE[tue-test-cmd]}" == "complete -o nospace -F tue_test_complete tue-test-cmd" ]]
}

@test "capture: two snapshots of an unchanged shell are byte-identical" {
    local __tue_env_a __tue_env_b
    __tue_env_a="$(__tue_env_track_dump)"
    __tue_env_b="$(__tue_env_track_dump)"
    [[ "${__tue_env_a}" == "${__tue_env_b}" ]]
}

@test "capture: the number of forks a load costs does not grow with the number of names" {
    # The spec used to promise "two forks per environment load", which has not been true since the
    # dump grew its nested substitutions - roughly ten is what it costs now. The number was never the
    # point; what the design has to hold is that it is a CONSTANT, so that a load stays cheap on a
    # shell with thousands of names in it. Establish that by measurement, then require the spec to
    # state that property rather than a count that has already gone stale once.
    local __tue_env_small __tue_env_big
    tue_track_min_forks 20
    __tue_env_small="${TUE_TRACK_FORKS}"
    tue_track_min_forks 500
    __tue_env_big="${TUE_TRACK_FORKS}"

    # 1440 more names in the snapshot. A fork per name would put the second figure at least that much
    # higher; a constant cost puts it level with the first, and the margin absorbs whatever PIDs the
    # rest of the machine took while the samples ran.
    [[ "${__tue_env_small}" -ge 0 ]]
    [[ "${__tue_env_big}" -ge 0 ]]
    [[ "${__tue_env_big}" -le $(( __tue_env_small + 100 )) ]]

    local __tue_env_spec
    __tue_env_spec="${TUE_TRACK_REPO_ROOT}/docs/superpowers/specs/2026-08-21-env-change-tracking-design.md"
    grep -q 'constant number of forks, never one per' "${__tue_env_spec}"
    [[ -z "$(grep 'two forks per environment load' "${__tue_env_spec}")" ]]
}

@test "capture: an extra exclude pattern hides a name" {
    __TUE_ENV_TRACK_EXTRA_EXCLUDE+=('TUE_TEST_HIDDEN')
    export TUE_TEST_HIDDEN=1
    tue_track_snapshot PRE
    [[ -z "${__TUE_ENV_PRE_VAR[TUE_TEST_HIDDEN]:-}" ]]
}

@test "capture: parsing a second snapshot replaces the first one's content" {
    export TUE_TEST_TRANSIENT=1
    tue_track_snapshot PRE
    [[ -n "${__TUE_ENV_PRE_VAR[TUE_TEST_TRANSIENT]:-}" ]]
    unset TUE_TEST_TRANSIENT
    tue_track_snapshot PRE
    [[ -z "${__TUE_ENV_PRE_VAR[TUE_TEST_TRANSIENT]:-}" ]]
}
