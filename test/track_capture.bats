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

@test "capture: an extra exclude pattern hides a name" {
    __TUE_ENV_TRACK_EXTRA_EXCLUDE+=('TUE_TEST_HIDDEN')
    export TUE_TEST_HIDDEN=1
    tue_track_snapshot PRE
    [[ -z "${__TUE_ENV_PRE_VAR[TUE_TEST_HIDDEN]:-}" ]]
}
