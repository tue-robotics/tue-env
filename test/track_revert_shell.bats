load helpers/track

setup() {
    tue_track_setup
}

@test "revert: an added function is removed" {
    _tue-env-track-begin
    tue_test_fn() {
        echo new
    }
    _tue-env-track-commit
    _tue-env-track-revert
    [[ -z "$(declare -F tue_test_fn)" ]]
}

@test "revert: a replaced function is restored" {
    tue_test_fn() {
        echo original
    }
    _tue-env-track-begin
    tue_test_fn() {
        echo replaced
    }
    _tue-env-track-commit
    _tue-env-track-revert
    [[ "$(tue_test_fn)" == "original" ]]
}

@test "revert: export -f is re-applied to a restored function" {
    tue_test_fn() {
        echo original
    }
    export -f tue_test_fn
    _tue-env-track-begin
    unset -f tue_test_fn
    tue_test_fn() {
        echo replaced
    }
    _tue-env-track-commit
    _tue-env-track-revert
    [[ "$(tue_test_fn)" == "original" ]]
    declare -Fx | grep -q ' tue_test_fn$'
}

@test "revert: a function the environment removed is restored" {
    tue_test_fn() {
        echo original
    }
    _tue-env-track-begin
    unset -f tue_test_fn
    _tue-env-track-commit
    [[ "${__TUE_ENV_LEDGER_FUNC[tue_test_fn]}" == "removed" ]]
    _tue-env-track-revert
    [[ "$(tue_test_fn)" == "original" ]]
}

@test "revert: a function the user redefined after the load is kept" {
    _tue-env-track-begin
    tue_test_fn() {
        echo fromenv
    }
    _tue-env-track-commit
    tue_test_fn() {
        echo mine
    }
    run _tue-env-track-revert
    [[ "${output}" == *"kept your version of function tue_test_fn"* ]]
}

@test "revert: a function the user redefined after the load is left in place" {
    _tue-env-track-begin
    tue_test_fn() {
        echo fromenv
    }
    _tue-env-track-commit
    tue_test_fn() {
        echo mine
    }
    _tue-env-track-revert
    [[ "$(tue_test_fn)" == "mine" ]]
}

@test "revert: an added alias is unset and a replaced one restored" {
    alias tue_test_keep='echo original'
    _tue-env-track-begin
    alias tue_test_new='echo new'
    alias tue_test_keep='echo fromenv'
    _tue-env-track-commit
    _tue-env-track-revert
    [[ -z "${BASH_ALIASES[tue_test_new]:-}" ]]
    [[ "${BASH_ALIASES[tue_test_keep]}" == "echo original" ]]
}

@test "revert: an added completion is removed and a replaced one restored" {
    tue_test_complete() {
        COMPREPLY=()
    }
    complete -F tue_test_complete tue-test-keep
    _tue-env-track-begin
    complete -o nospace -F tue_test_complete tue-test-new
    complete -o default -F tue_test_complete tue-test-keep
    _tue-env-track-commit
    _tue-env-track-revert
    [[ -z "$(complete -p tue-test-new 2> /dev/null)" ]]
    [[ "$(complete -p tue-test-keep)" == "complete -F tue_test_complete tue-test-keep" ]]
}

@test "revert: the ledger is empty afterwards" {
    _tue-env-track-begin
    export TUE_TEST_NEW=1
    alias tue_test_a='echo a'
    _tue-env-track-commit
    _tue-env-track-revert
    __tue_env_track_empty
}

@test "revert: an empty ledger returns 1 and changes nothing" {
    export TUE_TEST_UNTOUCHED=1
    run _tue-env-track-revert
    [[ "${status}" -eq 1 ]]
    [[ -z "${output}" ]]
    [[ "${TUE_TEST_UNTOUCHED}" == "1" ]]
}

@test "revert: a second revert with nothing left returns 1" {
    _tue-env-track-begin
    export TUE_TEST_NEW=1
    _tue-env-track-commit
    _tue-env-track-revert
    run _tue-env-track-revert
    [[ "${status}" -eq 1 ]]
}
