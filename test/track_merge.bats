load helpers/track

setup() {
    tue_track_setup
}

@test "depth: a nested begin/commit pair contributes to the outer load" {
    _tue-env-track-begin
    export TUE_TEST_OUTER=1
    _tue-env-track-begin
    export TUE_TEST_INNER=1
    _tue-env-track-commit
    [[ -z "${__TUE_ENV_LEDGER_VAR[TUE_TEST_INNER]:-}" ]]
    _tue-env-track-commit
    [[ "${__TUE_ENV_LEDGER_VAR[TUE_TEST_OUTER]}" == "added" ]]
    [[ "${__TUE_ENV_LEDGER_VAR[TUE_TEST_INNER]}" == "added" ]]
    [[ "${__TUE_ENV_TRACK_DEPTH}" -eq 0 ]]
}

@test "depth: a commit without a begin does nothing" {
    _tue-env-track-commit
    [[ "${__TUE_ENV_TRACK_DEPTH}" -eq 0 ]]
    __tue_env_track_empty
}

@test "merge: the original pre-load state is kept and the new post-load state taken" {
    export TUE_TEST_S=user
    _tue-env-track-begin
    export TUE_TEST_S=load1
    _tue-env-track-commit
    _tue-env-track-begin
    export TUE_TEST_S=load2
    _tue-env-track-commit
    [[ "${__TUE_ENV_LEDGER_VAR[TUE_TEST_S]}" == "replaced" ]]
    [[ "${__TUE_ENV_LEDGER_VAR_PRE[TUE_TEST_S]}" == 'declare -x TUE_TEST_S="user"' ]]
    [[ "${__TUE_ENV_LEDGER_VAR_POST[TUE_TEST_S]}" == 'declare -x TUE_TEST_S="load2"' ]]
}

@test "merge: a change made by hand between two loads is not attributed to the environment" {
    _tue-env-track-begin
    export TUE_TEST_A=1
    _tue-env-track-commit
    export TUE_TEST_MINE=mine
    _tue-env-track-begin
    export TUE_TEST_B=2
    _tue-env-track-commit
    [[ -z "${__TUE_ENV_LEDGER_VAR[TUE_TEST_MINE]:-}" ]]
    [[ "${__TUE_ENV_LEDGER_VAR[TUE_TEST_A]}" == "added" ]]
    [[ "${__TUE_ENV_LEDGER_VAR[TUE_TEST_B]}" == "added" ]]
}

@test "merge: added on one load and unset on the next drops out of the ledger" {
    _tue-env-track-begin
    export TUE_TEST_TMP=1
    _tue-env-track-commit
    _tue-env-track-begin
    unset TUE_TEST_TMP
    _tue-env-track-commit
    [[ -z "${__TUE_ENV_LEDGER_VAR[TUE_TEST_TMP]:-}" ]]
    [[ -z "${__TUE_ENV_LEDGER_VAR_PRE[TUE_TEST_TMP]:-}" ]]
}

@test "merge: a value the user set by hand is handed back when a later load overwrites it" {
    _tue-env-track-begin
    export TUE_TEST_OTHER=1
    _tue-env-track-commit
    export TUE_TEST_MINE=mine
    _tue-env-track-begin
    export TUE_TEST_MINE=fromenv
    _tue-env-track-commit
    [[ "${__TUE_ENV_LEDGER_VAR[TUE_TEST_MINE]}" == "replaced" ]]
    [[ "${__TUE_ENV_LEDGER_VAR_PRE[TUE_TEST_MINE]}" == 'declare -x TUE_TEST_MINE="mine"' ]]
}

@test "merge: the added entries of an extended variable are unioned" {
    export TUE_TEST_LIST="/usr/bin"
    _tue-env-track-begin
    export TUE_TEST_LIST="/one:${TUE_TEST_LIST}"
    _tue-env-track-commit
    _tue-env-track-begin
    export TUE_TEST_LIST="/two:${TUE_TEST_LIST}"
    _tue-env-track-commit
    [[ "${__TUE_ENV_LEDGER_VAR[TUE_TEST_LIST]}" == "extended" ]]
    [[ "$(tue_track_added TUE_TEST_LIST)" == "0=/one,0=/two" ]]
    [[ "${__TUE_ENV_LEDGER_VAR_PRE[TUE_TEST_LIST]}" == 'declare -x TUE_TEST_LIST="/usr/bin"' ]]
}

@test "merge: a variable first added and then extended stays entry-wise" {
    _tue-env-track-begin
    export TUE_TEST_PP="/one"
    _tue-env-track-commit
    export TUE_TEST_PP="${TUE_TEST_PP}:/mine"
    _tue-env-track-begin
    export TUE_TEST_PP="/two:${TUE_TEST_PP}"
    _tue-env-track-commit
    [[ "${__TUE_ENV_LEDGER_VAR[TUE_TEST_PP]}" == "extended" ]]
    [[ "$(tue_track_added TUE_TEST_PP)" == "0=/one,0=/two" ]]
}

@test "merge: a function replaced on two loads keeps the first pre-load body" {
    tue_test_fn() {
        echo original
    }
    _tue-env-track-begin
    tue_test_fn() {
        echo load1
    }
    _tue-env-track-commit
    _tue-env-track-begin
    tue_test_fn() {
        echo load2
    }
    _tue-env-track-commit
    [[ "${__TUE_ENV_LEDGER_FUNC[tue_test_fn]}" == "replaced" ]]
    [[ "${__TUE_ENV_LEDGER_FUNC_PRE[tue_test_fn]}" == *original* ]]
    [[ "${__TUE_ENV_LEDGER_FUNC_POST[tue_test_fn]}" == *load2* ]]
}

@test "merge: an alias added and then removed drops out of the ledger" {
    _tue-env-track-begin
    alias tue_test_a='echo a'
    _tue-env-track-commit
    _tue-env-track-begin
    unalias tue_test_a
    _tue-env-track-commit
    [[ -z "${__TUE_ENV_LEDGER_ALIAS[tue_test_a]:-}" ]]
}

@test "merge: a name already in the ledger keeps its original absent pre-load state" {
    _tue-env-track-begin
    export TUE_TEST_S=env1
    _tue-env-track-commit
    export TUE_TEST_S=mine
    _tue-env-track-begin
    export TUE_TEST_S=env2
    _tue-env-track-commit
    [[ "${__TUE_ENV_LEDGER_VAR[TUE_TEST_S]}" == "added" ]]
    [[ "${__TUE_ENV_LEDGER_VAR_PRE[TUE_TEST_S]}" == "" ]]
    [[ "${__TUE_ENV_LEDGER_VAR_POST[TUE_TEST_S]}" == 'declare -x TUE_TEST_S="env2"' ]]
}

@test "merge: a variable the environment replaced and then unset becomes removed" {
    export TUE_TEST_S=original
    _tue-env-track-begin
    export TUE_TEST_S=fromenv
    _tue-env-track-commit
    [[ "${__TUE_ENV_LEDGER_VAR[TUE_TEST_S]}" == "replaced" ]]
    _tue-env-track-begin
    unset TUE_TEST_S
    _tue-env-track-commit
    [[ "${__TUE_ENV_LEDGER_VAR[TUE_TEST_S]}" == "removed" ]]
    [[ "${__TUE_ENV_LEDGER_VAR_PRE[TUE_TEST_S]}" == 'declare -x TUE_TEST_S="original"' ]]
    [[ "${__TUE_ENV_LEDGER_VAR_POST[TUE_TEST_S]}" == "" ]]
    [[ -z "$(tue_track_added TUE_TEST_S)" ]]
}

@test "merge: a function added on one load and removed on the next drops out of the ledger" {
    _tue-env-track-begin
    tue_test_fn() {
        echo added
    }
    _tue-env-track-commit
    [[ "${__TUE_ENV_LEDGER_FUNC[tue_test_fn]}" == "added" ]]
    _tue-env-track-begin
    unset -f tue_test_fn
    _tue-env-track-commit
    [[ -z "${__TUE_ENV_LEDGER_FUNC[tue_test_fn]:-}" ]]
    [[ -z "${__TUE_ENV_LEDGER_FUNC_PRE[tue_test_fn]:-}" ]]
    [[ -z "${__TUE_ENV_LEDGER_FUNC_XPRE[tue_test_fn]:-}" ]]
}

@test "merge: a completion replaced on two loads keeps the original registration" {
    tue_test_complete() {
        COMPREPLY=()
    }
    complete -F tue_test_complete tue-test-cmd
    _tue-env-track-begin
    complete -o nospace -F tue_test_complete tue-test-cmd
    _tue-env-track-commit
    _tue-env-track-begin
    complete -o default -F tue_test_complete tue-test-cmd
    _tue-env-track-commit
    [[ "${__TUE_ENV_LEDGER_COMPLETE[tue-test-cmd]}" == "replaced" ]]
    [[ "${__TUE_ENV_LEDGER_COMPLETE_PRE[tue-test-cmd]}" == \
       "complete -F tue_test_complete tue-test-cmd" ]]
    [[ "${__TUE_ENV_LEDGER_COMPLETE_POST[tue-test-cmd]}" == \
       "complete -o default -F tue_test_complete tue-test-cmd" ]]
}
