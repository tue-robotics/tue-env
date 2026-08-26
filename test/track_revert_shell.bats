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

@test "revert: an alias the user had with an empty value survives the unload" {
    # The data-loss half of the empty-alias conflation: the ledger called it `added`, so the unload
    # removed it instead of putting the user's own alias back.
    alias tue_test_empty=''
    _tue-env-track-begin
    alias tue_test_empty='echo fromenv'
    _tue-env-track-commit
    _tue-env-track-revert
    [[ -n "${BASH_ALIASES[tue_test_empty]+set}" ]]
    [[ "${BASH_ALIASES[tue_test_empty]}" == "" ]]
}

@test "revert: an alias the environment added with an empty value is removed" {
    # The leak half: nothing was recorded at all, so the environment's alias survived the unload.
    _tue-env-track-begin
    alias tue_test_empty=''
    _tue-env-track-commit
    [[ "${__TUE_ENV_LEDGER_ALIAS[tue_test_empty]}" == "added" ]]
    _tue-env-track-revert
    [[ -z "${BASH_ALIASES[tue_test_empty]+set}" ]]
}

@test "revert: an alias the environment removed and the user re-created empty is kept" {
    # The conflict check has the same blind spot: an alias that is present with an empty value reads
    # exactly like an absent one, so the revert saw no change since the load and overwrote the
    # user's alias with the pre-load text.
    alias tue_test_a='echo original'
    _tue-env-track-begin
    unalias tue_test_a
    _tue-env-track-commit
    [[ "${__TUE_ENV_LEDGER_ALIAS[tue_test_a]}" == "removed" ]]
    alias tue_test_a=''
    run _tue-env-track-revert
    [[ "${output}" == *"kept your version of alias tue_test_a"* ]]
    _tue-env-track-revert
    [[ -n "${BASH_ALIASES[tue_test_a]+set}" ]]
    [[ "${BASH_ALIASES[tue_test_a]}" == "" ]]
}

@test "revert: an alias value holding a record separator is restored byte-exactly" {
    # The framing bytes used to end the record early: a value with an RS in the middle came back
    # truncated at the byte, and one that began with an RS came back destroyed.
    local __tue_env_want=$'echo foo\x1eBAR'
    alias "tue_test_rs=${__tue_env_want}"
    _tue-env-track-begin
    alias tue_test_rs='echo fromenv'
    _tue-env-track-commit
    _tue-env-track-revert
    [[ -n "${BASH_ALIASES[tue_test_rs]+set}" ]]
    [[ "${BASH_ALIASES[tue_test_rs]}" == "${__tue_env_want}" ]]
}

@test "revert: an alias value that begins with a record separator is not destroyed" {
    local __tue_env_want=$'\x1eecho mine'
    alias "tue_test_rs=${__tue_env_want}"
    _tue-env-track-begin
    alias tue_test_rs='echo fromenv'
    _tue-env-track-commit
    _tue-env-track-revert
    [[ -n "${BASH_ALIASES[tue_test_rs]+set}" ]]
    [[ "${BASH_ALIASES[tue_test_rs]}" == "${__tue_env_want}" ]]
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

@test "revert: a function whose own body holds a record separator is restored, not just spared" {
    # Function bodies are captured raw - `declare -f` writes into the snapshot's own command
    # substitution, and pulling its output into a variable to escape it would cost a fork per
    # function. A literal RS byte in the user's function therefore still splits its record, and the
    # ledger used to keep only the piece before the byte: the eval that restores it could not parse,
    # and the user was left with the environment's version instead of their own. The parse rejoins
    # the pieces now, so the body that comes back is the body that went in.
    eval "tue_test_fn() { echo \$'\x1e'original; }"
    local __tue_env_want
    __tue_env_want="$(declare -f tue_test_fn)"
    _tue-env-track-begin
    tue_test_fn() {
        echo replaced
    }
    _tue-env-track-commit
    [[ "${__TUE_ENV_LEDGER_FUNC[tue_test_fn]}" == "replaced" ]]
    # Not `run`: that evaluates its command inside a command substitution, so anything the revert
    # does to the function would be undone by the subshell exiting and could never be asserted on.
    local __tue_env_status=0
    __tue_env_track_revert_funcs || __tue_env_status=$?
    [[ "${__tue_env_status}" -eq 0 ]]
    [[ -n "$(declare -F tue_test_fn)" ]]
    [[ "$(declare -f tue_test_fn)" == "${__tue_env_want}" ]]
    [[ "$(tue_test_fn)" == $'\x1eoriginal' ]]
}

@test "revert: a function body cannot forge a record for a variable the load never touched" {
    # The worst of the raw-body cases. A body carrying a record separator followed by what looks
    # like the start of a variable record used to have its tail parsed as a record of its own,
    # overwriting the real pre-load state of a variable it names - so the unload either restored a
    # value the user never had or, as here, saw no change at all and left the environment's value in
    # place. Every record carries __TUE_ENV_MARK now, which a body written before the tracker ran
    # cannot be carrying.
    export TUE_TEST_PRECIOUS="the user's own value"
    eval "tue_test_inj() { echo \$'\x1eV\x1fTUE_TEST_PRECIOUS\x1fdeclare -x TUE_TEST_PRECIOUS=\"INJECTED\"'; }"
    _tue-env-track-begin
    export TUE_TEST_PRECIOUS="from the environment"
    _tue-env-track-commit
    [[ "${__TUE_ENV_LEDGER_VAR[TUE_TEST_PRECIOUS]}" == "replaced" ]]
    [[ "${__TUE_ENV_LEDGER_VAR_PRE[TUE_TEST_PRECIOUS]}" == *"the user's own value"* ]]
    __tue_env_track_revert_vars
    [[ "${TUE_TEST_PRECIOUS}" == "the user's own value" ]]
}

@test "revert: KNOWN LIMITATION - a body carrying the marker itself still forges a record" {
    # CHARACTERISATION TEST. Everything it asserts is a BUG being recorded, not a contract being
    # kept. It exists so that this residual cannot change without somebody noticing; do not read a
    # single assertion in it as desirable behaviour.
    #
    # The marker is a heuristic, not a guarantee. It works because a function body written before
    # the tracker ran cannot be carrying the marker - but a body that DOES carry the literal
    # sequence RS + TUEENVREC + FS forges a record exactly as a bare RS + V + FS used to before the
    # marker existed, and the test above no longer catches it. Escaping the body would close the
    # hole for good and cannot be afforded: `declare -f` writes straight into the snapshot's own
    # command substitution, and pulling its output into a variable in order to escape it costs a
    # fork per function - measured at 62 forks for 20 names and 422 for 200, against a flat 12.
    #
    # The literal below is deliberate: the forgery is only possible because the marker is a
    # compile-time constant that a body can carry verbatim. So if anyone ever closes the hole - a
    # per-snapshot nonce in the marker is the obvious candidate - this test FAILS, loudly and on
    # purpose. That is the whole point of it. Do not paper over the failure: delete this test and
    # let the one above it, which asserts the CORRECT behaviour, cover the case instead.
    [[ "${__TUE_ENV_MARK}" == "TUEENVREC" ]]  # the eval below has to carry the REAL marker
    export TUE_TEST_PRECIOUS="the user's own value"
    # The body of the forgery, in two pieces only because one line of it would run past 120 columns:
    # a record separator, the marker, and then a variable record naming a variable of the user's.
    local __tue_env_forged="\$'\x1eTUEENVREC\x1fV\x1fTUE_TEST_PRECIOUS"
    __tue_env_forged+="\x1fdeclare -x TUE_TEST_PRECIOUS=\"INJECTED\"'"
    eval "tue_test_inj() { echo ${__tue_env_forged}; }"

    # Directly: the rejoin hands the forged tail to the record parse, so the body is truncated at
    # its own RS and a variable the snapshot already recorded correctly is overwritten by the body.
    tue_track_snapshot PRE
    [[ "${__TUE_ENV_PRE_FUNC[tue_test_inj]}" != "$(declare -f tue_test_inj)" ]]
    [[ "${__TUE_ENV_PRE_VAR[TUE_TEST_PRECIOUS]}" == *"INJECTED"* ]]

    # End to end: the forgery lands in the pre-load AND the post-load snapshot, so the two agree
    # about the variable and the load's real change to it is never recorded...
    _tue-env-track-begin
    export TUE_TEST_PRECIOUS="from the environment"
    _tue-env-track-commit
    [[ -z "${__TUE_ENV_LEDGER_VAR[TUE_TEST_PRECIOUS]:-}" ]]

    # ...and the unload therefore leaves the environment's value in the user's shell. What SHOULD
    # happen here is what the test above it gets: "the user's own value" back.
    __tue_env_track_revert_vars
    [[ "${TUE_TEST_PRECIOUS}" == "from the environment" ]]
}

@test "revert: an export -f the environment added is cleared from a restored function" {
    # Guards the half of the above that the vanished `unset -f` used to cover for free: the pre-load
    # function was not exported, the environment exported it, and the restore has to take that flag
    # back off with `export -nf` rather than by destroying and re-creating the function.
    tue_test_fn() {
        echo original
    }
    _tue-env-track-begin
    tue_test_fn() {
        echo replaced
    }
    export -f tue_test_fn
    _tue-env-track-commit
    _tue-env-track-revert
    [[ "$(tue_test_fn)" == "original" ]]
    [[ -z "$(declare -Fx | grep ' tue_test_fn$')" ]]
}
