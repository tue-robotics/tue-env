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

@test "capture: the escape round-trips every framing byte byte-exactly" {
    # The wire format frames records with RS, fields with FS and index/entry pairs with PS, and a
    # payload that holds one of those bytes used to end its record or its field early: an alias value
    # was silently truncated at the byte, and one that started with the byte was destroyed outright.
    # The escape has to be exact in both directions for every arrangement of those bytes and of the
    # escape byte itself - one at the very start, one at the very end, several, and none at all - or
    # it trades one corruption for another.
    local __tue_env_t __tue_env_i
    local -a __tue_env_cases=(
        ''
        'plain text, no framing bytes at all'
        $'\x1e' $'\x1f' $'\x1d' $'\x1b'
        $'\x1eleading record separator'
        $'trailing record separator\x1e'
        $'\x1bleading escape'
        $'trailing escape\x1b'
        $'\x1e\x1e\x1e' $'\x1b\x1b\x1b' $'\x1f\x1f'
        $'\x1b0' $'\x1b1' $'\x1b2' $'\x1b3' $'\x1b\x1b1'
        $'a\x1eb\x1fc\x1dd\x1be'
        $'\x1e\x1f\x1d\x1ba mix of every one of them\x1b\x1d\x1f\x1e'
        $'echo foo\x1eBAR'
        $'\x1eecho mine'
    )
    for (( __tue_env_i = 0; __tue_env_i < ${#__tue_env_cases[@]}; __tue_env_i++ ))
    do
        __tue_env_t="${__tue_env_cases[__tue_env_i]}"
        __tue_env_track_escape "${__tue_env_t}"
        # no framing byte survives the escape, whatever went in
        [[ "${__TUE_ENV_ESCAPED}" != *$'\x1e'* ]]
        [[ "${__TUE_ENV_ESCAPED}" != *$'\x1f'* ]]
        [[ "${__TUE_ENV_ESCAPED}" != *$'\x1d'* ]]
        __tue_env_track_unescape "${__TUE_ENV_ESCAPED}"
        [[ "${__TUE_ENV_UNESCAPED}" == "${__tue_env_t}" ]]
    done
}

@test "capture: an alias value holding framing bytes survives the round trip byte-exactly" {
    local __tue_env_t __tue_env_i
    local -a __tue_env_cases=(
        'plain'
        $'echo foo\x1eBAR'
        $'\x1eecho mine'
        $'echo mine\x1e'
        $'echo a\x1fb'
        $'echo a\x1db'
        $'echo a\x1bb'
        $'\x1b1\x1e\x1b0'
        $'\x1e\x1e'
    )
    for (( __tue_env_i = 0; __tue_env_i < ${#__tue_env_cases[@]}; __tue_env_i++ ))
    do
        __tue_env_t="${__tue_env_cases[__tue_env_i]}"
        alias "tue_test_rs=${__tue_env_t}"
        tue_track_snapshot PRE
        [[ "${__TUE_ENV_PRE_ALIAS[tue_test_rs]}" == "${__tue_env_t}" ]]
    done
}

@test "capture: a function body holding a record separator is captured whole" {
    # `declare -f` writes straight into the snapshot's command substitution, so its output is the one
    # payload that cannot be escaped without paying a fork per function. The parse puts a body a raw
    # RS split back together instead; what comes out has to be exactly what `declare -f` printed.
    eval "tue_test_fn() { echo \$'\x1e'original; }"
    eval "tue_test_two() { echo \$'\x1e\x1e'twice\$'\x1e'; }"
    tue_track_snapshot PRE
    [[ "${__TUE_ENV_PRE_FUNC[tue_test_fn]}" == "$(declare -f tue_test_fn)" ]]
    [[ "${__TUE_ENV_PRE_FUNC[tue_test_two]}" == "$(declare -f tue_test_two)" ]]
    # the split body did not become a record of its own, and the records after it still parsed
    [[ -z "${__TUE_ENV_PRE_FUNC[original]:-}" ]]
    [[ -n "${__TUE_ENV_PRE_VAR[PATH]:-}" ]]
}

@test "capture: a variable's declare line carries no raw framing byte, so it is never escaped" {
    # Variables must not go through the escape the other payloads go through, and the reason is that
    # they cannot need it: `declare -p` renders every control byte as printable `$'\036'` text, for a
    # scalar, for an array's elements and for an associative array's keys alike. Pin that, because it
    # is the whole argument - if the capture ever stopped going through `declare -p`, or bash ever
    # stopped quoting, a value would start carrying framing bytes onto the wire and the escape would
    # have to cover it too. Then pin what the user actually gets: the value back, byte for byte.
    local __tue_env_want=$'a\x1eb\x1fc\x1dd\x1be\x1b1f\x1b0g'
    TUE_TEST_ESCY="${__tue_env_want}"
    TUE_TEST_ESCARR=($'x\x1by' $'p\x1eq')
    declare -A TUE_TEST_ESCMAP=([$'k\x1b1']=$'v\x1e')
    local __tue_env_l
    for __tue_env_l in "$(declare -p TUE_TEST_ESCY)" "$(declare -p TUE_TEST_ESCARR)" \
                       "$(declare -p TUE_TEST_ESCMAP)"
    do
        [[ "${__tue_env_l}" != *$'\x1e'* ]]
        [[ "${__tue_env_l}" != *$'\x1f'* ]]
        [[ "${__tue_env_l}" != *$'\x1d'* ]]
        [[ "${__tue_env_l}" != *$'\x1b'* ]]
        # Say the same thing once more in the form that survives the escape growing an alphabet:
        # escaping a `declare -p` line is the IDENTITY. The four assertions above only rule out the
        # four bytes the escape happens to cover today, so widening it - to backslash, say, which
        # every one of these lines carries because that is how bash quotes a control byte - would
        # leave all four green while variables silently started arriving on the wire in a form the
        # parse does not unescape.
        __tue_env_track_escape "${__tue_env_l}"
        [[ "${__TUE_ENV_ESCAPED}" == "${__tue_env_l}" ]]
    done

    tue_track_snapshot PRE
    [[ "${__TUE_ENV_PRE_VAR[TUE_TEST_ESCY]}" == "$(declare -p TUE_TEST_ESCY)" ]]
    [[ "${__TUE_ENV_PRE_VAR[TUE_TEST_ESCARR]}" == "$(declare -p TUE_TEST_ESCARR)" ]]

    _tue-env-track-begin
    TUE_TEST_ESCY=changed
    _tue-env-track-commit
    _tue-env-track-revert
    [[ "${TUE_TEST_ESCY}" == "${__tue_env_want}" ]]
}

@test "capture: two snapshots of an unchanged shell are byte-identical" {
    # The record token is drawn by the CALLER of the dump, never inside it, so a single dump stays a
    # pure function of the shell and the nonce cannot make two back-to-back dumps differ. Drawing it
    # inside would break exactly this, and on the bash 5.0 floor `$RANDOM` inside a command
    # substitution is not guaranteed to be reseeded per subshell anyway.
    local __tue_env_a __tue_env_b
    __tue_env_a="$(__tue_env_track_dump)"
    __tue_env_b="$(__tue_env_track_dump)"
    [[ "${__tue_env_a}" == "${__tue_env_b}" ]]
}

@test "capture: every snapshot is framed with a record token drawn for that snapshot" {
    # What closes the forgery: a body already defined in the shell cannot be carrying a number that
    # had not been drawn when it was written. Assert the three things that makes true - the token
    # changes between draws, it is the base plus digits only so it can never pick up a framing byte,
    # and the stream really is framed with the token that was live when it was taken.
    local __tue_env_a __tue_env_b __tue_env_ma __tue_env_mb __tue_env_digits
    __tue_env_track_nonce
    __tue_env_ma="${__TUE_ENV_MARK}"
    __tue_env_a="$(__tue_env_track_dump)"
    __tue_env_track_nonce
    __tue_env_mb="${__TUE_ENV_MARK}"
    __tue_env_b="$(__tue_env_track_dump)"

    [[ "${__tue_env_ma}" != "${__tue_env_mb}" ]]

    # The base plus digits and nothing else: a token that could pick up a framing byte of its own
    # would end its record before the record had begun.
    __tue_env_digits="${__tue_env_ma#"${__TUE_ENV_MARK_BASE}"}"
    [[ -n "${__tue_env_digits}" ]]
    [[ -z "${__tue_env_digits//[0-9]/}" ]]

    [[ "${__tue_env_a}" == "${__tue_env_ma}${__TUE_ENV_FS}"* ]]
    [[ "${__tue_env_b}" == "${__tue_env_mb}${__TUE_ENV_FS}"* ]]
}

@test "capture: the parse reads the token out of the stream, not out of the live global" {
    # The reason the nonce is safe at all. setup.bash is re-sourced from INSIDE the tracked span, so
    # __TUE_ENV_MARK does not still hold the token the pre-load stream was framed with by the time
    # that stream is parsed. A parse that read the global would match nothing, classify the whole
    # shell as `added`, and have the unload destroy variables the user owned.
    export TUE_TEST_DERIVED=1
    local __tue_env_snap
    __tue_env_track_nonce
    __tue_env_snap="$(__tue_env_track_dump)"
    # Stand in for what a re-source does to the global between the dump and the parse of its stream.
    __TUE_ENV_MARK="${__TUE_ENV_MARK_BASE}"
    __tue_env_track_parse "${__tue_env_snap}" PRE
    [[ -n "${__TUE_ENV_PRE_VAR[TUE_TEST_DERIVED]:-}" ]]
}

@test "capture: an empty or unframed stream parses to nothing rather than to nonsense" {
    # The token is derived from the stream's first record, so a stream that carries no token has to
    # leave the arrays as the parse cleared them - empty - rather than adopting whatever that first
    # record's first field happens to be as a token and reading somebody's function body as records.
    # The five cases are: nothing at all; a fragment with no separator in it; a first field that is
    # empty; a first RECORD that is empty; and a first field that is somebody else's text. The last
    # three carry a well-formed-looking forged variable record behind that first field. Cases three
    # and five are the ones that forge if the token's constant base stops being checked; case four
    # is the empty-first-record path, which no stream the dump produces can take either.
    export TUE_TEST_TOCLEAR=1
    tue_track_snapshot PRE
    [[ -n "${__TUE_ENV_PRE_VAR[TUE_TEST_TOCLEAR]:-}" ]]

    local __tue_env_case
    for __tue_env_case in "" "no separators here at all" \
        $'\x1fV\x1fTUE_TEST_TOCLEAR\x1fdeclare -x TUE_TEST_TOCLEAR="FORGED"\x1e' \
        $'\x1eTUEENVREC1\x1fV\x1fTUE_TEST_TOCLEAR\x1fdeclare -x TUE_TEST_TOCLEAR="FORGED"\x1e' \
        $'garbage\x1fV\x1fTUE_TEST_TOCLEAR\x1fdeclare -x TUE_TEST_TOCLEAR="FORGED"\x1e'
    do
        __tue_env_track_parse "${__tue_env_case}" PRE
        [[ "${#__TUE_ENV_PRE_VAR[@]}" -eq 0 ]]
        [[ "${#__TUE_ENV_PRE_FUNC[@]}" -eq 0 ]]
        [[ "${#__TUE_ENV_PRE_ALIAS[@]}" -eq 0 ]]
        [[ "${#__TUE_ENV_PRE_COMPLETE[@]}" -eq 0 ]]
    done
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
