# Environment change tracking Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or
> superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Record what loading a `tue-env` environment does to the bash session, so unloading it undoes exactly that
and nothing else.

**Architecture:** A new `setup/tue-env-track.bash` snapshots the shell (variables, functions, aliases, completions)
before and after the load, diffs the two snapshots, and accumulates the diff into a per-shell *ledger* held in
associative arrays. `setup.bash` brackets the load with `_tue-env-track-begin` / `_tue-env-track-commit`, and
`_tue-env-deactivate-current-env` applies the ledger instead of its old unset heuristic, keeping that heuristic only
as the empty-ledger fallback.

**Tech Stack:** Bash 5 builtins only (`compgen`, `declare`, `complete`, `BASH_ALIASES`); bats-core for tests; no new
runtime dependency.

**Spec:** `docs/superpowers/specs/2026-08-21-env-change-tracking-design.md`

## Global Constraints

- Branch `feature/env-change-tracking`. The `no-commit-to-branch` pre-commit hook blocks commits to `master`.
- Bash 5.0+ only (namerefs, `BASH_ALIASES`). No python, no external command on the load path.
- The tracker runs on every interactive shell start: builtins only, and a constant number of forks per load — never
  a fork per name.
- The ledger is never exported and never written to disk.
- Naming rule, load-bearing: every global in `setup/tue-env-track.bash` is named `__TUE_ENV_*`, every local
  `__tue_env_*`. A snapshot taken inside a function sees the locals of every enclosing function, so both prefixes
  are excluded from tracking. A local without the prefix ends up in the ledger.
- Never write `(( x++ ))` and never end a function with a bare `cond && action`: both yield exit status 1 (verified),
  and bats runs test bodies under `set -e`, so either one aborts a test from inside a helper. Use
  `x=$(( x + 1 ))` and close every function with an explicit `return 0`.
- `shellcheck` must pass: pre-commit runs shellcheck v0.10.0.1 on `\S\.(bash|sh)`, CI runs `shellcheck --
  **/*.bash **/*.sh`. `.bats` files are matched by neither.
- Markdown is linted at 120 columns including code blocks (`.markdownlint.yaml`), YAML at 120 (`.yamllint`,
  `--strict`).
- bats-core is pinned to `v1.13.0` (commit `d6a46f2cc2d3025ee3ffb59991c6d93ef903e339`).
- New `.bash` files with a shebang that are not executable must be added to the
  `check-shebang-scripts-are-executable` exclude list in `.pre-commit-config.yaml`, as `setup.bash` and its
  siblings already are.

## Deviations from the spec

Each of these is a decision taken while turning the design into code, not a change of intent.

1. Aliases are captured through `BASH_ALIASES` rather than `alias -p`: the same information, without a
   fork and without having to split `alias -p` output back into per-name records.
2. A snapshot costs a constant handful of forks (one command substitution per category list) rather
   than the literal two the spec names. What matters — never a fork per name — holds.
3. A ledger entry is a set of parallel associative arrays keyed by name, one per field, instead of one
   framed string per name: a `declare -p` payload can contain any byte, the field separator included.
4. Readonly variables are not tracked at all. They can be neither re-declared nor unset, so an entry
   for one could never be applied.
5. `_tue-env-main` collects the load's status with `_tue-env-load "$@" || __tue_env_ret=$?` instead of
   the spec's `local ret=$?`, so that a caller running under `set -e` cannot skip the commit, and names
   the local `__tue_env_ret` so that the commit snapshot does not record it as a variable the load
   added.
6. `tue-env switch` gets a `begin`/`commit` pair of its own. It exports `TUE_ENV` and `TUE_ENV_DIR`
   before sourcing `setup.bash`, so without this both would sit outside the tracked span and leak past
   the next deactivate. The depth counter makes `setup.bash`'s inner pair a no-op, which is exactly
   what it is for.
7. The merge rules are spelled out for mixed kinds too (the table in Task 4); the spec's three bullets
   cover the cases where both loads classified a name the same way.

## File Structure

| File | Responsibility |
| --- | --- |
| `setup/tue-env-track.bash` (new) | The tracker: separators, exclusions, snapshot, diff, ledger, revert, report |
| `setup.bash` (modify) | Split into `_tue-env-bootstrap` / `_tue-env-load`; `_tue-env-main` becomes the wrapper |
| `setup/tue-env.bash` (modify) | Ledger-first deactivate, `tue-env changes`, `deactivate --dry-run`, help, completion |
| `test/install-bats.bash` (new) | Clones the pinned bats-core into `test/.bats` (gitignored); idempotent |
| `test/helpers/track.bash` (new) | Sources the tracker into a test, hides bats' bookkeeping, assertion helpers |
| `test/helpers/env.bash` (new) | Builds a throw-away tue-env installation in `BATS_TEST_TMPDIR` for Task 8 |
| `test/track_capture.bats` (new) | Snapshot and parse round trips, exclusions, hostile values |
| `test/track_diff_vars.bats` (new) | Variable classification: added / removed / extended / replaced |
| `test/track_diff_shell.bats` (new) | Function, alias and completion classification |
| `test/track_merge.bats` (new) | Depth counter, and merging a second load into the ledger |
| `test/track_revert_vars.bats` (new) | Entry-wise removal, scalar restore, the conflict path |
| `test/track_revert_shell.bats` (new) | Function / alias / completion revert, `hash -r`, empty ledger |
| `test/track_report.bats` (new) | `changes` and dry-run renderings |
| `test/setup_integration.bats` (new) | A real `setup.bash` load in a fixture environment, incl. the `PS1` fix |
| `.gitignore`, `.pre-commit-config.yaml`, `main.yml` (modify) | Test dependency, lint exclude, CI job |
| `README.md`, `VERSION` (modify) | The two new commands, the revert contract, version bump |

## Shared reference

Every task below relies on these definitions. They are introduced in Task 1 and Task 2 and never change afterwards.

**Separators.** Newlines cannot frame records (`declare -p` emits literal newlines inside values and spans several
lines for arrays) and NUL cannot either (command substitution discards NUL bytes), so three control characters do it:

| Global | Value | Use |
| --- | --- | --- |
| `__TUE_ENV_RS` | `$'\x1e'` | between records in a snapshot, and between pairs in an added-entry list |
| `__TUE_ENV_FS` | `$'\x1f'` | between the fields of one snapshot record |
| `__TUE_ENV_PS` | `$'\x1d'` | between the index and the entry of one added-entry pair |

**Snapshot record grammar.** One record per name, each terminated by `RS`:

```text
V FS name FS <declare -p line>            RS
F FS name FS <"x" when exported> FS <declare -f body> RS
A FS name FS <alias value>                RS
C FS name FS <complete -p line>           RS
```

**Ledger.** Parallel associative arrays keyed by name, one set per category. The spec describes an entry as a
(kind, pre-load state, post-load state) triple; it is stored as parallel arrays rather than one framed string because
a `declare -p` payload may itself contain any byte, including the field separator.

| Array | Content |
| --- | --- |
| `__TUE_ENV_LEDGER_VAR` | kind: `added`, `removed`, `extended` or `replaced` |
| `__TUE_ENV_LEDGER_VAR_PRE` | pre-load `declare -p` line, empty when the name was absent |
| `__TUE_ENV_LEDGER_VAR_POST` | post-load `declare -p` line, empty when the name is absent |
| `__TUE_ENV_LEDGER_VAR_ADD` | added entries as `index PS entry` pairs joined by `RS` |
| `__TUE_ENV_LEDGER_FUNC`, `_PRE`, `_POST` | kind, pre-load `declare -f` body, post-load body |
| `__TUE_ENV_LEDGER_FUNC_XPRE` | `x` when the pre-load function carried `export -f` |
| `__TUE_ENV_LEDGER_ALIAS`, `_PRE`, `_POST` | kind, pre-load alias value, post-load alias value |
| `__TUE_ENV_LEDGER_COMPLETE`, `_PRE`, `_POST` | kind, pre-load `complete -p` line, post-load line |
| `__TUE_ENV_TRACK_DEPTH` | re-entrancy counter; snapshots happen only at the 0 boundary |

**Transient snapshot arrays.** `__tue_env_track_parse <stream> PRE|POST` fills
`__TUE_ENV_PRE_VAR`, `__TUE_ENV_PRE_FUNC`, `__TUE_ENV_PRE_FUNCX`, `__TUE_ENV_PRE_ALIAS`, `__TUE_ENV_PRE_COMPLETE` (and
the `POST_` set). They are cleared at the end of a commit.

**Entry points** (the four the rest of the repository calls):

| Function | Contract |
| --- | --- |
| `_tue-env-track-begin` | increments the depth counter; snapshots into `__TUE_ENV_SNAP_PRE` at 0 to 1 |
| `_tue-env-track-commit` | decrements it; at 0 snapshots, diffs, merges into the ledger, drops transients |
| `_tue-env-track-revert` | applies the ledger, clears it, `hash -r`; returns 1 on an empty ledger, changing nothing |
| `_tue-env-track-report changes\|revert` | renders the ledger or the planned revert; returns 1 on an empty ledger |

**Test conventions**, in `test/helpers/track.bash`:

- A test that holds a snapshot in a variable must name that variable `__tue_env_*`, or use the
  `tue_track_snapshot` helper. A plain `local pre` is itself a shell change: it is empty in the first snapshot and
  holds the whole first snapshot in the second, so the test would see it as a modified variable.
- `__TUE_ENV_TRACK_EXTRA_EXCLUDE` hides bats' own bookkeeping. `BATS_DEBUG_*` is rewritten by bats' DEBUG trap on
  every command, so without it every test sees spurious changes (verified).
- `run` runs its argument in a subshell: use it to assert on printed output, and call the function directly in a
  separate test to assert on shell state.
- Integration tests source `setup.bash` as `source ... || true`, which suppresses `set -e` for the whole dynamic
  extent of the load; without it a failing load aborts the test instead of returning.

---

### Task 1: Test harness and shell capture

**Files:**

- Create: `test/install-bats.bash`, `test/helpers/track.bash`, `setup/tue-env-track.bash`
- Create: `test/track_capture.bats`
- Modify: `.gitignore`, `.pre-commit-config.yaml`, `.github/workflows/main.yml`

**Interfaces:**

- Consumes: nothing.
- Produces: `__TUE_ENV_RS`, `__TUE_ENV_FS`, `__TUE_ENV_PS`, `__TUE_ENV_TRACK_EXTRA_EXCLUDE`, the ledger arrays,
  `__TUE_ENV_TRACK_DEPTH`, `__tue_env_track_excluded <name>`, `__tue_env_track_dump` (snapshot on stdout),
  `__tue_env_track_parse <stream> <PRE|POST>`, and the `__TUE_ENV_{PRE,POST}_{VAR,FUNC,FUNCX,ALIAS,COMPLETE}` arrays.

- [ ] **Step 1: Add the bats installer**

Create `test/install-bats.bash` and make it executable (`chmod +x`; pre-commit's
`check-executables-have-shebangs` and `check-shebang-scripts-are-executable` both apply to it):

```bash
#! /usr/bin/env bash
#
# Install the pinned bats-core into test/.bats. Idempotent; safe to run in CI and by hand.

set -e

BATS_VERSION="v1.13.0"
TEST_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
BATS_DIR="${TEST_DIR}/.bats"

if [[ -x "${BATS_DIR}/bin/bats" ]]
then
    installed="$("${BATS_DIR}"/bin/bats --version)"
    if [[ "${installed}" == "Bats ${BATS_VERSION#v}" ]]
    then
        echo "[test] ${installed} already installed in '${BATS_DIR}'"
        exit 0
    fi
    echo "[test] replacing '${installed}' by bats ${BATS_VERSION}"
    rm -rf "${BATS_DIR}"
fi

echo "[test] cloning bats-core ${BATS_VERSION} into '${BATS_DIR}'"
git clone --depth 1 --branch "${BATS_VERSION}" --quiet https://github.com/bats-core/bats-core.git "${BATS_DIR}"
"${BATS_DIR}"/bin/bats --version
```

- [ ] **Step 2: Ignore the test dependency and run the installer**

Append to `.gitignore`:

```text
test/.bats
```

Run: `./test/install-bats.bash`
Expected: `Bats 1.13.0`

- [ ] **Step 3: Add the test helper**

Create `test/helpers/track.bash`. It has no shebang on purpose — a `# shellcheck shell=bash` directive keeps
shellcheck happy without making the file executable:

```bash
# shellcheck shell=bash
#
# Common helper for the change-tracking tests:
#
#   load helpers/track
#
#   setup() {
#       tue_track_setup
#   }
#
# Any test variable that holds a snapshot must be named __tue_env_*, or the snapshot will contain it.

TUE_TRACK_REPO_ROOT="$( cd "${BATS_TEST_DIRNAME}/.." && pwd )"

function tue_track_setup
{
    # shellcheck source=/dev/null
    source "${TUE_TRACK_REPO_ROOT}/setup/tue-env-track.bash"

    # bats rewrites BATS_DEBUG_* from its DEBUG trap on every command, and `run` rewrites
    # output/status/lines; without hiding them every snapshot pair would differ.
    __TUE_ENV_TRACK_EXTRA_EXCLUDE=('BATS_*' 'output' 'status' 'lines' 'stderr' 'stderr_lines')
    return 0
}

function tue_track_snapshot
{
    # $1: PRE or POST. Snapshots this shell into the __TUE_ENV_$1_* arrays.
    local __tue_env_snap
    __tue_env_snap="$(__tue_env_track_dump)"
    __tue_env_track_parse "${__tue_env_snap}" "$1"
    return 0
}

function tue_track_added
{
    # $1: variable name. Prints the recorded added entries as "index=entry,index=entry".
    local __tue_env_r="${__TUE_ENV_LEDGER_VAR_ADD[$1]:-}" __tue_env_p __tue_env_o=""
    while [[ -n "${__tue_env_r}" ]]
    do
        __tue_env_p="${__tue_env_r%%"${__TUE_ENV_RS}"*}"
        __tue_env_r="${__tue_env_r#*"${__TUE_ENV_RS}"}"
        [[ -z "${__tue_env_p}" ]] && continue
        __tue_env_o+="${__tue_env_o:+,}${__tue_env_p%%"${__TUE_ENV_PS}"*}"
        __tue_env_o+="=${__tue_env_p#*"${__TUE_ENV_PS}"}"
    done
    printf '%s' "${__tue_env_o}"
    return 0
}
```

- [ ] **Step 4: Write the failing capture tests**

Create `test/track_capture.bats`:

```bash
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
```

- [ ] **Step 5: Run the tests to verify they fail**

Run: `./test/.bats/bin/bats --print-output-on-failure test/track_capture.bats`
Expected: every test fails, `setup/tue-env-track.bash: No such file or directory`

- [ ] **Step 6: Write the tracker skeleton, exclusions, dump and parse**

Create `setup/tue-env-track.bash`:

```bash
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

    # One mapfile pass rather than shrinking the stream with ${x%%RS*} / ${x#*RS} per record: those
    # rescan the whole remaining string every iteration, which is quadratic in the snapshot size. On a
    # 77 KB snapshot the shrinking loop measured 1115 ms against 22 ms here, and
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
```

- [ ] **Step 7: Exempt the new file from the executable-shebang hook**

`setup/tue-env-track.bash` carries a shebang and is not executable, like its siblings. In
`.pre-commit-config.yaml`, insert `|^setup/tue-env-track\.bash$` into the
`check-shebang-scripts-are-executable` exclude pattern, right after `^setup/tue-env.bash$`.

- [ ] **Step 8: Run the tests to verify they pass**

Run: `./test/.bats/bin/bats --print-output-on-failure test/track_capture.bats`
Expected: 9 tests, all pass

Run: `shellcheck -- setup/tue-env-track.bash test/install-bats.bash test/helpers/track.bash`
Expected: no output

- [ ] **Step 9: Add the CI job**

In `.github/workflows/main.yml`, after the `linting_shellcheck` job:

```yaml
  test_bats:
    name: Bats
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v7
      - name: Install bats
        run: ./test/install-bats.bash
      - name: Run bats
        # test/*.bats, not `--recursive test`: the recursive walk descends into test/.bats, the
        # bats-core checkout, and gathers its own abort fixtures instead of this repo's tests.
        run: ./test/.bats/bin/bats --print-output-on-failure test/*.bats
```

Run: `yamllint --strict .github/workflows/main.yml`
Expected: no output

- [ ] **Step 10: Commit**

```bash
git add test .gitignore .pre-commit-config.yaml .github/workflows/main.yml setup/tue-env-track.bash
git commit -m "Add bats test harness and shell state capture for env change tracking"
```

---

### Task 2: Variable classification

**Files:**

- Modify: `setup/tue-env-track.bash`
- Create: `test/track_diff_vars.bats`

**Interfaces:**

- Consumes: `__tue_env_track_dump`, `__tue_env_track_parse`, the `__TUE_ENV_{PRE,POST}_VAR` arrays (Task 1).
- Produces:
  - `__tue_env_track_attrs <declare line>` → `__TUE_ENV_ATTRS`, the attribute letters without the leading dash
  - `__tue_env_track_listable <pre line> <post line>` → 0 when neither side is an array
  - `__tue_env_track_value <declare line>` → `__TUE_ENV_VALUE`, the plain scalar value
  - `__tue_env_track_entries <pre value> <post value>` → returns 0 when the pre entries are a subsequence of the
    post entries, and fills `__TUE_ENV_ADDED` with `index PS entry` pairs joined by `RS`
  - `__tue_env_track_diff_vars` → writes `added` / `removed` / `extended` / `replaced` entries for every changed
    variable into the four `__TUE_ENV_LEDGER_VAR*` arrays

- [ ] **Step 1: Write the failing classification tests**

Create `test/track_diff_vars.bats`:

```bash
load helpers/track

setup() {
    tue_track_setup
}

@test "diff: a new variable is added" {
    tue_track_snapshot PRE
    export TUE_TEST_NEW=1
    tue_track_snapshot POST
    __tue_env_track_diff_vars
    [[ "${__TUE_ENV_LEDGER_VAR[TUE_TEST_NEW]}" == "added" ]]
    [[ "${__TUE_ENV_LEDGER_VAR_PRE[TUE_TEST_NEW]}" == "" ]]
    [[ "${__TUE_ENV_LEDGER_VAR_POST[TUE_TEST_NEW]}" == 'declare -x TUE_TEST_NEW="1"' ]]
}

@test "diff: an unset variable is removed" {
    export TUE_TEST_GONE=1
    tue_track_snapshot PRE
    unset TUE_TEST_GONE
    tue_track_snapshot POST
    __tue_env_track_diff_vars
    [[ "${__TUE_ENV_LEDGER_VAR[TUE_TEST_GONE]}" == "removed" ]]
    [[ "${__TUE_ENV_LEDGER_VAR_PRE[TUE_TEST_GONE]}" == 'declare -x TUE_TEST_GONE="1"' ]]
    [[ "${__TUE_ENV_LEDGER_VAR_POST[TUE_TEST_GONE]}" == "" ]]
}

@test "diff: an untouched variable produces no entry" {
    export TUE_TEST_SAME=1
    tue_track_snapshot PRE
    tue_track_snapshot POST
    __tue_env_track_diff_vars
    [[ -z "${__TUE_ENV_LEDGER_VAR[TUE_TEST_SAME]:-}" ]]
}

@test "diff: a prepended list value is extended, with the indices of the added entries" {
    export TUE_TEST_LIST="/usr/bin:/bin"
    tue_track_snapshot PRE
    export TUE_TEST_LIST="/usr/lib/ccache:/opt/ros/jazzy/bin:${TUE_TEST_LIST}"
    tue_track_snapshot POST
    __tue_env_track_diff_vars
    [[ "${__TUE_ENV_LEDGER_VAR[TUE_TEST_LIST]}" == "extended" ]]
    [[ "$(tue_track_added TUE_TEST_LIST)" == "0=/usr/lib/ccache,1=/opt/ros/jazzy/bin" ]]
}

@test "diff: an appended duplicate entry records the new index" {
    export TUE_TEST_LIST="A:X:B"
    tue_track_snapshot PRE
    export TUE_TEST_LIST="A:X:B:X"
    tue_track_snapshot POST
    __tue_env_track_diff_vars
    [[ "${__TUE_ENV_LEDGER_VAR[TUE_TEST_LIST]}" == "extended" ]]
    [[ "$(tue_track_added TUE_TEST_LIST)" == "3=X" ]]
}

@test "diff: a reordered list is replaced, not extended" {
    export TUE_TEST_LIST="A:B"
    tue_track_snapshot PRE
    export TUE_TEST_LIST="B:A"
    tue_track_snapshot POST
    __tue_env_track_diff_vars
    [[ "${__TUE_ENV_LEDGER_VAR[TUE_TEST_LIST]}" == "replaced" ]]
}

@test "diff: a changed scalar is replaced" {
    export TUE_TEST_RMW=rmw_fastrtps_cpp
    tue_track_snapshot PRE
    export TUE_TEST_RMW=rmw_cyclonedds_cpp
    tue_track_snapshot POST
    __tue_env_track_diff_vars
    [[ "${__TUE_ENV_LEDGER_VAR[TUE_TEST_RMW]}" == "replaced" ]]
    [[ "${__TUE_ENV_LEDGER_VAR_PRE[TUE_TEST_RMW]}" == 'declare -x TUE_TEST_RMW="rmw_fastrtps_cpp"' ]]
}

@test "diff: a changed array is replaced, never extended" {
    TUE_TEST_ARR=(a)
    tue_track_snapshot PRE
    TUE_TEST_ARR=(a b)
    tue_track_snapshot POST
    __tue_env_track_diff_vars
    [[ "${__TUE_ENV_LEDGER_VAR[TUE_TEST_ARR]}" == "replaced" ]]
    [[ "${__TUE_ENV_LEDGER_VAR_POST[TUE_TEST_ARR]}" == "$(declare -p TUE_TEST_ARR)" ]]
}

@test "diff: a value holding a newline is replaced, never extended" {
    export TUE_TEST_NL="a:b"
    tue_track_snapshot PRE
    export TUE_TEST_NL=$'a:b\nc'
    tue_track_snapshot POST
    __tue_env_track_diff_vars
    [[ "${__TUE_ENV_LEDGER_VAR[TUE_TEST_NL]}" == "replaced" ]]
}

@test "diff: a readonly variable is not tracked at all" {
    tue_track_snapshot PRE
    readonly TUE_TEST_RO=1
    tue_track_snapshot POST
    __tue_env_track_diff_vars
    [[ -z "${__TUE_ENV_LEDGER_VAR[TUE_TEST_RO]:-}" ]]
}

@test "diff: an added scalar also records its entries, for a later merge" {
    tue_track_snapshot PRE
    export TUE_TEST_PP="/a:/b"
    tue_track_snapshot POST
    __tue_env_track_diff_vars
    [[ "${__TUE_ENV_LEDGER_VAR[TUE_TEST_PP]}" == "added" ]]
    [[ "$(tue_track_added TUE_TEST_PP)" == "0=/a,1=/b" ]]
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `./test/.bats/bin/bats --print-output-on-failure test/track_diff_vars.bats`
Expected: every test fails with `__tue_env_track_diff_vars: command not found`

- [ ] **Step 3: Implement the classification helpers**

Append to `setup/tue-env-track.bash`:

```bash
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
```

- [ ] **Step 4: Implement a straight-through ledger writer**

The merge rules land in Task 4; for now the writer stores what it is given. Append to
`setup/tue-env-track.bash`:

```bash
function __tue_env_track_ledger_var
{
    # $1: name, $2: kind, $3: pre-load declare line, $4: post-load declare line, $5: added entries.
    __TUE_ENV_LEDGER_VAR["$1"]="$2"
    __TUE_ENV_LEDGER_VAR_PRE["$1"]="$3"
    __TUE_ENV_LEDGER_VAR_POST["$1"]="$4"
    __TUE_ENV_LEDGER_VAR_ADD["$1"]="$5"
    return 0
}
```

- [ ] **Step 5: Run the tests to verify they pass**

Run: `./test/.bats/bin/bats --print-output-on-failure test/track_diff_vars.bats`
Expected: 11 tests, all pass

Run: `shellcheck -- setup/tue-env-track.bash`
Expected: no output

- [ ] **Step 6: Commit**

```bash
git add setup/tue-env-track.bash test/track_diff_vars.bats
git commit -m "Classify variable changes for env change tracking"
```

---

### Task 3: Function, alias and completion classification

**Files:**

- Modify: `setup/tue-env-track.bash`
- Create: `test/track_diff_shell.bats`

**Interfaces:**

- Consumes: the `__TUE_ENV_{PRE,POST}_{FUNC,FUNCX,ALIAS,COMPLETE}` arrays (Task 1).
- Produces:
  - `__tue_env_track_diff_funcs` → `added` / `removed` / `replaced` entries in `__TUE_ENV_LEDGER_FUNC*`
  - `__tue_env_track_diff_simple <ALIAS|COMPLETE>` → the same triple for the other two categories
  - `__tue_env_track_ledger_func <name> <kind> <pre> <post> <pre export flag>`
  - `__tue_env_track_ledger_simple <ALIAS|COMPLETE> <name> <kind> <pre> <post>`

- [ ] **Step 1: Write the failing tests**

Create `test/track_diff_shell.bats`:

```bash
load helpers/track

setup() {
    tue_track_setup
}

@test "diff: a new function is added and its export attribute recorded" {
    tue_track_snapshot PRE
    tue_test_fn() {
        echo new
    }
    export -f tue_test_fn
    tue_track_snapshot POST
    __tue_env_track_diff_funcs
    [[ "${__TUE_ENV_LEDGER_FUNC[tue_test_fn]}" == "added" ]]
    [[ "${__TUE_ENV_LEDGER_FUNC_PRE[tue_test_fn]}" == "" ]]
    [[ "${__TUE_ENV_LEDGER_FUNC_POST[tue_test_fn]}" == "$(declare -f tue_test_fn)" ]]
    [[ "${__TUE_ENV_LEDGER_FUNC_XPRE[tue_test_fn]}" == "" ]]
}

@test "diff: a redefined function is replaced and the pre-load export flag kept" {
    tue_test_fn() {
        echo original
    }
    export -f tue_test_fn
    tue_track_snapshot PRE
    unset -f tue_test_fn
    tue_test_fn() {
        echo replaced
    }
    tue_track_snapshot POST
    __tue_env_track_diff_funcs
    [[ "${__TUE_ENV_LEDGER_FUNC[tue_test_fn]}" == "replaced" ]]
    [[ "${__TUE_ENV_LEDGER_FUNC_XPRE[tue_test_fn]}" == "x" ]]
    [[ "${__TUE_ENV_LEDGER_FUNC_PRE[tue_test_fn]}" == *original* ]]
}

@test "diff: a removed function is removed" {
    tue_test_fn() {
        echo gone
    }
    tue_track_snapshot PRE
    unset -f tue_test_fn
    tue_track_snapshot POST
    __tue_env_track_diff_funcs
    [[ "${__TUE_ENV_LEDGER_FUNC[tue_test_fn]}" == "removed" ]]
    [[ "${__TUE_ENV_LEDGER_FUNC_POST[tue_test_fn]}" == "" ]]
}

@test "diff: an untouched function produces no entry" {
    tue_test_fn() {
        echo same
    }
    tue_track_snapshot PRE
    tue_track_snapshot POST
    __tue_env_track_diff_funcs
    [[ -z "${__TUE_ENV_LEDGER_FUNC[tue_test_fn]:-}" ]]
}

@test "diff: an added alias, a changed alias and a removed alias" {
    alias tue_test_keep='echo keep'
    alias tue_test_gone='echo gone'
    tue_track_snapshot PRE
    alias tue_test_new='echo new'
    alias tue_test_keep='echo changed'
    unalias tue_test_gone
    tue_track_snapshot POST
    __tue_env_track_diff_simple ALIAS
    [[ "${__TUE_ENV_LEDGER_ALIAS[tue_test_new]}" == "added" ]]
    [[ "${__TUE_ENV_LEDGER_ALIAS[tue_test_keep]}" == "replaced" ]]
    [[ "${__TUE_ENV_LEDGER_ALIAS_PRE[tue_test_keep]}" == "echo keep" ]]
    [[ "${__TUE_ENV_LEDGER_ALIAS[tue_test_gone]}" == "removed" ]]
}

@test "diff: an added and a removed completion" {
    tue_test_complete() {
        COMPREPLY=()
    }
    complete -F tue_test_complete tue-test-old
    tue_track_snapshot PRE
    complete -o nospace -F tue_test_complete tue-test-new
    complete -r tue-test-old
    tue_track_snapshot POST
    __tue_env_track_diff_simple COMPLETE
    [[ "${__TUE_ENV_LEDGER_COMPLETE[tue-test-new]}" == "added" ]]
    [[ "${__TUE_ENV_LEDGER_COMPLETE_POST[tue-test-new]}" == \
       "complete -o nospace -F tue_test_complete tue-test-new" ]]
    [[ "${__TUE_ENV_LEDGER_COMPLETE[tue-test-old]}" == "removed" ]]
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `./test/.bats/bin/bats --print-output-on-failure test/track_diff_shell.bats`
Expected: every test fails with `__tue_env_track_diff_funcs: command not found`

- [ ] **Step 3: Implement the two differs and the two ledger writers**

Append to `setup/tue-env-track.bash`:

```bash
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
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `./test/.bats/bin/bats --print-output-on-failure test/track_diff_shell.bats`
Expected: 6 tests, all pass

Run: `shellcheck -- setup/tue-env-track.bash`
Expected: no output

- [ ] **Step 5: Commit**

```bash
git add setup/tue-env-track.bash test/track_diff_shell.bats
git commit -m "Classify function, alias and completion changes for env change tracking"
```

---

### Task 4: Begin, commit and the ledger merge

**Files:**

- Modify: `setup/tue-env-track.bash`
- Create: `test/track_merge.bats`

**Interfaces:**

- Consumes: everything from Tasks 1 to 3.
- Produces: `_tue-env-track-begin`, `_tue-env-track-commit`, `__tue_env_track_empty` (returns 0 when the
  ledger holds nothing), and merging versions of the three ledger writers.

The merge rules, as a table. `pre` is always the **old** entry's pre-load state and `post` the **new**
post-load state; `old` and `new` are the two kinds:

| Condition | Result |
| --- | --- |
| `pre` and `post` both absent | the entry is dropped: added on one load, unset on the next, is a no-op |
| `post` absent | `removed` |
| `old` and `new` both `extended` | `extended`, added entries unioned as a multiset |
| `pre` absent, either kind `extended` | `extended`, entries unioned: list-shaped, may hold user entries |
| `pre` absent, neither kind `extended` | `added`, keeping the new entry list |
| otherwise | `replaced`: the environment overwrote the value, so only restoring the original is correct |

Two consequences of the spec worth keeping straight while implementing this. A name **already** in the
ledger keeps its original pre-load state, so a value the environment sets on two consecutive loads is
still unset on revert even if the user changed it in between — the environment, not the user, had the
last word. A name **not yet** in the ledger is inserted as-is, so a value the user set by hand and a
later load then overwrote gets the user's value as its pre-load state and is handed back on revert.

- [ ] **Step 1: Write the failing tests**

Create `test/track_merge.bats`:

```bash
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
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `./test/.bats/bin/bats --print-output-on-failure test/track_merge.bats`
Expected: every test fails with `_tue-env-track-begin: command not found`

- [ ] **Step 3: Implement the entry points and the emptiness check**

Append to `setup/tue-env-track.bash`:

```bash
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
```

- [ ] **Step 4: Turn the three ledger writers into merging writers**

Replace the body of `__tue_env_track_ledger_var` in `setup/tue-env-track.bash`:

```bash
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
```

Replace the body of `__tue_env_track_ledger_func`:

```bash
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
```

Replace the body of `__tue_env_track_ledger_simple`:

```bash
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
```

`unset "__tue_env_lk[$2]"` through a nameref removes the element from the array the nameref points at;
verify that in the "drops out of the ledger" tests rather than trusting it.

- [ ] **Step 5: Run the whole suite to verify nothing regressed**

Run: `./test/.bats/bin/bats --print-output-on-failure test/*.bats`
Expected: all tests pass, including the Task 2 and 3 tests — with an empty ledger, merging reduces to
inserting the entry as-is

Run: `shellcheck -- setup/tue-env-track.bash`
Expected: no output

- [ ] **Step 6: Commit**

```bash
git add setup/tue-env-track.bash test/track_merge.bats
git commit -m "Add tracked span entry points and ledger merging"
```

---

### Task 5: Reverting variables

**Files:**

- Modify: `setup/tue-env-track.bash`
- Create: `test/track_revert_vars.bats`

**Interfaces:**

- Consumes: the `__TUE_ENV_LEDGER_VAR*` arrays, `__tue_env_track_value`, `__tue_env_track_attrs`.
- Produces:
  - `__tue_env_track_restore_line <declare line>` — re-declares a captured variable globally
  - `__tue_env_track_strip <current value> <added entries>` → `__TUE_ENV_VALUE`
  - `__tue_env_track_kept <what>` — prints the one-line note about a kept user change
  - `__tue_env_track_revert_vars` — applies every `__TUE_ENV_LEDGER_VAR` entry

- [ ] **Step 1: Write the failing tests**

Create `test/track_revert_vars.bats`:

```bash
load helpers/track

setup() {
    tue_track_setup
}

@test "revert: entries are removed one by one and entries the user added survive" {
    export TUE_TEST_LIST="/usr/bin:/bin"
    _tue-env-track-begin
    export TUE_TEST_LIST="/usr/lib/ccache:/opt/ros/jazzy/bin:${TUE_TEST_LIST}"
    _tue-env-track-commit
    export TUE_TEST_LIST="/opt/foo/bin:${TUE_TEST_LIST}"
    __tue_env_track_revert_vars
    [[ "${TUE_TEST_LIST}" == "/opt/foo/bin:/usr/bin:/bin" ]]
}

@test "revert: a duplicate entry is removed exactly once, at the recorded index" {
    export TUE_TEST_LIST="A:X:B"
    _tue-env-track-begin
    export TUE_TEST_LIST="A:X:B:X"
    _tue-env-track-commit
    __tue_env_track_revert_vars
    [[ "${TUE_TEST_LIST}" == "A:X:B" ]]
}

@test "revert: an entry the ledger recorded but that is gone is skipped silently" {
    export TUE_TEST_LIST="/usr/bin"
    _tue-env-track-begin
    export TUE_TEST_LIST="/gone:${TUE_TEST_LIST}"
    _tue-env-track-commit
    export TUE_TEST_LIST="/usr/bin"
    __tue_env_track_revert_vars
    [[ "${TUE_TEST_LIST}" == "/usr/bin" ]]
}

@test "revert: an added list variable is unset when nothing is left of it" {
    _tue-env-track-begin
    export TUE_TEST_PP="/a:/b"
    _tue-env-track-commit
    __tue_env_track_revert_vars
    [[ -z "${TUE_TEST_PP+set}" ]]
}

@test "revert: an added list variable the user appended to is kept whole" {
    # `added` is not `extended`: the spec puts everything that is not entry-wise through the conflict
    # check, so a variable the environment created and the user then changed is handed back untouched.
    _tue-env-track-begin
    export TUE_TEST_PP="/a"
    _tue-env-track-commit
    export TUE_TEST_PP="${TUE_TEST_PP}:/mine"
    run __tue_env_track_revert_vars
    [[ "${output}" == *"kept your value for TUE_TEST_PP"* ]]
    __tue_env_track_revert_vars
    [[ "${TUE_TEST_PP}" == "/a:/mine" ]]
}

@test "revert: a replaced scalar is restored" {
    export TUE_TEST_RMW=rmw_fastrtps_cpp
    _tue-env-track-begin
    export TUE_TEST_RMW=rmw_cyclonedds_cpp
    _tue-env-track-commit
    __tue_env_track_revert_vars
    [[ "${TUE_TEST_RMW}" == "rmw_fastrtps_cpp" ]]
}

@test "revert: an added scalar is unset" {
    _tue-env-track-begin
    export TUE_TEST_NEW=1
    _tue-env-track-commit
    __tue_env_track_revert_vars
    [[ -z "${TUE_TEST_NEW+set}" ]]
}

@test "revert: a variable the environment removed is restored" {
    export TUE_TEST_GONE=mine
    _tue-env-track-begin
    unset TUE_TEST_GONE
    _tue-env-track-commit
    __tue_env_track_revert_vars
    [[ "${TUE_TEST_GONE}" == "mine" ]]
}

@test "revert: the user's later value is kept" {
    _tue-env-track-begin
    export TUE_TEST_RMW=rmw_cyclonedds_cpp
    _tue-env-track-commit
    export TUE_TEST_RMW=rmw_connext
    __tue_env_track_revert_vars
    [[ "${TUE_TEST_RMW}" == "rmw_connext" ]]
}

@test "revert: keeping the user's value prints one note naming it" {
    _tue-env-track-begin
    export TUE_TEST_RMW=rmw_cyclonedds_cpp
    _tue-env-track-commit
    export TUE_TEST_RMW=rmw_connext
    run __tue_env_track_revert_vars
    [[ "${status}" -eq 0 ]]
    [[ "${output}" == *"kept your value for TUE_TEST_RMW"* ]]
}

@test "revert: an array and its export attribute survive the round trip" {
    TUE_TEST_ARR=(one "two three")
    export TUE_TEST_X=original
    _tue-env-track-begin
    TUE_TEST_ARR=(replaced)
    export TUE_TEST_X=fromenv
    _tue-env-track-commit
    __tue_env_track_revert_vars
    [[ "${TUE_TEST_ARR[1]}" == "two three" ]]
    [[ "${#TUE_TEST_ARR[@]}" -eq 2 ]]
    [[ "$(declare -p TUE_TEST_X)" == 'declare -x TUE_TEST_X="original"' ]]
}

@test "revert: a value holding a newline and a quote is restored verbatim" {
    TUE_TEST_NASTY=$'a\nb:c d'\''e'
    local __tue_env_want="${TUE_TEST_NASTY}"
    _tue-env-track-begin
    TUE_TEST_NASTY=changed
    _tue-env-track-commit
    __tue_env_track_revert_vars
    [[ "${TUE_TEST_NASTY}" == "${__tue_env_want}" ]]
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `./test/.bats/bin/bats --print-output-on-failure test/track_revert_vars.bats`
Expected: every test fails with `__tue_env_track_revert_vars: command not found`

- [ ] **Step 3: Implement the restore helpers**

Append to `setup/tue-env-track.bash`:

```bash
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
    eval "declare -g${__tue_env_flags} ${__tue_env_rest}"
    return 0
}

function __tue_env_track_strip
{
    # $1: current value, $2: recorded added entries. Result in __TUE_ENV_VALUE: one occurrence of each
    # recorded entry removed. The occurrence closest to the recorded index is the one that goes, so
    # that an entry duplicating one the user already had does not silently reorder their value. An
    # entry that is no longer present is skipped.
    local -a __tue_env_c
    IFS=':' read -r -a __tue_env_c <<< "$1"
    local -A __tue_env_dead=()
    local __tue_env_rest="$2" __tue_env_pair __tue_env_idx __tue_env_e
    local __tue_env_best __tue_env_bestd __tue_env_p __tue_env_d

    while [[ -n "${__tue_env_rest}" ]]
    do
        __tue_env_pair="${__tue_env_rest%%"${__TUE_ENV_RS}"*}"
        __tue_env_rest="${__tue_env_rest#*"${__TUE_ENV_RS}"}"
        [[ -z "${__tue_env_pair}" ]] && continue
        __tue_env_idx="${__tue_env_pair%%"${__TUE_ENV_PS}"*}"
        __tue_env_e="${__tue_env_pair#*"${__TUE_ENV_PS}"}"

        __tue_env_best=""
        __tue_env_bestd=-1
        for (( __tue_env_p = 0; __tue_env_p < ${#__tue_env_c[@]}; __tue_env_p++ ))
        do
            [[ -n "${__tue_env_dead[${__tue_env_p}]:-}" ]] && continue
            [[ "${__tue_env_c[__tue_env_p]}" != "${__tue_env_e}" ]] && continue
            __tue_env_d=$(( __tue_env_p - __tue_env_idx ))
            if (( __tue_env_d < 0 ))
            then
                __tue_env_d=$(( 0 - __tue_env_d ))
            fi
            if (( __tue_env_bestd < 0 )) || (( __tue_env_d < __tue_env_bestd ))
            then
                __tue_env_bestd="${__tue_env_d}"
                __tue_env_best="${__tue_env_p}"
            fi
        done
        if [[ -n "${__tue_env_best}" ]]
        then
            __tue_env_dead["${__tue_env_best}"]=1
        fi
    done

    local __tue_env_o=""
    for (( __tue_env_p = 0; __tue_env_p < ${#__tue_env_c[@]}; __tue_env_p++ ))
    do
        [[ -n "${__tue_env_dead[${__tue_env_p}]:-}" ]] && continue
        __tue_env_o+="${__tue_env_o:+:}${__tue_env_c[__tue_env_p]}"
    done
    __TUE_ENV_VALUE="${__tue_env_o}"
    return 0
}
```

- [ ] **Step 4: Implement the variable revert**

Append to `setup/tue-env-track.bash`:

```bash
function __tue_env_track_revert_vars
{
    # Applies every variable entry in the ledger. Names are sorted so that the notes printed for kept
    # user changes come out in a stable order.
    local __tue_env_n __tue_env_kind __tue_env_pre __tue_env_post __tue_env_cur
    local -a __tue_env_names
    mapfile -t __tue_env_names < <(printf '%s\n' "${!__TUE_ENV_LEDGER_VAR[@]}" | LC_ALL=C sort)

    for __tue_env_n in "${__tue_env_names[@]}"
    do
        [[ -z "${__tue_env_n}" ]] && continue
        __tue_env_kind="${__TUE_ENV_LEDGER_VAR[${__tue_env_n}]}"
        __tue_env_pre="${__TUE_ENV_LEDGER_VAR_PRE[${__tue_env_n}]}"
        __tue_env_post="${__TUE_ENV_LEDGER_VAR_POST[${__tue_env_n}]}"
        __tue_env_cur="$(declare -p "${__tue_env_n}" 2> /dev/null)"

        if [[ "${__tue_env_kind}" == "extended" ]]
        then
            # Entry-wise removal needs no conflict check: whatever the user added stays by
            # construction.
            [[ -z "${__tue_env_cur}" ]] && continue
            __tue_env_track_value "${__tue_env_cur}"
            __tue_env_track_strip "${__TUE_ENV_VALUE}" "${__TUE_ENV_LEDGER_VAR_ADD[${__tue_env_n}]}"
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
```

`printf -v` assigns through to the global, because nothing in this file declares a local by that name;
that is also why the naming rule matters here.

- [ ] **Step 5: Run the tests to verify they pass**

Run: `./test/.bats/bin/bats --print-output-on-failure test/track_revert_vars.bats`
Expected: 12 tests, all pass

Run: `shellcheck -- setup/tue-env-track.bash`
Expected: no output

- [ ] **Step 6: Commit**

```bash
git add setup/tue-env-track.bash test/track_revert_vars.bats
git commit -m "Revert tracked variable changes from the ledger"
```

---

### Task 6: Reverting functions, aliases and completions

**Files:**

- Modify: `setup/tue-env-track.bash`
- Create: `test/track_revert_shell.bats`

**Interfaces:**

- Consumes: the `__TUE_ENV_LEDGER_{FUNC,ALIAS,COMPLETE}*` arrays, `__tue_env_track_kept`,
  `__tue_env_track_revert_vars`, `__tue_env_track_empty`.
- Produces: `__tue_env_track_current <FUNC|ALIAS|COMPLETE> <name>` → `__TUE_ENV_CURRENT`,
  `__tue_env_track_revert_funcs`, `__tue_env_track_revert_simple <ALIAS|COMPLETE>`,
  `__tue_env_track_clear`, and the `_tue-env-track-revert` entry point.

- [ ] **Step 1: Write the failing tests**

Create `test/track_revert_shell.bats`:

```bash
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
    ! declare -F tue_test_fn > /dev/null
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
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `./test/.bats/bin/bats --print-output-on-failure test/track_revert_shell.bats`
Expected: every test fails with `_tue-env-track-revert: command not found`

- [ ] **Step 3: Implement the current-state reader and the two reverts**

Append to `setup/tue-env-track.bash`:

```bash
function __tue_env_track_current
{
    # $1: FUNC, ALIAS or COMPLETE, $2: name. Result in __TUE_ENV_CURRENT, empty when absent.
    case "$1" in
        FUNC )
            __TUE_ENV_CURRENT="$(declare -f "$2" 2> /dev/null)" ;;
        ALIAS )
            __TUE_ENV_CURRENT="${BASH_ALIASES[$2]:-}" ;;
        COMPLETE )
            __TUE_ENV_CURRENT="$(complete -p "$2" 2> /dev/null)" ;;
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
                # `declare -f` output does not encode `export -f`, so it has to be re-applied.
                export -f "${__tue_env_n}"
            fi
        fi
    done

    return 0
}

function __tue_env_track_revert_simple
{
    # $1: ALIAS or COMPLETE.
    local -n __tue_env_lk="__TUE_ENV_LEDGER_$1"
    local -n __tue_env_lp="__TUE_ENV_LEDGER_$1_PRE"
    local -n __tue_env_lq="__TUE_ENV_LEDGER_$1_POST"
    local __tue_env_n __tue_env_pre __tue_env_label
    local -a __tue_env_names
    mapfile -t __tue_env_names < <(printf '%s\n' "${!__tue_env_lk[@]}" | LC_ALL=C sort)

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
        if [[ "${__TUE_ENV_CURRENT}" != "${__tue_env_lq[${__tue_env_n}]}" ]]
        then
            __tue_env_track_kept "version of ${__tue_env_label} ${__tue_env_n}"
            continue
        fi

        __tue_env_pre="${__tue_env_lp[${__tue_env_n}]}"
        if [[ "$1" == "ALIAS" ]]
        then
            unalias "${__tue_env_n}" 2> /dev/null || true
            if [[ -n "${__tue_env_pre}" ]]
            then
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
```

- [ ] **Step 4: Run the whole suite to verify it passes**

Run: `./test/.bats/bin/bats --print-output-on-failure test/*.bats`
Expected: all tests pass

Run: `shellcheck -- setup/tue-env-track.bash`
Expected: no output

- [ ] **Step 5: Commit**

```bash
git add setup/tue-env-track.bash test/track_revert_shell.bats
git commit -m "Revert tracked functions, aliases and completions from the ledger"
```

---

### Task 7: Reporting and dry runs

**Files:**

- Modify: `setup/tue-env-track.bash`
- Create: `test/track_report.bats`

**Interfaces:**

- Consumes: the whole ledger, `__tue_env_track_attrs`, `__tue_env_track_value`, `__tue_env_track_current`,
  `__tue_env_track_empty`.
- Produces: `__tue_env_track_display <declare line>` → `__TUE_ENV_VALUE`,
  `__tue_env_track_entry_list <added entries>` → `__TUE_ENV_LIST`, `__tue_env_track_report_vars <mode>`,
  `__tue_env_track_report_objects <mode> <FUNC|ALIAS|COMPLETE> <label>`, and the
  `_tue-env-track-report changes|revert` entry point.

`changes` answers "what did loading do", `revert` answers "what would unloading do right now". They diverge
exactly when the user changed something since the load.

- [ ] **Step 1: Write the failing tests**

Create `test/track_report.bats`:

```bash
load helpers/track

setup() {
    tue_track_setup
}

@test "report: changes lists added entries, added scalars and replaced values" {
    export TUE_TEST_LIST="/usr/bin"
    PS1='\u@\h \$ '
    _tue-env-track-begin
    export TUE_TEST_LIST="/usr/lib/ccache:/opt/ros/jazzy/bin:${TUE_TEST_LIST}"
    export TUE_TEST_RMW=rmw_cyclonedds_cpp
    PS1='\u@\h git \$ '
    alias tue_test_trunk='echo trunk'
    tue_test_fn() {
        echo fn
    }
    _tue-env-track-commit
    run _tue-env-track-report changes
    [[ "${status}" -eq 0 ]]
    [[ "${output}" == *"added   TUE_TEST_LIST entries: /usr/lib/ccache, /opt/ros/jazzy/bin"* ]]
    [[ "${output}" == *"added   TUE_TEST_RMW=rmw_cyclonedds_cpp"* ]]
    [[ "${output}" == *"changed PS1 (was '\\u@\\h \$ ')"* ]]
    [[ "${output}" == *"added   alias tue_test_trunk"* ]]
    [[ "${output}" == *"added   function tue_test_fn"* ]]
}

@test "report: revert names what would go, what would come back and what would stay" {
    export TUE_TEST_LIST="/usr/bin"
    PS1='\u@\h \$ '
    _tue-env-track-begin
    export TUE_TEST_LIST="/usr/lib/ccache:${TUE_TEST_LIST}"
    export TUE_TEST_RMW=rmw_cyclonedds_cpp
    PS1='\u@\h git \$ '
    alias tue_test_trunk='echo trunk'
    _tue-env-track-commit
    export TUE_TEST_RMW=rmw_connext
    run _tue-env-track-report revert
    [[ "${status}" -eq 0 ]]
    [[ "${output}" == *"would remove TUE_TEST_LIST entries: /usr/lib/ccache"* ]]
    [[ "${output}" == *"would keep   TUE_TEST_RMW=rmw_connext (changed since load)"* ]]
    [[ "${output}" == *"would restore PS1 to '\\u@\\h \$ '"* ]]
    [[ "${output}" == *"would unset  alias tue_test_trunk"* ]]
}

@test "report: an array is reported without trying to render its value" {
    _tue-env-track-begin
    TUE_TEST_ARR=(a b)
    _tue-env-track-commit
    run _tue-env-track-report changes
    [[ "${output}" == *"added   TUE_TEST_ARR=(array)"* ]]
}

@test "report: several objects of one kind are grouped on one line" {
    _tue-env-track-begin
    alias tue_test_a='echo a'
    alias tue_test_b='echo b'
    _tue-env-track-commit
    run _tue-env-track-report changes
    [[ "${output}" == *"added   alias tue_test_a, alias tue_test_b"* ]]
}

@test "report: an empty ledger returns 1 and prints nothing" {
    run _tue-env-track-report changes
    [[ "${status}" -eq 1 ]]
    [[ -z "${output}" ]]
}

@test "report: a dry run mutates nothing" {
    export TUE_TEST_LIST="/usr/bin"
    _tue-env-track-begin
    export TUE_TEST_LIST="/ccache:${TUE_TEST_LIST}"
    export TUE_TEST_NEW=1
    alias tue_test_a='echo a'
    _tue-env-track-commit
    _tue-env-track-report revert > /dev/null
    [[ "${TUE_TEST_LIST}" == "/ccache:/usr/bin" ]]
    [[ "${TUE_TEST_NEW}" == "1" ]]
    [[ "${BASH_ALIASES[tue_test_a]}" == "echo a" ]]
    [[ -n "${__TUE_ENV_LEDGER_VAR[TUE_TEST_NEW]:-}" ]]
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `./test/.bats/bin/bats --print-output-on-failure test/track_report.bats`
Expected: every test fails with `_tue-env-track-report: command not found`

- [ ] **Step 3: Implement the renderers**

Append to `setup/tue-env-track.bash`:

```bash
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
        __tue_env_pair="${__tue_env_rest%%"${__TUE_ENV_RS}"*}"
        __tue_env_rest="${__tue_env_rest#*"${__TUE_ENV_RS}"}"
        [[ -z "${__tue_env_pair}" ]] && continue
        __tue_env_o+="${__tue_env_o:+, }${__tue_env_pair#*"${__TUE_ENV_PS}"}"
    done
    __TUE_ENV_LIST="${__tue_env_o}"
    return 0
}

function __tue_env_track_report_vars
{
    # $1: changes or revert.
    local __tue_env_n __tue_env_k __tue_env_pre __tue_env_post __tue_env_cur
    local -a __tue_env_names
    mapfile -t __tue_env_names < <(printf '%s\n' "${!__TUE_ENV_LEDGER_VAR[@]}" | LC_ALL=C sort)

    for __tue_env_n in "${__tue_env_names[@]}"
    do
        [[ -z "${__tue_env_n}" ]] && continue
        __tue_env_k="${__TUE_ENV_LEDGER_VAR[${__tue_env_n}]}"
        __tue_env_pre="${__TUE_ENV_LEDGER_VAR_PRE[${__tue_env_n}]}"
        __tue_env_post="${__TUE_ENV_LEDGER_VAR_POST[${__tue_env_n}]}"

        if [[ "${__tue_env_k}" == "extended" ]]
        then
            __tue_env_track_entry_list "${__TUE_ENV_LEDGER_VAR_ADD[${__tue_env_n}]}"
            if [[ "$1" == "changes" ]]
            then
                echo "added   ${__tue_env_n} entries: ${__TUE_ENV_LIST}"
            else
                echo "would remove ${__tue_env_n} entries: ${__TUE_ENV_LIST}"
            fi
            continue
        fi

        __tue_env_cur="$(declare -p "${__tue_env_n}" 2> /dev/null)"
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
    local -n __tue_env_lk="__TUE_ENV_LEDGER_$2"
    local -n __tue_env_lq="__TUE_ENV_LEDGER_$2_POST"
    local __tue_env_n __tue_env_add="" __tue_env_gone="" __tue_env_chg="" __tue_env_keep=""
    local -a __tue_env_names
    mapfile -t __tue_env_names < <(printf '%s\n' "${!__tue_env_lk[@]}" | LC_ALL=C sort)

    for __tue_env_n in "${__tue_env_names[@]}"
    do
        [[ -z "${__tue_env_n}" ]] && continue
        __tue_env_track_current "$2" "${__tue_env_n}"
        if [[ "$1" == "revert" ]] && [[ "${__TUE_ENV_CURRENT}" != "${__tue_env_lq[${__tue_env_n}]}" ]]
        then
            __tue_env_keep+="${__tue_env_keep:+, }$3 ${__tue_env_n}"
            continue
        fi
        case "${__tue_env_lk[${__tue_env_n}]}" in
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
        [[ -n "${__tue_env_keep}" ]] && echo "would keep   ${__tue_env_keep} (changed since load)"
    fi

    return 0
}

function _tue-env-track-report
{
    # $1: changes or revert. Renders the ledger, or the revert it would perform, without mutating
    # anything. Returns 1 when the ledger is empty.
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
```

- [ ] **Step 4: Run the whole suite to verify it passes**

Run: `./test/.bats/bin/bats --print-output-on-failure test/*.bats`
Expected: all tests pass

Run: `shellcheck -- setup/tue-env-track.bash`
Expected: no output

- [ ] **Step 5: Commit**

```bash
git add setup/tue-env-track.bash test/track_report.bats
git commit -m "Report tracked changes and planned reverts"
```

---

### Task 8: Bracket the load in setup.bash

**Files:**

- Modify: `setup.bash`
- Create: `test/helpers/env.bash`, `test/setup_integration.bats`

**Interfaces:**

- Consumes: `_tue-env-track-begin`, `_tue-env-track-commit`, `_tue-env-track-revert`.
- Produces: `_tue-env-bootstrap` (everything outside the tracked span) and `_tue-env-load` (everything
  inside it), both unset again at the end of `setup.bash`; `tue_env_clean_shell`, `tue_env_fixture`,
  `tue_env_fixture_venv` and `tue_env_fixture_target` for the tests. Task 9 uses
  `tue_env_clean_shell` too.

The span begins immediately after `setup/tue-env.bash` is sourced and ends at the end of the load, so
`TUE_DIR`, `TUE_BIN`, their `PATH` entries, the `tue-env` function and `_tue-check-env-vars` survive an
unload, while everything `user_setup.bash`, the virtual environment, `tue-functions.bash`,
`tue-misc.bash` and `target_setup.bash` do is removed by it.

- [ ] **Step 1: Add the environment fixture helper**

Create `test/helpers/env.bash`:

```bash
# shellcheck shell=bash
#
# Builds a throw-away tue-env installation under BATS_TEST_TMPDIR so that the integration tests can
# source a real setup.bash without touching the user's own ~/.tue/user tree.
#
# Requires helpers/track to be loaded first, for TUE_TRACK_REPO_ROOT.

function tue_env_clean_shell
{
    # The shell running the tests usually has a tue-env environment of its own loaded, and exports it
    # into every child process: on the machine this was written on a child inherits TUE_ENV, TUE_DIR
    # and sixteen exported tue functions. Left in place they would land in the pre-load snapshot, so
    # `tue-make` would be classified `replaced` instead of `added`, and the inherited TUE_ENV would
    # decide which environment the fixture loads. Clear them, but keep the harness's own TUE_TRACK_*
    # and TUE_TEST_* variables.
    local __tue_env_n __tue_env_d1 __tue_env_d2

    for __tue_env_n in $(compgen -v TUE_) $(compgen -v ROS_) $(compgen -v VIRTUAL_ENV) \
                       $(compgen -v _OLD_VIRTUAL)
    do
        case "${__tue_env_n}" in
            TUE_TRACK_* | TUE_TEST_* )
                continue ;;
        esac
        unset "${__tue_env_n}"
    done
    unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH RMW_IMPLEMENTATION

    # Only exported functions: those are the ones that came in from the parent shell. The tracker's
    # own functions are not exported, so this cannot remove them.
    while read -r __tue_env_d1 __tue_env_d2 __tue_env_n
    do
        case "${__tue_env_n}" in
            tue-* | _tue-* | __tue-* | _git_* )
                unset -f "${__tue_env_n}" ;;
        esac
    done <<< "$(declare -Fx)"

    return 0
}

function tue_env_fixture
{
    # $1: environment name. Sets TUE_TEST_DIR (the TUE_DIR copy) and TUE_TEST_ENV_DIR.
    # The name goes into user/config/default_env rather than into TUE_ENV, so that setup.bash exports
    # TUE_ENV itself, inside the tracked span, exactly as it does for a real interactive shell.
    tue_env_clean_shell
    TUE_TEST_DIR="${BATS_TEST_TMPDIR}/tue"
    TUE_TEST_ENV_DIR="${BATS_TEST_TMPDIR}/env"
    mkdir -p "${TUE_TEST_DIR}/setup" "${TUE_TEST_DIR}/user/envs" "${TUE_TEST_DIR}/user/config" \
             "${TUE_TEST_ENV_DIR}/.env/targets" "${TUE_TEST_ENV_DIR}/.env/setup"
    cp "${TUE_TRACK_REPO_ROOT}/setup.bash" "${TUE_TEST_DIR}/"
    cp "${TUE_TRACK_REPO_ROOT}"/setup/*.bash "${TUE_TEST_DIR}/setup/"
    printf '%s\n' "${TUE_TEST_ENV_DIR}" > "${TUE_TEST_DIR}/user/envs/$1"
    printf '%s\n' "$1" > "${TUE_TEST_DIR}/user/config/default_env"
    printf 'export TUE_TEST_USER_SETUP=1\n' > "${TUE_TEST_ENV_DIR}/.env/setup/user_setup.bash"
    return 0
}

function tue_env_fixture_venv
{
    # A stand-in for the real virtualenv activate script: it makes the changes that matter to the
    # ledger, without the cost of creating a virtualenv.
    mkdir -p "${TUE_TEST_ENV_DIR}/.env/venv/bin"
    cat > "${TUE_TEST_ENV_DIR}/.env/venv/bin/activate" << 'ACTIVATE'
VIRTUAL_ENV="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
export VIRTUAL_ENV
_OLD_VIRTUAL_PATH="${PATH}"
PATH="${VIRTUAL_ENV}/bin:${PATH}"
export PATH
_OLD_VIRTUAL_PS1="${PS1:-}"
PS1="(venv) ${PS1:-}"
VIRTUAL_ENV_PROMPT="(venv) "
export VIRTUAL_ENV_PROMPT
deactivate() {
    PATH="${_OLD_VIRTUAL_PATH}"
    export PATH
    PS1="${_OLD_VIRTUAL_PS1}"
    unset _OLD_VIRTUAL_PATH _OLD_VIRTUAL_PS1 VIRTUAL_ENV VIRTUAL_ENV_PROMPT
    unset -f deactivate
}
ACTIVATE
    return 0
}

function tue_env_fixture_target
{
    # A stand-in for target_setup.bash and the target setup scripts it chains, including the git-ps1
    # target that clobbers the prompt the virtualenv saved. The PATH entry it adds is deliberately
    # unique: a developer's own PATH already holds /usr/lib/ccache from the real ccache target, which
    # would mask the assertion that the entry is gone after the revert.
    cat > "${TUE_TEST_ENV_DIR}/.env/setup/target_setup.bash" << 'TARGET'
export TUE_TEST_TARGET_VAR=1
export PATH="/opt/tue-test/bin${PATH:+:${PATH}}"
alias tue_test_target_alias='echo aliased'
tue_test_target_fn() {
    echo target
}
_OLD_VIRTUAL_PS1="\\u@\\h git \\\$ "
PS1="\\u@\\h git \\\$ "
TARGET
    return 0
}
```

- [ ] **Step 2: Write the failing integration tests**

Create `test/setup_integration.bats`:

```bash
load helpers/track
load helpers/env

setup() {
    tue_track_setup
}

@test "setup: a full load is tracked and unloading undoes exactly it" {
    tue_env_fixture testenv
    tue_env_fixture_target
    source "${TUE_TEST_DIR}/setup.bash" || true

    [[ "${TUE_ENV}" == "testenv" ]]
    [[ "${TUE_TEST_USER_SETUP}" == "1" ]]
    [[ ":${PATH}:" == *":/opt/tue-test/bin:"* ]]

    _tue-env-track-revert

    [[ -z "${TUE_ENV+set}" ]]
    [[ -z "${TUE_ENV_DIR+set}" ]]
    [[ -z "${TUE_ENV_TARGETS_DIR+set}" ]]
    [[ -z "${TUE_TEST_USER_SETUP+set}" ]]
    [[ -z "${TUE_TEST_TARGET_VAR+set}" ]]
    [[ -z "${BASH_ALIASES[tue_test_target_alias]:-}" ]]
    ! declare -F tue_test_target_fn > /dev/null
    ! declare -F tue-make > /dev/null
    [[ -z "$(complete -p tue-make 2> /dev/null)" ]]
    [[ ":${PATH}:" != *":/opt/tue-test/bin:"* ]]

    # Everything the bootstrap did is outside the tracked span, so it survives
    [[ "${TUE_DIR}" == "${TUE_TEST_DIR}" ]]
    [[ ":${PATH}:" == *":${TUE_TEST_DIR}/bin:"* ]]
    declare -F tue-env > /dev/null
    declare -F _tue-check-env-vars > /dev/null
}

@test "setup: unloading brings back the prompt the user started with" {
    tue_env_fixture testenv
    tue_env_fixture_venv
    tue_env_fixture_target
    PS1='\u@\h \$ '
    local __tue_env_want="${PS1}"

    source "${TUE_TEST_DIR}/setup.bash" || true
    [[ "${PS1}" != "${__tue_env_want}" ]]
    [[ -n "${VIRTUAL_ENV}" ]]
    declare -F deactivate > /dev/null

    _tue-env-track-revert

    # The venv saved the pristine prompt in _OLD_VIRTUAL_PS1 and the target then clobbered it, so the
    # venv's own deactivate would have restored the wrong one. The ledger keeps the pre-load state
    # from first sight, so the prompt comes back pristine.
    [[ "${PS1}" == "${__tue_env_want}" ]]
    ! declare -F deactivate > /dev/null
    [[ -z "${VIRTUAL_ENV+set}" ]]
    [[ -z "${_OLD_VIRTUAL_PS1+set}" ]]
    [[ -z "${_OLD_VIRTUAL_PATH+set}" ]]
}

@test "setup: a load that fails after exporting TUE_ENV is still tracked" {
    tue_env_fixture testenv
    rmdir "${TUE_TEST_ENV_DIR}/.env/targets"
    source "${TUE_TEST_DIR}/setup.bash" || true

    [[ "${__TUE_ENV_LEDGER_VAR[TUE_ENV]}" == "added" ]]
    _tue-env-track-revert
    [[ -z "${TUE_ENV+set}" ]]
}

@test "setup: re-sourcing setup.bash keeps one ledger and the original pre-load state" {
    tue_env_fixture testenv
    tue_env_fixture_target
    PS1='\u@\h \$ '
    local __tue_env_want="${PS1}"

    source "${TUE_TEST_DIR}/setup.bash" || true
    source "${TUE_TEST_DIR}/setup.bash" || true

    [[ "${__TUE_ENV_TRACK_DEPTH}" -eq 0 ]]
    _tue-env-track-revert
    [[ "${PS1}" == "${__tue_env_want}" ]]
    [[ -z "${TUE_ENV+set}" ]]
    # /opt/tue-test/bin was added by both loads, and both occurrences have to go
    [[ ":${PATH}:" != *":/opt/tue-test/bin:"* ]]
}

@test "setup: a change made by hand between two loads survives the unload" {
    tue_env_fixture testenv
    tue_env_fixture_target
    source "${TUE_TEST_DIR}/setup.bash" || true
    export PATH="/opt/mine/bin:${PATH}"
    export TUE_TEST_OF_MINE=1
    source "${TUE_TEST_DIR}/setup.bash" || true

    _tue-env-track-revert
    [[ ":${PATH}:" == *":/opt/mine/bin:"* ]]
    [[ "${TUE_TEST_OF_MINE}" == "1" ]]
}
```

- [ ] **Step 3: Run the tests to verify they fail**

Run: `./test/.bats/bin/bats --print-output-on-failure test/setup_integration.bats`
Expected: failures — `setup.bash` does not yet source the tracker, so `_tue-env-track-revert` is not
defined and nothing is tracked

- [ ] **Step 4: Split setup.bash and add the wrapper**

In `setup.bash`, keep `_tue-check-env-vars` as it is. Replace the single `_tue-env-main` by three
functions. `_tue-env-bootstrap` takes the current lines that set `TUE_DIR`, `TUE_BIN` and `PATH` and
source `setup/tue-env.bash`, and gains the tracker source:

```bash
function _tue-env-bootstrap
{
    # Everything here runs before the tracked span begins, so it survives `tue-env deactivate`.

    # <-- lines 16 to 38 of the current setup.bash, moved here verbatim: the TUE_DIR block, the
    #     TUE_BIN and PATH block, and `source "${TUE_DIR}"/setup/tue-env.bash` -->

    # -----------------------------------------
    # Load the change tracker, so that it exists before the tracked span begins
    # shellcheck disable=SC1091
    source "${TUE_DIR}"/setup/tue-env-track.bash
}
```

Move those lines rather than retyping them: `TUE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"`
still resolves to the directory of `setup.bash`, because `BASH_SOURCE[0]` names the file a function was
defined in, not the function.

`_tue-env-load` is lines 40 to 113 of the current `setup.bash` — the body of `_tue-env-main` from the
`# Load the (optional) default environment` comment down to and including the `target_setup.bash`
block — moved verbatim into a function of its own:

```bash
function _tue-env-load
{
    # Load the (optional) default environment
    if [[ -z "${TUE_ENV}" ]]
    then
        [[ -f "${TUE_DIR}"/user/config/default_env ]] || return 0  # Quietly return

        TUE_ENV=$(cat "${TUE_DIR}"/user/config/default_env)
        export TUE_ENV

        if [[ ! -f "${TUE_DIR}"/user/envs/"${TUE_ENV}" ]]
        then
            echo "[tue] No such environment: '${TUE_ENV}'"
            return 1
        fi
    fi

    # ... the rest of the current _tue-env-main body, verbatim, down to and including the
    #     target_setup.bash block ...
}
```

And `_tue-env-main` becomes the wrapper:

```bash
function _tue-env-main
{
    _tue-env-bootstrap

    # The wrapper is required, not cosmetic: _tue-env-load returns early on a missing environment, a
    # missing TUE_ENV_DIR and a missing targets directory, all after TUE_ENV has been exported.
    # Without committing on those paths a failed load would leave TUE_ENV* set and untracked.
    # Collecting the status through `||` also keeps a caller's `set -e` from skipping the commit, and
    # the local is prefixed so that the commit snapshot does not record it as an added variable.
    local __tue_env_ret=0
    _tue-env-track-begin
    _tue-env-load "$@" || __tue_env_ret=$?
    _tue-env-track-commit
    return "${__tue_env_ret}"
}
```

Replace the last two lines of the file by:

```bash
_tue-env-main "$@"

unset -f _tue-env-bootstrap _tue-env-load _tue-env-main
```

- [ ] **Step 5: Run the tests to verify they pass**

Run: `./test/.bats/bin/bats --print-output-on-failure test/setup_integration.bats`
Expected: 5 tests, all pass

Run: `./test/.bats/bin/bats --print-output-on-failure test/*.bats`
Expected: all tests pass

Run: `shellcheck -- setup.bash test/helpers/env.bash`
Expected: no output

- [ ] **Step 6: Verify it works in a real shell**

Run: `bash -i -c 'source ~/.tue/setup.bash; tue-env current; echo "ledger: ${#__TUE_ENV_LEDGER_VAR[@]} vars,
${#__TUE_ENV_LEDGER_FUNC[@]} functions"'`
Expected: the current environment name, and a ledger holding tens of variables and functions

- [ ] **Step 7: Commit**

```bash
git add setup.bash test/helpers/env.bash test/setup_integration.bats
git commit -m "Bracket the environment load with the change tracker"
```

---

### Task 9: Wire the ledger into tue-env

**Files:**

- Modify: `setup/tue-env.bash`
- Create: `test/tue_env_cmd.bats`

**Interfaces:**

- Consumes: `_tue-env-track-revert`, `_tue-env-track-report`, `_tue-env-track-begin`,
  `_tue-env-track-commit`.
- Produces: a ledger-first `_tue-env-deactivate-current-env`, `tue-env changes`,
  `tue-env deactivate --dry-run`, and the matching help and completion entries.

- [ ] **Step 1: Write the failing tests**

Create `test/tue_env_cmd.bats`:

```bash
load helpers/track
load helpers/env

setup() {
    tue_track_setup
    # Before sourcing tue-env.bash, not after: the inherited TUE_ENV would make the "no environment
    # is active" test fail, and cleaning afterwards would unset the `tue-env` this test needs.
    tue_env_clean_shell
    # shellcheck source=/dev/null
    source "${TUE_TRACK_REPO_ROOT}/setup/tue-env.bash"
    TUE_DIR="${BATS_TEST_TMPDIR}/tue"
    export TUE_DIR
}

@test "deactivate: a non-empty ledger is applied and the virtualenv deactivate is not called" {
    deactivate() {
        echo "DEACTIVATE RAN"
        return 1
    }
    export VIRTUAL_ENV="${BATS_TEST_TMPDIR}/venv"
    _tue-env-track-begin
    export TUE_TEST_NEW=1
    _tue-env-track-commit
    run _tue-env-deactivate-current-env
    [[ "${status}" -eq 0 ]]
    [[ "${output}" != *"DEACTIVATE RAN"* ]]
}

@test "deactivate: an empty ledger falls back to the old heuristic" {
    export TUE_ENV=fake
    export TUE_ENV_DIR="${BATS_TEST_TMPDIR}/fake"
    export ROS_DISTRO=jazzy
    export AMENT_PREFIX_PATH=/opt/ros/jazzy
    _tue-env-deactivate-current-env
    [[ -z "${TUE_ENV+set}" ]]
    [[ -z "${TUE_ENV_DIR+set}" ]]
    [[ -z "${ROS_DISTRO+set}" ]]
    [[ -z "${AMENT_PREFIX_PATH+set}" ]]
}

@test "changes: reports what the load added" {
    export TUE_ENV=fake
    _tue-env-track-begin
    export TUE_TEST_RMW=rmw_cyclonedds_cpp
    _tue-env-track-commit
    run tue-env changes
    [[ "${status}" -eq 0 ]]
    [[ "${output}" == *"added   TUE_TEST_RMW=rmw_cyclonedds_cpp"* ]]
}

@test "changes: says so when there is nothing tracked" {
    export TUE_ENV=fake
    run tue-env changes
    [[ "${status}" -eq 1 ]]
    [[ "${output}" == *"no tracked changes"* ]]
}

@test "changes: refuses when no environment is active" {
    run tue-env changes
    [[ "${status}" -eq 1 ]]
    [[ "${output}" == *"No environment is currently active"* ]]
}

@test "changes: --help is accepted" {
    run tue-env changes --help
    [[ "${status}" -eq 1 ]]
    [[ "${output}" == *"Usage: tue-env changes"* ]]
}

@test "deactivate --dry-run: reports and changes nothing" {
    export TUE_ENV=fake
    _tue-env-track-begin
    export TUE_TEST_RMW=rmw_cyclonedds_cpp
    _tue-env-track-commit
    export TUE_TEST_RMW=rmw_connext
    tue-env deactivate --dry-run > /dev/null
    [[ "${TUE_TEST_RMW}" == "rmw_connext" ]]
    [[ -n "${__TUE_ENV_LEDGER_VAR[TUE_TEST_RMW]:-}" ]]
    [[ "${TUE_ENV}" == "fake" ]]
}

@test "deactivate --dry-run: names the user's value as kept" {
    export TUE_ENV=fake
    _tue-env-track-begin
    export TUE_TEST_RMW=rmw_cyclonedds_cpp
    _tue-env-track-commit
    export TUE_TEST_RMW=rmw_connext
    run tue-env deactivate --dry-run
    [[ "${status}" -eq 0 ]]
    [[ "${output}" == *"would keep   TUE_TEST_RMW=rmw_connext (changed since load)"* ]]
}

@test "deactivate: an unknown option is still rejected" {
    export TUE_ENV=fake
    run tue-env deactivate --nonsense
    [[ "${status}" -eq 1 ]]
    [[ "${output}" == *"Unknown option --nonsense"* ]]
}

@test "switch: the new TUE_ENV is tracked, so a later deactivate unsets it" {
    tue_env_fixture envone
    printf '%s\n' "${TUE_TEST_ENV_DIR}" > "${TUE_TEST_DIR}/user/envs/envtwo"
    source "${TUE_TEST_DIR}/setup.bash" || true
    [[ "${TUE_ENV}" == "envone" ]]

    tue-env switch envtwo || true
    [[ "${TUE_ENV}" == "envtwo" ]]
    [[ -n "${__TUE_ENV_LEDGER_VAR[TUE_ENV]:-}" ]]

    _tue-env-track-revert
    [[ -z "${TUE_ENV+set}" ]]
    [[ -z "${TUE_ENV_DIR+set}" ]]
}

@test "completion: changes and --dry-run are offered" {
    COMP_WORDS=(tue-env "")
    COMP_CWORD=1
    _tue-env
    [[ "${COMPREPLY[*]}" == *"changes"* ]]
    COMP_WORDS=(tue-env deactivate "")
    COMP_CWORD=2
    _tue-env
    [[ "${COMPREPLY[*]}" == *"--dry-run"* ]]
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `./test/.bats/bin/bats --print-output-on-failure test/tue_env_cmd.bats`
Expected: the `changes`, `--dry-run`, completion and ledger-first tests fail; the fallback test passes,
because that is the behaviour that already exists

- [ ] **Step 3: Make the deactivate helper ledger-first**

In `setup/tue-env.bash`, insert at the top of `_tue-env-deactivate-current-env`, before the existing
virtualenv block:

```bash
    # The ledger is the sole authority when it holds anything: it recorded everything the load did,
    # including everything the virtual environment's `activate` did, so `deactivate` is not called any
    # more and its _OLD_* variables are unset with everything else.
    if declare -F _tue-env-track-revert > /dev/null && _tue-env-track-revert
    then
        return 0
    fi

    # Empty ledger: a shell started before change tracking existed, or a non-interactive child shell
    # that inherited `tue-env` but not the ledger. Behave exactly as this function did before.
```

- [ ] **Step 4: Add the `--dry-run` flag to `deactivate`**

In the `deactivate` branch of `tue-env`, declare the flag and accept it:

```bash
        local dry_run
        dry_run="false"
        for i in "$@"
        do
            case $i in
                --help | -h )
                    show_help="true"
                    break
                    ;;
                --dry-run )
                    dry_run="true" ;;
                --*)
                    echo "[tue-env](deactivate) Unknown option $i"
                    show_help="true"
                    ;;
                * )
                    echo "[tue-env](deactivate) Unknown input variable $i"
                    show_help="true"
                    ;;
            esac
        done
```

Extend the usage text of the branch:

```bash
            echo """Usage: tue-env deactivate [options]

    Possible options:
                --dry-run      - Show what unloading would do, without changing anything
                --help, -h     - Show this help message and exit
"""
```

And insert the dry run after the "No environment is currently active" check, before the deactivation:

```bash
        if [[ "${dry_run}" == "true" ]]
        then
            if ! declare -F _tue-env-track-report > /dev/null
            then
                echo "[tue-env](deactivate) change tracking is not available in this shell"
                return 1
            fi
            if ! _tue-env-track-report revert
            then
                echo "[tue-env](deactivate) no tracked changes; this shell was started before change tracking"
                return 1
            fi
            return 0
        fi
```

- [ ] **Step 5: Add the `changes` command**

In `tue-env`, add a branch next to `deactivate`:

```bash
    elif [[ ${cmd} == "changes" ]]
    then
        for i in "$@"
        do
            case $i in
                --help | -h )
                    show_help="true"
                    break
                    ;;
                --*)
                    echo "[tue-env](changes) Unknown option $i"
                    show_help="true"
                    ;;
                * )
                    echo "[tue-env](changes) Unknown input variable $i"
                    show_help="true"
                    ;;
            esac
        done

        if [[ ${show_help} == "true" ]]
        then
            # shellcheck disable=SC1078,SC1079
            echo """Usage: tue-env changes [options]

    Shows what loading the current environment changed in this shell.

    Possible options:
                --help, -h     - Show this help message and exit
"""
            return 1
        fi

        if [[ -z "${TUE_ENV}" ]]
        then
            echo "[tue-env](changes) No environment is currently active"
            return 1
        fi

        if ! declare -F _tue-env-track-report > /dev/null
        then
            echo "[tue-env](changes) change tracking is not available in this shell"
            return 1
        fi

        if ! _tue-env-track-report changes
        then
            echo "[tue-env](changes) no tracked changes; this shell was started before change tracking"
            return 1
        fi

        return 0
```

Add the command to the main help text, after the `switch` line:

```bash
        changes        - Shows what loading the current environment changed in this shell
```

- [ ] **Step 6: Track what `switch` sets before it sources setup.bash**

`tue-env switch` exports `TUE_ENV` and `TUE_ENV_DIR` itself and only then sources `setup.bash`, so those
two would be set before the span begins and would leak on the next deactivate. Bracketing the block with
the same pair fixes it: the depth counter makes `setup.bash`'s own inner pair a no-op, and the outer
commit sees both variables. No new local may be declared between the two calls, or the commit would
record it.

In the `switch` branch, replace the block that sets the variables and sources `setup.bash` with:

```bash
        # Successful, so we can set the environment variables. Begin the tracked span here rather than
        # in setup.bash, so that TUE_ENV and TUE_ENV_DIR are part of the diff; setup.bash's own
        # begin/commit pair nests inside this one and is a no-op.
        _tue-env-track-begin

        TUE_ENV=${tue_env}
        export TUE_ENV
        TUE_ENV_DIR=${tue_env_dir}
        export TUE_ENV_DIR

        echo "[tue-env](switch) Loading the new '${TUE_ENV}' environment"
        # shellcheck disable=SC1091
        source "$TUE_DIR"/setup.bash

        _tue-env-track-commit
```

- [ ] **Step 7: Add both to the completion function**

In `_tue-env`, the first-level word list is one long `compgen -W` line. Insert `'changes '\n` into it,
directly after `'current '\n`, leaving the rest of the line untouched.

And extend the `deactivate` branch of the same function to offer the flag, and give `changes` its own:

```bash
        elif [[ ${cmd} == "deactivate" ]]
        then
            if [[ "${COMP_CWORD}" -eq 2 ]]
            then
                mapfile -t COMPREPLY < <(compgen -W "$(echo -e "'--dry-run '\n${help_options}")" -- "${cur}")
            fi
        elif [[ ${cmd} == "changes" ]]
        then
            if [[ "${COMP_CWORD}" -eq 2 ]]
            then
                mapfile -t COMPREPLY < <(compgen -W "$(echo -e "${help_options}")" -- "${cur}")
            fi
```

- [ ] **Step 8: Run the tests to verify they pass**

Run: `./test/.bats/bin/bats --print-output-on-failure test/tue_env_cmd.bats`
Expected: 11 tests, all pass

Run: `./test/.bats/bin/bats --print-output-on-failure test/*.bats`
Expected: all tests pass

Run: `shellcheck -- setup/tue-env.bash`
Expected: no output

- [ ] **Step 9: Verify the real commands in a real shell**

Run: `bash -i -c 'source ~/.tue/setup.bash; tue-env changes | head -20; echo ---;
tue-env deactivate --dry-run | head -20'`
Expected: a `changes` listing of the entries the load produced, then a `would ...` listing; the shell is
unchanged by the dry run

- [ ] **Step 10: Commit**

```bash
git add setup/tue-env.bash test/tue_env_cmd.bats
git commit -m "Apply the ledger on deactivate and add tue-env changes and --dry-run"
```

---

### Task 10: Documentation and version

**Files:**

- Modify: `README.md`, `VERSION`

**Interfaces:**

- Consumes: the commands added in Task 9.
- Produces: nothing other tasks depend on.

- [ ] **Step 1: Document the commands and the revert contract**

In `README.md`, after the `## Different environments` section (which ends with the `tue-env set-default`
sentence) and before `## Guidelines on creating a new target`, add:

````markdown
### Unloading an environment

`tue-env deactivate` unloads the current environment from the running shell. Loading an environment is
recorded as it happens, so unloading removes what the load added and leaves anything you changed
afterwards alone: a `PATH` entry you added by hand stays, and a variable you set after loading keeps your
value, with one line printed to say it was kept.

Two commands show what is going on:

```bash
tue-env changes              # what loading the current environment did to this shell
tue-env deactivate --dry-run # what unloading would do right now
```

Readline bindings a target makes with `bind` are not tracked and survive an unload. A shell started
before this feature existed, and a non-interactive child shell, have no record to work from; there
`tue-env deactivate` falls back to unsetting `TUE_ENV*`, `ROS_*` and a few known variables, as it did
before.
````

- [ ] **Step 2: Check the documentation lints**

Run: `pre-commit run markdownlint-cli2 --files README.md docs/superpowers/plans/2026-08-24-env-change-tracking.md`
Expected: Passed

- [ ] **Step 3: Commit the documentation**

```bash
git add README.md
git commit -m "Document tue-env changes, deactivate --dry-run and the revert contract"
```

- [ ] **Step 4: Run everything once more**

Run: `./test/.bats/bin/bats --print-output-on-failure test/*.bats`
Expected: all tests pass

Run: `pre-commit run --all-files`
Expected: Passed for every hook (`no-commit-to-branch` passes because the branch is not `master`)

Run: `shopt -s globstar; shellcheck -- **/*.bash **/*.sh`
Expected: no output, matching what CI runs

- [ ] **Step 5: Bump the version**

The repository bumps `VERSION` in its own commit. This adds a feature, so the minor version moves:

```bash
echo "1.43.0" > VERSION
git add VERSION
git commit -m "Bump VERSION to 1.43.0"
```
