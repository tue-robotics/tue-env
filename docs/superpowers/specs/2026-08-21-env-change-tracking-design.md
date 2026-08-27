# Environment change tracking

Record what loading a `tue-env` environment does to the bash session, so unloading it can undo exactly that and
nothing else.

## Problem

`setup.bash` loads an environment by sourcing, in order, `user_setup.bash`, the python virtual environment,
`tue-functions.bash`, `tue-misc.bash` and `target_setup.bash`. The last of these chains the `setup` script of every
installed target, so the set of changes made to the shell is open-ended: it is whatever the installed targets happen
to do.

Unloading is `_tue-env-deactivate-current-env` in `setup/tue-env.bash`, a fixed heuristic. It calls the virtual
environment's `deactivate`, unsets every `TUE_ENV*` and `ROS_*` variable, and unsets `AMENT_PREFIX_PATH`,
`COLCON_PREFIX_PATH` and `PYTHONPATH`.

Everything outside that list leaks. Surveying the 204 target `setup` scripts installed across the ten environments on
the development machine this design was written against, the leaks include:

- `PATH` entries (`/usr/lib/ccache` from the `ccache` target, `/opt/ros/<distro>/bin` from `ros2`)
- `LD_LIBRARY_PATH`, `CMAKE_PREFIX_PATH`, `PKG_CONFIG_PATH`
- scalar variables: `RMW_IMPLEMENTATION`, `CYCLONEDDS_URI`, `RCUTILS_CONSOLE_OUTPUT_FORMAT`, `CURRENT_CMAKE_BUILD_DIR`
- non-exported variables: `PS1`, `GIT_PS1_*`
- aliases: `ros2`, `trunk`, `local-core`, `..`
- shell functions: `qtcreator`, `tsync`, `_ROS_MASTER_NAME`, `_ROS2_DISCOVERY`, and the ~74 defined by
  `tue-functions.bash`
- completion registrations: `tue-get`, `tue-make`, `tue-git-clean-local`, and the colcon/ros2 argcomplete hooks

There is a second, pre-existing defect in the same area. The virtual environment's `activate` saves the pristine `PS1`
into `_OLD_VIRTUAL_PS1`, but the `git-ps1` target's `setup` runs later and overwrites `_OLD_VIRTUAL_PS1` with its own
prompt variant. `tue-env deactivate` therefore restores a `git-ps1` prompt rather than the prompt the user started
with. Symmetrically, `_OLD_VIRTUAL_PATH` is a whole-value snapshot taken before `target_setup.bash` ran, so restoring
it discards both the environment's later `PATH` additions and any the user made by hand.

## Approach

Snapshot the shell state before and after the load, diff the two, and keep the diff in a per-shell **ledger**.
Unloading applies the ledger.

Rejected alternatives:

- **Compute the diff in python.** The repository already ships `setup/generate_setup_file.py`, so a
  `setup/env_diff.py` would fit and would be straightforward to unit test with pytest. It was rejected because it puts
  a python process on the critical path of every interactive shell start, and requires a serialisation layer plus
  temp-file lifecycle keyed to the shell PID.
- **Intercept `export`, `alias` and `declare` during the load.** Cheapest at load time, but it cannot see
  `VAR=x; export VAR`, a bare `PS1=...`, or `function f { ... }`. Every target `setup` script is exactly such a path.

## Tracked span

The span begins immediately after `setup/tue-env.bash` is sourced and ends at the end of `_tue-env-main`. Everything
inside is environment-scoped and is removed on unload:

| Survives unload | Removed on unload |
| --- | --- |
| `TUE_DIR`, `TUE_BIN` | `TUE_ENV`, `TUE_ENV_DIR`, `TUE_ENV_TARGETS_DIR`, `TUE_ENV_WS_DIR`, `TUE_ENV_RELEASE_DIR` |
| the `TUE_BIN` entry in `PATH` | everything `user_setup.bash` exports |
| the `tue-env` function | the virtual environment |
| `_tue-check-env-vars` | `tue-get`, `tue-make` and the rest of `tue-functions.bash` |
| | `LC_ALL` and the pip completion from `tue-misc.bash` |
| | everything `target_setup.bash` and its chained target scripts do |

`tue-env` itself is defined in `setup/tue-env.bash`, which is sourced before the span starts, so it stays available
after a deactivate and the user can switch back.

## Revert rules

Two rules, decided by how the value changed.

**Extended (list-shaped) variables.** A change counts as an extension when the pre-load value's `:`-separated entries
are a subsequence, in order, of the post-load value's entries. This is exactly the shape produced by
`export PATH=/usr/lib/ccache:${PATH}` and by `/opt/ros/<distro>/setup.bash`. The ledger records the added entries as a
multiset, each with the index it occupied in the post-load value. Unloading deletes one occurrence of each recorded
entry and leaves everything else alone.

Recording the index matters when the added entry duplicates one that was already present. With `PATH=A:X:B` and a
target that appends `X`, the post-load value is `A:X:B:X`; deleting the leftmost `X` would yield `A:B:X` and silently
reorder the user's path, while deleting the occurrence nearest the recorded index yields `A:X:B`. At unload the
occurrences that line up, in order, with the pre-load value are set aside as the user's own, and the one closest to
the recorded index is removed from what remains. Both halves are needed: without the alignment, a user who prepends
entries of their own shifts every position rightwards until "closest to the recorded index" is the user's copy and
the environment's outlives the unload.

```text
pre-load  : /usr/bin:/bin
after load: /usr/lib/ccache:/opt/ros/jazzy/bin:/usr/bin:/bin
user adds : /opt/foo/bin:/usr/lib/ccache:/opt/ros/jazzy/bin:/usr/bin:/bin
unload    : /opt/foo/bin:/usr/bin:/bin
```

The subsequence test needs no list of variable names and generalises to `LD_LIBRARY_PATH`, `PYTHONPATH`,
`AMENT_PREFIX_PATH`, `CMAKE_PREFIX_PATH` and `PKG_CONFIG_PATH`. An entry the ledger recorded that is no longer present
is skipped silently.

**Everything else.** Scalar variables, arrays, aliases, functions and completions are restored to their pre-load
state only if their current state still matches what the load produced. If the user changed them afterwards, their
version is kept and one line naming it is printed.

```text
load sets : RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
user sets : RMW_IMPLEMENTATION=rmw_connext
unload    : RMW_IMPLEMENTATION=rmw_connext (kept)
            [tue-env](deactivate) kept your value for RMW_IMPLEMENTATION
```

The unifying principle across both rules: **changes the user makes after loading always survive the unload.**

## The ledger

The ledger is the accumulated diff, one entry per name, held in shell-local associative arrays
(`__TUE_ENV_LEDGER_VAR`, `__TUE_ENV_LEDGER_ALIAS`, `__TUE_ENV_LEDGER_FUNC`, `__TUE_ENV_LEDGER_COMPLETE`). It is not
exported and never written to disk. Snapshots are transient; they exist only for the duration of a load.

An entry holds a kind (`added`, `removed`, `extended`, `replaced`), the pre-load state, and the post-load state. The
post-load state is what the conflict check compares the current shell against.

### Merging a second load

Loading an environment while it is already loaded — which `source ~/.bashrc` does, and which is common enough that
users alias it — takes a fresh snapshot pair and merges the resulting diff into the existing ledger:

- for a name already in the ledger: keep the **original** pre-load state, take the **new** post-load state
- for an extended variable already in the ledger: union the added entries as a multiset
- for a name not yet in the ledger: insert the entry as-is

Because a change the user made by hand between the two loads is present in both the new pre-load and the new
post-load snapshot, it cancels out of the diff and is never attributed to the environment. This is the reason the
ledger accumulates diffs rather than re-baselining or holding two state blobs.

Two consequences follow from the merge rules and are intended:

- a variable the environment adds on one load and unsets on the next collapses to a no-op and drops out of the ledger
- a variable the user set by hand after the first load, which the second load then overwrites, gets the **user's**
  value as its pre-load state, so unloading hands that value back rather than unsetting the variable — but only for
  a name the ledger did not already hold. Once the environment owns the name, the first merge rule wins and the
  original pre-load state is kept: a variable the environment added, the user then changed and a later load
  overwrote is **unset** by the unload, not given the user's intervening value back. The ledger is the accumulated
  diff of what the environment did, not an undo log of every state a name passed through, and that intervening
  value was destroyed by the second load rather than by the unload. Both halves are pinned by tests.

## Capture

A snapshot is a single command substitution around a dump function built entirely from builtins: `compgen -v`,
`declare -p`, `compgen -A function`, `declare -f`, `declare -Fx`, `BASH_ALIASES`, `complete -p`. The per-name work all
runs inside the one already-forked subshell, so an environment load costs **a constant number of forks, never one per
name**: the outer substitution, plus the handful the dump and the parse each make for a whole category at a time
(`declare -Fx`, `complete -p`, the `compgen` process substitutions, the `mapfile`s). That count does not grow with
the size of the environment, which is the property that matters; the exact number is an implementation detail and has
changed as the dump has.

Records are framed with `\x1e` and `\x1f` separators rather than newlines: `declare -p` emits literal newlines inside
values and spans multiple lines for arrays. NUL is unusable as a separator because command substitution discards NUL
bytes. Parsing back into associative arrays uses bash string operations only.

A payload can hold those bytes itself — an alias value, a completion registration or a `:`-separated entry is
arbitrary text — so each is escaped at capture, `\x1b` first and then each separator to `\x1b` plus a digit, and
unescaped again as the ledger is built. Both directions are pure parameter expansion, because escaping must not be
what turns a constant fork count into one per name. Variables are the exception and are left alone: `declare -p`
already renders every control byte as printable `$'\036'` text, for a scalar, for an array's elements and for an
associative array's keys, so a variable payload never carries a framing byte in the first place.

Function bodies are the one payload that cannot be escaped. `declare -f` writes into the snapshot's own command
substitution, and pulling its output into a variable first would cost a fork per function. A raw `\x1e` inside
somebody's function therefore still splits its record, so every record instead begins with a marker token: a piece
that does not begin with it is the rest of the body in the piece before it, and the parse puts the byte back and
rejoins. A bare one-letter kind would not be enough of a marker — a body holding `\x1e V \x1f` would read as a
variable record and write a pre-load state for a variable the load never touched.

A constant marker would only narrow that forgery surface, not close it: a body carrying the literal sequence
`\x1e TUEENVREC \x1f` would forge a record and truncate itself exactly as `\x1e V \x1f` did before the marker
existed. The marker is therefore not a constant. It is `TUEENVREC` plus a per-snapshot nonce, two `$RANDOM` draws,
and a body already defined in the shell cannot be carrying a number that had not been drawn when it was written.
`$RANDOM` is a parameter expansion, so the nonce costs no fork and the constant fork count is unaffected.

The nonce is drawn by the *caller*, immediately before each dump, and never inside the dump. Two consequences follow,
and both are load-bearing. A single dump stays a pure function of the shell, so two back-to-back snapshots of an
unchanged shell are still byte-identical. And the parse takes the marker from the *stream it is reading* — the first
field of its first record — rather than from the live global, because `setup.bash` is re-sourced from inside the
tracked span and that resets the global mid-load. A parse that read the global would match nothing in the pre-load
stream, classify the entire shell as `added`, and have `tue-env deactivate` unset variables the user owned before the
load. What the parse still checks against the global is the constant `TUEENVREC` base, which re-sourcing can only
assign the value it already had, so a stream that carries no marker at all parses to nothing rather than adopting its
own first field as one.

What the nonce buys is robustness against an unlucky payload — tue-env's own fixtures, a dotfile function that greps
snapshots, a here-doc holding a captured stream — and nothing more. It is **not** a security property. `$RANDOM` is
not cryptographically strong, and it does not need to be: code running inside the tracked span already has the user's
privileges and can assign to the ledger arrays directly, so it never has to forge a record to do harm. The tracker
has no integrity boundary against the code it is tracking and cannot have one. A shell with `RANDOM` unset degrades
the marker to the bare constant, which is fail-safe: the old behaviour, not a broken parse.

Variables are captured with `declare -p "${name}"`, not with the `${!ref@A}` parameter transformation. `@A` under
indirection silently mangles arrays — it indirects to the first element and then renders that:

```console
$ arr=(x y); ref=arr; printf '%s\n' "${!ref@A}"
declare -a arr='x'
```

`declare -p` carries the value and the attributes (`-x`, `-a`, `-A`, `-i`, `-r`), so arrays and export status survive
a round trip.

Four categories are captured:

| Category | Captured with | Notes |
| --- | --- | --- |
| Variables | `declare -p` per name from `compgen -v` | value plus attributes |
| Functions | `declare -f` bodies, `declare -Fx` for export status | `tue-functions.bash` exports ~20 |
| Aliases | the `BASH_ALIASES` array | names and values, without a fork |
| Completions | `complete -p` | tue-env's 5, plus colcon/ros2 argcomplete hooks |

### Exclusions

Bash's own volatile state must be excluded or every load reports spurious changes: `BASH*`, `PWD`, `OLDPWD`, `SHLVL`,
`_`, `RANDOM`, `SRANDOM`, `SECONDS`, `EPOCH*`, `LINENO`, `HISTCMD`, `FUNCNAME`, `GROUPS`, `DIRSTACK`, `PIPESTATUS`,
`COMP_*`, and the tracker's own `__TUE_ENV_LEDGER_*`.

### Classification

| Pre-load | Post-load | Ledger entry |
| --- | --- | --- |
| absent | present | `added`, with the post-load `declare -p` line |
| present | absent | `removed`, with the pre-load `declare -p` line |
| scalar, entries are a subsequence of the post-load ones | changed | `extended`, with the added entries as a multiset |
| anything else, including all arrays | changed | `replaced`, with both `declare -p` lines |

A variable going from absent to a single value has an empty pre-load entry list, which is trivially a subsequence.
Removing that one entry at unload leaves nothing, which is the same outcome as classifying it `added`, so the
degenerate case needs no special handling.

Functions, aliases and completions use the same `added`/`removed`/`replaced` triple. The list rule does not apply to
them.

## The ledger is the sole authority for reverting

Targets that save and restore their own old values are not consulted on unload. Their `_OLD_*` variables are ledger
entries like any other and are unset with everything else.

Concretely, the virtual environment's `deactivate` is no longer called. The ledger already recorded everything
`activate` did: `PATH` extended by the venv's `bin`, `PS1` replaced, `VIRTUAL_ENV`, `VIRTUAL_ENV_PROMPT` and
`_OLD_VIRTUAL_*` added, and `deactivate` added as a function. Reverting from the ledger unsets the `deactivate`
function along with the rest. `_tue-env-track-revert` ends with `hash -r`, the one non-variable thing the venv's
`deactivate` does.

This fixes the `PS1` defect described under **Problem** as a side effect. Because the ledger keeps the pre-load state
from first sight and the post-load state from last sight, the venv → `git-ps1` chain on `PS1` collapses to one entry
whose pre-load state is the user's pristine prompt.

The old heuristic is retained as a fallback for the empty-ledger case only, so shells started before this change, and
child shells, behave as they do today rather than worse. The fallback is today's `_tue-env-deactivate-current-env`
unchanged, including its call to the virtual environment's `deactivate`.

## Components

New file `setup/tue-env-track.bash`, sourced from `setup.bash` immediately after `setup/tue-env.bash` so the tracker
exists before the span begins. It exposes four entry points:

| Function | Responsibility |
| --- | --- |
| `_tue-env-track-begin` | take the transient pre-load snapshot, guarded by a depth counter |
| `_tue-env-track-commit` | take the post-load snapshot, diff, merge the diff into the ledger |
| `_tue-env-track-revert` | apply the ledger to the current shell, clear it, `hash -r` |
| `_tue-env-track-report` | render the ledger, or the planned revert, without mutating anything |

### `setup.bash`

`_tue-env-main`'s body moves into `_tue-env-load`, and `_tue-env-main` becomes a wrapper:

```bash
_tue-env-track-begin
_tue-env-load "$@"
local ret=$?
_tue-env-track-commit
return "${ret}"
```

The wrapper is required, not cosmetic: `_tue-env-main` returns early on a missing environment, a missing
`TUE_ENV_DIR` and a missing targets directory, all *after* `TUE_ENV` has been exported. Without it, a failed load
leaves `TUE_ENV*` set and untracked.

The depth counter makes the pair re-entrant. `_tue-env-track-begin` increments it and snapshots only when it moves
from zero; `_tue-env-track-commit` decrements it and snapshots, diffs and merges only when it returns to zero. A
target `setup` script that sources `setup.bash` recursively therefore contributes to the outer load's diff instead of
starting a second one. Re-sourcing `~/.bashrc` at top level is not nesting: the outer load has already committed, so
the counter is back at zero and the second load produces its own diff, which is merged as described under
**Merging a second load**.

### `setup/tue-env.bash`

- `_tue-env-deactivate-current-env` applies the ledger when it is non-empty, and falls back to the existing
  `TUE_ENV*`/`ROS_*` unset heuristic when it is empty
- new `tue-env changes` subcommand: what loading the current environment did
- new `tue-env deactivate --dry-run` flag: what unloading would do right now, including which of the user's later
  changes would be kept
- both added to the help texts and to the `_tue-env` completion function

`tue-env changes` and `tue-env deactivate --dry-run` answer different questions and diverge exactly when the user has
changed something since loading:

```console
$ tue-env changes
added   PATH entries: /usr/lib/ccache, /opt/ros/jazzy/bin
added   RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
changed PS1 (was '\u@\h \$ ')
added   alias trunk, alias ros2
added   function qtcreator, function tsync

$ export RMW_IMPLEMENTATION=rmw_connext
$ tue-env deactivate --dry-run
would remove PATH entries: /usr/lib/ccache, /opt/ros/jazzy/bin
would keep   RMW_IMPLEMENTATION=rmw_connext (changed since load)
would restore PS1 to '\u@\h \$ '
would unset  alias trunk, alias ros2
would unset  function qtcreator, function tsync
```

### Restoring: implementation traps

- `eval`ing a captured `declare -- name=...` line from inside `_tue-env-track-revert` creates a **function-local**
  variable and silently does nothing. Captured lines must be rewritten to `declare -g` before evaluation.
- `export -f` must be re-applied for restored functions that carried it; `declare -f` output does not encode it.
- An `extended` variable is put back entry-wise, and that assignment carries none of the pre-load attributes, so a
  load that both extends a variable and changes an attribute of it (`export FOO=/new:${FOO}` on a variable the user
  had set but never exported) would leave the attribute behind. The attribute letters are reconciled against the
  pre-load line instead, one letter at a time. Re-declaring the whole pre-load line is not an option: it would
  restore the pre-load value too and discard the entries the user added after the load. `r` is left alone — bash
  cannot remove it, and a readonly variable cannot have been extended by the load in the first place.
- Completions are removed with `complete -r <name>` and restored by evaluating the captured `complete -p` line.

## Boundaries

**Child shells.** Associative arrays cannot be exported, so a child `bash` inherits the environment variables but not
the ledger. `changes` and `deactivate --dry-run` fail there with their explanatory message. Plain `deactivate`
never reaches the empty-ledger fallback at all: `_tue-env-deactivate-current-env` is not exported either, so the
child errors out and tells the user to stop using the terminal — as it has always done, a pre-existing limitation
this feature does not change. Serialising the ledger into an exported variable was rejected: the function bodies
alone are roughly 60 KB, which every subsequent `fork`+`exec` would have to copy.

**Readline bindings.** Not tracked, so `bind` calls made by a target survive an unload. Documented, not engineered
around.

**A function's export status alone.** The diff sees the post-load `export -f` flag as well as the pre-load one, but
the ledger keeps only the pre-load one, so the conflict check that decides whether a function is still the
environment's compares bodies — and `declare -f` renders the same body whether or not the function is exported. A
user who changes nothing but the export status of a function the load added therefore leaves no trace the unload can
see, and the function is removed under them, against the rule that later user changes survive. Closing it needs a
fifth ledger array carrying the post-load flag through the merge, the revert, the report and the reset, and that is
not paid for a case this narrow: a body the user rewrote IS caught, which is the case that loses work. A
characterisation test pins the residual as a known limitation, so that closing it fails that test rather than
changing the behaviour silently.

Also out of scope: persistence of the ledger to disk, `shopt`/`set` flag tracking, a target-level `unsetup` hook, and
any change to how target `setup` scripts are written.

## Testing

The repository has no shell test framework today; CI runs actionlint, hadolint, black, shellcheck and docker builds.
This design adds bats-core and a CI job alongside the existing shellcheck job. The new file must also pass the
shellcheck pre-commit hook.

Cases that earn their place:

- entry-wise `PATH` removal preserving an entry the user added after load
- a duplicate entry removed exactly once
- scalar restore, and the conflict path keeping the user's value and printing the note
- added-then-unset, and removed-then-restored
- array values and the `export` attribute surviving a round trip
- an attribute the load changed on an extended variable reconciled without losing the entries the user added
- `export -f` re-applied to a restored function
- completions added and removed
- re-source accumulation: the ledger keeps the original pre-load state and folds in the new post-load state
- a change made by hand between two loads is not attributed to the environment
- virtual environment: `PS1` comes back pristine and the `deactivate` function is gone
- values containing newlines, colons, spaces and single quotes
- `--dry-run` mutates nothing
- empty-ledger fallback reproduces today's behaviour

## Documentation

`README.md` gains the two new commands and a short statement of the revert contract: unloading removes what loading
added, and anything changed after loading is left alone.
