# G1 Task 11 Portable Verification Closure Design

**Status:** `DESIGN_REVISION_CHECKPOINT_PENDING_HUMAN_REVIEW`

**Decision:** `FIXED_NODEIDS_DUAL_DIGEST_WITH_P_BOUND_EXTERNAL_ATTESTATION`

**Implementation commit (`E_impl`):**
`aa47af3946f2f9f934147b4b263affe345a9d450`

**First design checkpoint (`D1`):**
`d561f3be49b3ba059286818e325adc81b5b0b269`

**Current design revision:** `D2 = this revision`

**Runtime decision:** `ATTEMPT_04_PROHIBITED`

This documentation-only revision closes the remaining Task 11 design gaps. It
fixes the collected node-ID set across the later verification implementation,
defines a commit-bound external-verification attestation, and makes G0 consume
that attestation without reading retained attempt-02 evidence. This checkpoint
does not implement the verification runner, edit tests, add the tracked
external-evidence manifest, run Task 11 verification, create G0 evidence,
change a task checkbox, or authorize Isaac Sim, fresh C2a, Task 12, attempt-04,
or a PressButton episode.

## 1. Bound state and truth boundary

`D1` is the clean commit
`d561f3be49b3ba059286818e325adc81b5b0b269`; its parent is
`E_impl=aa47af3946f2f9f934147b4b263affe345a9d450`. `D2` is this
documentation-only child revision. At the design boundary, T150 is `[x]`;
T151, T152, and T070 are `[ ]`; attempt-04 remains
`ATTEMPT_04_PROHIBITED`.

The retained historical attempt-02 evidence remains ignored and external to
Git. Its `checksums.sha256` file has SHA-256:

```text
cc53c4b4bc3cefdc7a2363c6446741e3abfc65e768ac0db71123aa593be528ed
```

Task 11 closes repository verification and status projection only. Neither the
external-verification attestation nor G0 turns historical C2a evidence into
current evidence, creates runtime evidence, or establishes C1, C2, G1,
controlled arrival, direct reset, repeatability, or a physical safety claim.

## 2. Deterministic closure constraints

### 2.1 Collection-order and sorted digests are independent

For the approved 966 current-GREEN nodes, the sorted-byte-stream SHA-256 is:

```text
00a6e84c5d2e1f623f2211db8272ca95859e8050417f7c25cbfeef9afd84efc7
```

The SHA-256 of the same node IDs in pytest collection order is:

```text
1c8e6a8e9b09da6b06435ea6c75191c5fb4b3c3fa7e1b97161951e65249d45ad
```

Both are authoritative for different ordered byte streams. Sorting before the
collection-order digest is computed is a hard stop.

### 2.2 The portable archive cannot execute the external-input node

The exact node is:

```text
tests/test_g1_pose_conditioned_tracking_cli.py::test_t152_attempt02_is_historical_and_emits_exact_refresh_blocker_before_close
```

It reads the ignored directory:

```text
outputs/evidence/G1/c2a-static-preliminary-0ace57ce7169-attempt-02
```

A tracked-only archive cannot contain that directory. The node must pass in
the main checkout and the archive must explicitly deselect it. Attempt-02 must
not be copied, linked, mounted, rewritten, rehashed, or modified for the
archive or attestation.

### 2.3 Later V preserves every collected node ID

Before any `V` edit, clean `D2` produces a pre-V snapshot from one complete
`pytest --collect-only -q` collection. After `V` is committed, the same
collection process produces the post-V snapshot. The following files are
compared byte-for-byte with `cmp`:

```text
all-nodeids.collection.txt
all-nodeids.sorted.txt
current-green.collection.txt
current-green.sorted.txt
```

The full collection remains exactly 1091 and current GREEN remains exactly
966. Both approved current-GREEN digests remain unchanged. Any byte, order,
count, spelling, expansion, or classification drift stops `V`; no waiver is
permitted.

In `tests/test_clean_checkout_cli.py`, `V` may modify only bodies,
non-parametrizing fixtures/helpers, and assertions of existing test nodes. `V`
may not add, delete, or rename a test function; add, remove, or rename a
parameterized expansion; change parameter IDs or values; or otherwise alter
collection. The existing 12 node IDs in that file are fixed:

```text
tests/test_clean_checkout_cli.py::test_clean_checkout_green_command_deselects_only_manifest_nodes
tests/test_clean_checkout_cli.py::test_clean_checkout_parses_future_red_junit_without_calling_failures_passes
tests/test_clean_checkout_cli.py::test_clean_checkout_plan_has_required_isolated_steps
tests/test_clean_checkout_cli.py::test_clean_checkout_rejects_duplicate_manifest_node
tests/test_clean_checkout_cli.py::test_clean_checkout_rejects_invalid_future_red_junit_outcomes[0-changes0-return code]
tests/test_clean_checkout_cli.py::test_clean_checkout_rejects_invalid_future_red_junit_outcomes[1-changes1-unexpected passes]
tests/test_clean_checkout_cli.py::test_clean_checkout_rejects_invalid_future_red_junit_outcomes[1-changes2-errors]
tests/test_clean_checkout_cli.py::test_clean_checkout_rejects_invalid_future_red_junit_outcomes[1-changes3-skipped]
tests/test_clean_checkout_cli.py::test_clean_checkout_rejects_missing_manifest_node
tests/test_clean_checkout_cli.py::test_clean_checkout_rejects_uncollected_future_node
tests/test_clean_checkout_cli.py::test_clean_checkout_report_records_future_red_count_and_digest
tests/test_clean_checkout_cli.py::test_future_red_manifest_has_exact_unique_nodeids
```

## 3. Dual-list and dual-digest contract

Task 11 creates the four node-ID lists at clean `D2` before `V`, clean `V`,
and clean `P`.

`all-nodeids.collection.txt`:

- is extracted from one complete `pytest --collect-only -q` invocation;
- preserves pytest collection order byte-for-byte;
- contains exactly 1091 non-empty node IDs; and
- contains no duplicate node ID.

`all-nodeids.sorted.txt` is `sort -u all-nodeids.collection.txt`, also contains
exactly 1091 node IDs, and is used only for set proofs.

`current-green.collection.txt` removes the exact 125 intentional future-RED
node IDs while preserving collection order and contains exactly 966 nodes.
`current-green.sorted.txt` is its sorted unique copy and also contains 966
nodes.

The digest input is the complete UTF-8 list file with one trailing newline per
node ID:

```text
collection-order count = 966
collection-order SHA-256 =
1c8e6a8e9b09da6b06435ea6c75191c5fb4b3c3fa7e1b97161951e65249d45ad

sorted count = 966
sorted SHA-256 =
00a6e84c5d2e1f623f2211db8272ca95859e8050417f7c25cbfeef9afd84efc7
```

Pre-V versus post-V comparison is byte-for-byte for all four lists. Clean `V`
pre-projection versus clean `P` final-projection comparison is also
byte-for-byte for all four lists and additionally compares counts, both
digests, and normalized JUnit totals. At `V` and `P`, main-checkout
current-GREEN totals are exactly `tests=966`, `failures=0`, `errors=0`, and
`skipped=0`.

## 4. External-evidence manifest and classification

The later `V` commit adds the tracked manifest:

```text
configs/repository/external-evidence-nodeids.txt
```

Its contract is exact:

- one non-comment, non-empty line;
- sorted and unique;
- the sole line is the exact node in Section 2.2;
- no unknown or additional node is accepted;
- the node is present in the complete collection;
- the node is absent from the intentional future-RED manifest; and
- a lowercase SHA-256 is computed from the tracked bytes.

The node remains one of the 966 main-checkout current-GREEN nodes. The complete
partition is:

```text
1091 total
= 966 current-GREEN + 125 intentional future-RED
= 965 portable current-GREEN + 1 external-evidence current-GREEN
  + 125 intentional future-RED
```

## 5. Commit-bound external-verification attestation

### 5.1 Required directory

Main-checkout final-projection verification at clean `P` creates:

```text
external-verification/
  verification-commit.txt
  external-evidence-nodeids.txt
  external-evidence-manifest.sha256
  external-evidence.xml
  external-evidence-junit-totals.json
  attempt02-checksum-before.txt
  attempt02-checksum-after.txt
  blocker.json
  checksums.sha256
```

Every payload file is written and closed before `checksums.sha256`, which is
written last and covers the other eight files by basename. The directory is
bound to `P`: `verification-commit.txt` contains exactly the 40-character
lowercase SHA for `P` plus a newline. The copied node-ID bytes and recorded
manifest SHA bind to the tracked external-evidence manifest at `P`.

`external-evidence.xml` comes from the focused execution of the exact external
node in the main checkout. Its normalized totals file is exactly:

```json
{"errors":0,"failures":0,"skipped":0,"tests":1}
```

The focused test uses a temporary base outside `external-verification/`. Its
generated stale-blocker report supplies the systemic failure message and code
for normalized `blocker.json`. A passing exact node is the assertion authority
for zero factory calls. `blocker.json` contains at least:

```json
{
  "verification_commit": "<P>",
  "node_id": "tests/test_g1_pose_conditioned_tracking_cli.py::test_t152_attempt02_is_historical_and_emits_exact_refresh_blocker_before_close",
  "systemic_failure_code": "CURRENT_C2A_REFRESH_REQUIRED_AFTER_GEOMETRY_SCHEMA_CHANGE",
  "systemic_failure_message": "<non-empty message from the generated blocker report>",
  "factory_call_count": 0,
  "attempt02_checksum_before": "cc53c4b4bc3cefdc7a2363c6446741e3abfc65e768ac0db71123aa593be528ed",
  "attempt02_checksum_after": "cc53c4b4bc3cefdc7a2363c6446741e3abfc65e768ac0db71123aa593be528ed"
}
```

Both attempt checksum files contain exactly:

```text
cc53c4b4bc3cefdc7a2363c6446741e3abfc65e768ac0db71123aa593be528ed
```

The before value is captured and validated before any Task 11 verification
command capable of executing the external node, including the full T152 run,
the main-checkout current-GREEN run, and the focused external run. The after
value is captured and validated after all such executions. The two files must
be byte-for-byte equal. Both checksum text files are computed and validated
before `blocker.json` is normalized; the normalizer reads those values and
stores them directly as `attempt02_checksum_before` and
`attempt02_checksum_after`. Attestation generation must not modify or copy any
attempt-02 file or directory.

`V` pre-projection verification may create an isomorphic `V`-bound directory
for comparison. Its commit-bearing fields and derived checksums necessarily
differ from the `P` version, but structure, exact node, tracked-manifest SHA,
JUnit totals, exact blocker code, non-empty message, factory-zero result, and
approved attempt checksum values must agree. Final G0 consumes only the
`P`-bound directory.

This attestation is repository verification metadata. It is not portable test
output, runtime evidence, physical evidence, fresh C2a, C1, C2, or G1 evidence.

## 6. Portable archive and G0 consumption contract

### 6.1 Pure archive execution

The archive is created only with:

```bash
git archive "$VERIFY_COMMIT" | tar -x -C "$CLEAN_DIR"
```

No attempt-02 material enters the archive. Portable archive tests do not read
the original worktree:

```yaml
portable_archive_reads_original_worktree: false
```

Inside the archive, complete collection remains 1091; the exact 125
intentional future-RED nodes and exact one external node are explicitly
deselected; exactly 965 portable current-GREEN nodes pass; and JUnit totals are
`tests=965`, `failures=0`, `errors=0`, and `skipped=0`.

Let `P_green`, `E_external`, and `F_future` denote the sorted portable GREEN,
external-evidence GREEN, and intentional future-RED sets. Task 11 proves:

```text
P_green intersection E_external = empty
P_green intersection F_future = empty
E_external intersection F_future = empty
P_green union E_external union F_future = full collection
|P_green| + |E_external| + |F_future| = 965 + 1 + 125 = 1091
```

### 6.2 Required G0 input and validation order

The future `scripts/check_clean_checkout.py` requires:

```text
--external-verification <directory>
```

Before archive creation, G0 fails closed unless it validates all of the
following:

1. the directory exists, has the exact required regular files, and contains no
   symlink substitution;
2. `checksums.sha256` covers and verifies all eight payload files;
3. `verification-commit.txt` equals current clean `HEAD`, which is `P`;
4. the attested node-ID file is byte-for-byte equal to the tracked manifest
   and its attested SHA equals the SHA of that tracked file;
5. the node list contains only the exact external node;
6. JUnit totals are exactly 1 test, 0 failures, 0 errors, and 0 skipped;
7. `blocker.json` names current `P` under `verification_commit`, names the exact
   node under `node_id`, records the exact systemic failure code, a non-empty
   systemic failure message, and `factory_call_count=0`;
8. the attempt checksum files are equal to each other and to the approved
   `cc53c4b4bc3cefdc7a2363c6446741e3abfc65e768ac0db71123aa593be528ed`;
   and
9. `blocker.json` fields `attempt02_checksum_before` and
   `attempt02_checksum_after` equal their respective checksum text files and
   the same approved SHA.

G0 does not rerun the external node and does not read attempt-02. After input
validation, it creates and tests the portable archive. Its evidence may copy
the attestation payload/checksums or summarize them with their digests, but
must label them as external-verification attestation, not portable test output.

The G0 report and manifest record at least:

```yaml
total_collected: 1091
current_green_total: 966
portable_green_selected_count: 965
portable_green_passed_count: 965
external_evidence_count: 1
external_evidence_nodeids:
  - tests/test_g1_pose_conditioned_tracking_cli.py::test_t152_attempt02_is_historical_and_emits_exact_refresh_blocker_before_close
external_evidence_manifest_sha256: <lowercase tracked-file sha256>
intentional_future_red_count: 125
portable_archive_reads_original_worktree: false
external_verification_attestation_consumed: true
external_verification_commit: <P>
external_verification_junit:
  tests: 1
  failures: 0
  errors: 0
  skipped: 0
```

It also retains both current-GREEN list counts/digests and normalized portable
JUnit totals. `PASS_BENCHMARK` means repository-integrity verification only.

The final invocation is exactly:

```bash
python scripts/check_clean_checkout.py \
  --output "$G0_OUTPUT" \
  --external-verification \
    /tmp/g1-t152-final-projection/external-verification
```

## 7. Verification-infrastructure commit V

After human approval and creation of `D2`, `V` may change only the verification
infrastructure required by this contract:

- create `configs/repository/external-evidence-nodeids.txt`;
- modify `scripts/check_clean_checkout.py`;
- modify only allowed bodies, non-parametrizing fixtures/helpers, and
  assertions in `tests/test_clean_checkout_cli.py`; and
- update the Task 11 verification helper/plan without changing Tasks 1-10
  history.

No new test node is permitted. The ten D1 RED contracts map to exact existing
nodes as follows; reuse means one existing node proves more than one related
contract without changing collection:

| D1 RED contract | Existing node ID or IDs used by V |
|---|---|
| 1. Exact external-manifest count, ordering, uniqueness, and spelling | `tests/test_clean_checkout_cli.py::test_future_red_manifest_has_exact_unique_nodeids` |
| 2. Collection membership of the external node | `tests/test_clean_checkout_cli.py::test_clean_checkout_rejects_uncollected_future_node` |
| 3. Disjoint external and future-RED sets | `tests/test_clean_checkout_cli.py::test_clean_checkout_rejects_duplicate_manifest_node` |
| 4. Portable deselection of exactly 125 future-RED plus one external node | `tests/test_clean_checkout_cli.py::test_clean_checkout_green_command_deselects_only_manifest_nodes` |
| 5. Portable selection and PASS count of exactly 965 | `tests/test_clean_checkout_cli.py::test_clean_checkout_parses_future_red_junit_without_calling_failures_passes`<br>`tests/test_clean_checkout_cli.py::test_clean_checkout_rejects_invalid_future_red_junit_outcomes[0-changes0-return code]`<br>`tests/test_clean_checkout_cli.py::test_clean_checkout_rejects_invalid_future_red_junit_outcomes[1-changes1-unexpected passes]`<br>`tests/test_clean_checkout_cli.py::test_clean_checkout_rejects_invalid_future_red_junit_outcomes[1-changes2-errors]`<br>`tests/test_clean_checkout_cli.py::test_clean_checkout_rejects_invalid_future_red_junit_outcomes[1-changes3-skipped]` |
| 6. Complete classification of exactly 1091 nodes | `tests/test_clean_checkout_cli.py::test_clean_checkout_rejects_missing_manifest_node` |
| 7. Required G0 report/manifest fields and counts | `tests/test_clean_checkout_cli.py::test_clean_checkout_report_records_future_red_count_and_digest` |
| 8. Portable archive isolation and prohibition on external-evidence projection into the archive | `tests/test_clean_checkout_cli.py::test_clean_checkout_plan_has_required_isolated_steps` |
| 9. Generation and comparison of both current-GREEN list views and digests | `tests/test_clean_checkout_cli.py::test_clean_checkout_report_records_future_red_count_and_digest`; `tests/test_clean_checkout_cli.py::test_future_red_manifest_has_exact_unique_nodeids` |
| 10. P-bound attestation validation, exact stale blocker/factory-zero contract, and attempt checksum preservation | `tests/test_clean_checkout_cli.py::test_clean_checkout_plan_has_required_isolated_steps`<br>`tests/test_clean_checkout_cli.py::test_clean_checkout_report_records_future_red_count_and_digest`<br>`tests/test_clean_checkout_cli.py::test_clean_checkout_rejects_invalid_future_red_junit_outcomes[0-changes0-return code]`<br>`tests/test_clean_checkout_cli.py::test_clean_checkout_rejects_invalid_future_red_junit_outcomes[1-changes1-unexpected passes]`<br>`tests/test_clean_checkout_cli.py::test_clean_checkout_rejects_invalid_future_red_junit_outcomes[1-changes2-errors]`<br>`tests/test_clean_checkout_cli.py::test_clean_checkout_rejects_invalid_future_red_junit_outcomes[1-changes3-skipped]` |

`V` uses RED-to-GREEN development without Isaac Sim. It must prove the new
assertions RED before implementation and GREEN afterward while preserving all
four pre-V/post-V node-ID files byte-for-byte. `D2` adds no tests or
implementation.

## 8. Commit topology and projection rule

The required topology is:

```text
E_impl = aa47af3946f2f9f934147b4b263affe345a9d450
-> D1 = d561f3be49b3ba059286818e325adc81b5b0b269
-> D2 = this revision
-> V = verification infrastructure implementation
-> P = final T152 projection/status commit
-> FINAL_E2 = P
```

The parent invariants are:

```text
D1^ = E_impl
D2^ = D1
V^ = D2
P^ = V
```

The pre-V node snapshot runs at clean `D2`. The complete pre-projection suite
runs only after `V` is committed, its post-V lists equal the clean `D2`
snapshot byte-for-byte, and the worktree is clean. If it passes, `P` may modify
only:

- `specs/001-benchmark-reconstruction/tasks.md`, changing T152 `[ ]` to `[x]`;
  and
- `specs/001-benchmark-reconstruction/g1-contact-exclusion-t152-implementation-plan.md`,
  recording literal, already-known SHAs for `E_impl`, `D1`, `D2`, and `V`.

Tracked files must never contain `P`'s own SHA. After creating `P`, bind
`FINAL_E2=$(git rev-parse HEAD)`, require `P^=V`, rerun the identical suite,
compare clean `V` and clean `P`, generate the `P`-bound attestation, and invoke
G0 with that directory. Any later tracked change invalidates `FINAL_E2`.

## 9. Stop conditions and next gate

Stop without creating `V`, `P`, or G0 as applicable if any of these occurs:

- a test function or parameterized expansion is added, deleted, renamed, or
  otherwise changes the 12-node `test_clean_checkout_cli.py` inventory;
- any pre-V/post-V node-ID file differs byte-for-byte;
- collection differs from 1091, current GREEN differs from 966, or either
  approved digest changes;
- the external node is missing, misspelled, duplicated, future-RED, skipped, or
  does not pass in the main checkout;
- the external-verification directory is not bound to current `P`, has a
  checksum/coverage/file/type error, or differs from the exact
  node/JUnit/blocker/factory/attempt checksum contract;
- attempt-02 is modified, copied, projected into the archive, or read by G0;
- portable archive tests read the original worktree;
- the portable archive executes anything other than 965 GREEN nodes;
- any partition overlap, missing node, extra node, duplicate, or unclassified
  node exists;
- any JUnit failure, error, or skip appears in a GREEN run;
- `D1^=E_impl`, `D2^=D1`, `V^=D2`, or `P^=V` is false;
- `P` changes anything beyond the two authorized Markdown files or records its
  own SHA;
- T151 or T070 advances; or
- Isaac Sim, Task 12, fresh C2a, attempt-04, C2b, C3, or an episode is
  requested without separate approval.

The next permitted stage is review and commit of this `D2` documentation
revision, followed by separately authorized `V` RED-to-GREEN implementation.
T152 remains `[ ]` and attempt-04 remains prohibited until projection
completes.

## 10. Design conclusion

```text
VERIFICATION_LIST_AUTHORITY = COLLECTION_ORDER_AND_SORTED_VIEWS
FULL_COLLECTION_COUNT = 1091
CURRENT_GREEN_COLLECTION_COUNT = 966
CURRENT_GREEN_COLLECTION_SHA256 = 1c8e6a8e9b09da6b06435ea6c75191c5fb4b3c3fa7e1b97161951e65249d45ad
CURRENT_GREEN_SORTED_COUNT = 966
CURRENT_GREEN_SORTED_SHA256 = 00a6e84c5d2e1f623f2211db8272ca95859e8050417f7c25cbfeef9afd84efc7
MAIN_CHECKOUT_CURRENT_GREEN = 966
PORTABLE_ARCHIVE_CURRENT_GREEN = 965
EXTERNAL_EVIDENCE_CURRENT_GREEN = 1
INTENTIONAL_FUTURE_RED = 125
PORTABLE_ARCHIVE_READS_ORIGINAL_WORKTREE = false
EXTERNAL_VERIFICATION_ATTESTATION_CONSUMED = true
COMMIT_TOPOLOGY = E_impl -> D1 -> D2 -> V -> P
FINAL_E2 = P
T152 = OPEN_PENDING_V_AND_PROJECTION
ATTEMPT_04 = PROHIBITED
```
