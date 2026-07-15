# G1 Task 11 Portable Verification Closure Design

**Status:** `DESIGN_CHECKPOINT_PENDING_HUMAN_REVIEW`

**Decision:** `DUAL_DIGEST_WITH_EXPLICIT_EXTERNAL_EVIDENCE_CLASSIFICATION`

**Implementation commit:**
`aa47af3946f2f9f934147b4b263affe345a9d450`

**Runtime decision:** `ATTEMPT_04_PROHIBITED`

This document resolves two deterministic contradictions in Task 11 verification.
It defines the later verification-infrastructure commit `V` and the final
projection commit `P`. This checkpoint does not implement the verification
runner, add the external-evidence manifest, run Task 11 verification, create G0
evidence, change a task checkbox, or authorize Isaac Sim, fresh C2a, or
attempt-04.

## 1. Bound state and truth boundary

The design checkpoint starts from clean implementation commit
`aa47af3946f2f9f934147b4b263affe345a9d450`. At that commit, local HEAD, the
branch ref, its tracking ref, live `origin`, and Draft PR #2 head are identical;
the PR is OPEN, Draft, and based on `main`. T150 is `[x]`; T151, T152, and T070
are `[ ]`; attempt-04 remains `ATTEMPT_04_PROHIBITED`.

The retained historical attempt-02 evidence remains external to Git. Its
`checksums.sha256` file has SHA-256:

```text
cc53c4b4bc3cefdc7a2363c6446741e3abfc65e768ac0db71123aa593be528ed
```

Task 11 closes repository verification and status projection only. It does not
turn historical C2a evidence into current evidence, create runtime evidence, or
establish C1, C2, G1, controlled arrival, direct reset, repeatability, or a
physical safety claim.

## 2. Deterministic contradictions in the prior closure procedure

### 2.1 Collection-order and sorted digests were conflated

The prior shell function extracted pytest node IDs and immediately applied
`sort -u`. It therefore hashed the sorted current-GREEN list. For the approved
966 current-GREEN nodes, that operation yields:

```text
00a6e84c5d2e1f623f2211db8272ca95859e8050417f7c25cbfeef9afd84efc7
```

Task 10 instead approved the digest of the same 966 node IDs in pytest
collection order:

```text
1c8e6a8e9b09da6b06435ea6c75191c5fb4b3c3fa7e1b97161951e65249d45ad
```

Both digests are correct for different ordered byte streams. The prior
procedure could not satisfy its own expected digest because it destroyed
collection order before hashing. Task 11 must retain and compare both views;
neither digest substitutes for the other.

### 2.2 A tracked-only archive cannot execute the historical-evidence node

The node

```text
tests/test_g1_pose_conditioned_tracking_cli.py::test_t152_attempt02_is_historical_and_emits_exact_refresh_blocker_before_close
```

reads the ignored directory

```text
outputs/evidence/G1/c2a-static-preliminary-0ace57ce7169-attempt-02
```

A pure `git archive` contains tracked files only, so that directory cannot be
present. Copying, linking, rewriting, or rehashing attempt-02 would defeat the
portable-checkout claim and the retained-evidence boundary. The node must pass
in the main checkout and be classified explicitly as an external-input current
GREEN node. A portable archive must deselect it openly and must not claim to
have executed all 966 current-GREEN nodes.

## 3. Dual-list and dual-digest contract

Task 11 creates four node-ID lists at both clean `V` and clean `P`.

### 3.1 Full collection views

`all-nodeids.collection.txt`:

- is extracted from one complete `pytest --collect-only -q` invocation;
- preserves pytest collection order byte-for-byte;
- contains exactly 1091 non-empty node IDs; and
- contains no duplicate node ID.

`all-nodeids.sorted.txt`:

- is `sort -u all-nodeids.collection.txt`;
- also contains exactly 1091 node IDs; and
- is used only for `comm`, disjointness, subset, union, and classification
  proofs.

The count equality between the collection and sorted views proves uniqueness.
The sorted view must never overwrite or stand in for the collection-order view.

### 3.2 Current-GREEN views

`current-green.collection.txt` is produced by removing the exact 125
intentional future-RED node IDs from `all-nodeids.collection.txt` while
preserving the order of all remaining lines. It contains exactly 966 nodes.

`current-green.sorted.txt` is the sorted unique copy of
`current-green.collection.txt`. It is used for classification and for the
portable/external split. It also contains exactly 966 nodes.

The authoritative current-GREEN snapshots are:

```text
collection-order count = 966
collection-order SHA-256 =
1c8e6a8e9b09da6b06435ea6c75191c5fb4b3c3fa7e1b97161951e65249d45ad

sorted count = 966
sorted SHA-256 =
00a6e84c5d2e1f623f2211db8272ca95859e8050417f7c25cbfeef9afd84efc7
```

The digest input is the complete UTF-8 list file including one trailing newline
per node ID. `V` pre-projection and `P` final-projection must compare:

- both full-collection list files;
- both current-GREEN list files;
- the full and current counts;
- both current-GREEN SHA-256 values; and
- normalized JUnit totals for tests, failures, errors, and skipped.

At both commits the main-checkout current-GREEN JUnit totals are exactly
`tests=966`, `failures=0`, `errors=0`, and `skipped=0`.

## 4. External-evidence manifest and classification

The later `V` commit adds the tracked manifest:

```text
configs/repository/external-evidence-nodeids.txt
```

Its contract is exact:

- one non-comment, non-empty line;
- sorted and unique;
- the sole line is the historical attempt-02 node named in Section 2.2;
- no unknown or additional node is accepted;
- the node must be present in the complete collection;
- the node must not appear in the intentional future-RED manifest; and
- the manifest file has a recorded lowercase SHA-256 computed from its tracked
  bytes.

This classification does not make the node a future-RED, skip, xfail, or
optional test. In the main checkout it remains one of the 966 current-GREEN
nodes and must actually pass.

The main-checkout partition is:

```text
1091 total
= 966 current-GREEN
  + 125 intentional future-RED
= 965 portable current-GREEN
  + 1 external-evidence current-GREEN
  + 125 intentional future-RED
```

Main-checkout verification must:

- execute all 966 current-GREEN nodes by deselecting only the exact 125
  intentional future-RED nodes;
- run the external node explicitly as a focused confirmation;
- observe its exact
  `CURRENT_C2A_REFRESH_REQUIRED_AFTER_GEOMETRY_SCHEMA_CHANGE` blocker;
- retain its `factory_builder` call count of zero; and
- compute the attempt-02 `checksums.sha256` file SHA before and after execution
  and require the approved value both times.

## 5. Portable archive and G0 contract

### 5.1 Pure archive execution

The archive is created only with:

```bash
git archive "$VERIFY_COMMIT" | tar -x -C "$CLEAN_DIR"
```

It receives no copied, linked, mounted, or rewritten attempt-02 evidence and
does not read the original worktree. Inside the archive:

- complete collection still yields the same 1091 node IDs;
- the exact 125 intentional future-RED nodes are deselected;
- the exact one external-evidence node is also deselected;
- exactly 965 portable current-GREEN nodes execute and pass; and
- the JUnit totals are `tests=965`, `failures=0`, `errors=0`, `skipped=0`.

The procedure must never report the portable run as 966 tests. The external
node's main-checkout PASS is recorded separately.

### 5.2 Set proof

Let `P_green`, `E_external`, and `F_future` denote the sorted portable GREEN,
external-evidence GREEN, and intentional future-RED sets. Task 11 proves:

```text
P_green ∩ E_external = ∅
P_green ∩ F_future = ∅
E_external ∩ F_future = ∅
P_green ∪ E_external ∪ F_future = full collection
|P_green| + |E_external| + |F_future| = 965 + 1 + 125 = 1091
```

Any overlap, missing node, extra node, duplicate node, or unclassified node is a
hard stop.

### 5.3 G0 report and manifest

The later G0 implementation records at least:

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
reads_original_worktree: false
```

It also retains both current-GREEN list counts and digests from Section 3 and
the normalized main/portable JUnit totals. `PASS_BENCHMARK` in this G0 context
means repository-integrity verification only. It explicitly states that the
external-evidence node passed separately in the main checkout and was not part
of portable archive execution. It cannot imply C1, C2, G1, simulator, Contact,
or physical evidence.

## 6. Verification-infrastructure commit V

After human approval of this design, `V` may modify only the verification
infrastructure needed by this contract:

- create `configs/repository/external-evidence-nodeids.txt`;
- modify `scripts/check_clean_checkout.py`;
- modify `tests/test_clean_checkout_cli.py`; and
- modify the Task 11 verification helper and its plan text.

`V` uses RED-to-GREEN development without Isaac Sim. The RED contracts cover:

1. exact external-manifest count, ordering, uniqueness, and spelling;
2. collection membership of the external node;
3. disjoint external and future-RED sets;
4. portable deselection of exactly 125 future-RED plus one external node;
5. portable selection and PASS count of exactly 965;
6. complete classification of exactly 1091 nodes;
7. the G0 report/manifest fields and counts in Section 5.3;
8. `reads_original_worktree=false` and no external evidence projection;
9. generation and comparison of both current-GREEN list views and digests; and
10. main-checkout attempt-02 checksum preservation and exact stale blocker.

The infrastructure must fail closed on an absent or malformed external
manifest, collection drift, set overlap, count drift, digest drift, JUnit
failure/error/skip, original-worktree access, or a portable run that claims the
external node.

This design checkpoint `D` does not create those tests or implementation.

## 7. Commit topology and projection rule

The old direct edge `E_impl -> P` is replaced by:

```text
E_impl = aa47af3946f2f9f934147b4b263affe345a9d450
→ D = this design checkpoint
→ V = verification infrastructure implementation
→ P = final T152 projection/status commit
→ FINAL_E2 = P
```

The parent invariants are:

```text
D^ = E_impl
V^ = D
P^ = V
```

The complete pre-projection suite runs only after `V` is committed and the
worktree is clean. If it passes, `P` may modify only:

- `specs/001-benchmark-reconstruction/tasks.md`, changing T152 `[ ]` to `[x]`;
  and
- `specs/001-benchmark-reconstruction/g1-contact-exclusion-t152-implementation-plan.md`,
  recording the already-known literal SHAs for `E_impl`, `D`, and `V`.

Tracked files must not contain `P`'s own SHA. After creating `P`, bind
`FINAL_E2=$(git rev-parse HEAD)`, require `P^=V`, rerun the identical final
suite with list/digest/JUnit comparison against the clean `V` snapshot, and
produce G0 at `P`. Any later tracked change invalidates `FINAL_E2` and requires
a newly reviewed projection.

## 8. Stop conditions and next gate

Stop without creating `P` or G0 if any of these occurs:

- either approved current-GREEN digest changes;
- the collection-order list is sorted before its digest is computed;
- the sorted list is used as the collection-order authority;
- the external node is missing, renamed, duplicated, future-RED, skipped, or
  not actually passing in the main checkout;
- attempt-02 checksum bytes change or the stale blocker/factory-zero contract
  changes;
- the portable archive reads or receives files from the original worktree;
- the portable archive executes anything other than 965 GREEN nodes;
- any partition overlap or unclassified node exists;
- any JUnit failure, error, or skip appears in a GREEN run;
- `D^=E_impl`, `V^=D`, or `P^=V` is false;
- the projection commit changes anything beyond the two authorized Markdown
  files;
- T151 or T070 advances; or
- Isaac Sim, fresh C2a, attempt-04, C2b, C3, or an episode is requested without
  its separate approval.

The next permitted stage after this document is human review, followed by a
separately authorized `V` RED-to-GREEN implementation. T152 remains `[ ]` and
attempt-04 remains `ATTEMPT_04_PROHIBITED` until the later projection procedure
completes.

## 9. Design conclusion

```text
VERIFICATION_LIST_AUTHORITY = COLLECTION_ORDER_AND_SORTED_VIEWS
CURRENT_GREEN_COLLECTION_COUNT = 966
CURRENT_GREEN_COLLECTION_SHA256 = 1c8e6a8e9b09da6b06435ea6c75191c5fb4b3c3fa7e1b97161951e65249d45ad
CURRENT_GREEN_SORTED_COUNT = 966
CURRENT_GREEN_SORTED_SHA256 = 00a6e84c5d2e1f623f2211db8272ca95859e8050417f7c25cbfeef9afd84efc7
MAIN_CHECKOUT_CURRENT_GREEN = 966
PORTABLE_ARCHIVE_CURRENT_GREEN = 965
EXTERNAL_EVIDENCE_CURRENT_GREEN = 1
INTENTIONAL_FUTURE_RED = 125
TOTAL_COLLECTION = 1091
READS_ORIGINAL_WORKTREE = false
COMMIT_TOPOLOGY = E_impl -> D -> V -> P
FINAL_E2 = P
T152 = OPEN_PENDING_V_AND_PROJECTION
ATTEMPT_04 = PROHIBITED
```
