# Quickstart: Documentation Validation and Future Execution

This quickstart validates the Spec Kit package produced by this documentation run. It does **not**
claim that the reconstruction tasks have been implemented.

## 1. Select the feature

From the repository root:

```bash
export SPECIFY_FEATURE=001-benchmark-reconstruction
bash .specify/scripts/bash/check-prerequisites.sh --json --require-tasks --include-tasks
```

Expected: the command resolves `spec.md`, `plan.md`, `tasks.md`, and the available design documents
under `specs/001-benchmark-reconstruction/`.

## 2. Validate documentation and machine-readable contracts

```bash
rg -n "NEEDS CLARIFICATION|\[FEATURE\]|\[###-feature|TODO|TBD" \
  specs/001-benchmark-reconstruction
git diff --check -- .specify specs/001-benchmark-reconstruction
python -m json.tool \
  specs/001-benchmark-reconstruction/contracts/evidence-manifest.schema.json >/dev/null
python -m json.tool \
  specs/001-benchmark-reconstruction/contracts/gate-status.schema.json >/dev/null
```

Expected: the placeholder search returns no matches and every other command exits zero.

## 3. Review the implementation handoff

Read in this order:

1. [spec.md](./spec.md) — scope, user stories, FRs, and measurable success.
2. [research.md](./research.md) — audited deficiencies and chosen design boundaries.
3. [plan.md](./plan.md) — gate architecture and implementation phases.
4. [data-model.md](./data-model.md) and [contracts/](./contracts/) — normative state and API rules.
5. [tasks.md](./tasks.md) — dependency-ordered implementation units.
6. [implementation.md](./implementation.md) — execution protocol and stop rules.
7. [acceptance.md](./acceptance.md) — command/evidence matrix and current status.

## 4. Begin implementation only when authorized

The future implementation session starts at G0 and follows `tasks.md` in order. At minimum, it must
run the no-simulator regression suite before and after each gate:

```bash
python -m pip install -e '.[test]'
pytest -q
```

Simulator commands in `acceptance.md` are future target commands. They remain blocked until a
supported Isaac Sim runtime and licensed FR3 assets are configured. Dry-run success must be recorded
as `PASS_SMOKE`, never `PASS_BENCHMARK`.

## 5. Evidence handling

Generated results belong under an immutable run directory such as:

```text
outputs/evidence/<gate-id>/<run-id>/
├── manifest.json
├── command.log
├── report.json
└── artifacts/
```

Validate `manifest.json` against `contracts/evidence-manifest.schema.json`, store artifact hashes,
and update the canonical gate status only after review. Changing semantic code/config/assets makes
older evidence stale and returns the gate to `IN_PROGRESS`.
