#!/usr/bin/env python3
"""Execute one immutable G1 baseline selection and verify its exact outcome."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
from xml.etree import ElementTree


NAMED_SELECTIONS = {
    "t152_expected_red",
    "t152_green_controls",
    "original_green",
    "intentional_future_red",
    "exact_hard_limit",
}


def _digest(node_ids: list[str], *, sorted_ids: bool) -> str:
    values = sorted(node_ids) if sorted_ids else node_ids
    return hashlib.sha256(("\n".join(values) + "\n").encode("utf-8")).hexdigest()


def _parse_expected_classifications(values: list[str]) -> dict[str, int]:
    parsed: dict[str, int] = {}
    for value in values:
        name, separator, count_text = value.partition("=")
        if not separator or not name or name in parsed:
            raise ValueError(f"invalid or duplicate classification expectation: {value!r}")
        parsed[name] = int(count_text)
    return parsed


def _collected_node_ids(output: str) -> list[str]:
    return [
        line.strip()
        for line in output.splitlines()
        if "::" in line and line.strip().startswith("tests/")
    ]


def _junit_summary(path: Path) -> tuple[dict[str, int], list[tuple[str, str]]]:
    root = ElementTree.parse(path).getroot()
    counts = {"passed": 0, "failed": 0, "error": 0, "skipped": 0}
    failures: list[tuple[str, str]] = []
    for case in root.findall(".//testcase"):
        failure = case.find("failure")
        error = case.find("error")
        skipped = case.find("skipped")
        if failure is not None:
            counts["failed"] += 1
            failures.append(
                (failure.attrib.get("message", ""), failure.text or "")
            )
        elif error is not None:
            counts["error"] += 1
        elif skipped is not None:
            counts["skipped"] += 1
        else:
            counts["passed"] += 1
    return counts, failures


def _is_assertion_red(failure: tuple[str, str]) -> bool:
    message, detail = failure
    return (
        message.startswith("AssertionError")
        or message.startswith("Failed: DID NOT RAISE")
        or message.startswith("assert ")
        or "AssertionError" in detail
        or "Failed: DID NOT RAISE" in detail
    )


def _validate_inventory(selection: dict[str, object]) -> tuple[list[str], list[str]]:
    node_ids = list(selection["node_ids"])
    outcomes = list(selection["observed_outcomes"])
    classifications = list(selection["classifications"])
    if not node_ids or len(node_ids) != len(set(node_ids)):
        raise ValueError("selection node IDs must be non-empty and unique")
    if not (len(node_ids) == len(outcomes) == len(classifications) == selection["count"]):
        raise ValueError("selection parallel arrays/count disagree")
    if _digest(node_ids, sorted_ids=True) != selection["node_id_sha256"]:
        raise ValueError("selection sorted node-ID digest mismatch")
    if _digest(node_ids, sorted_ids=False) != selection["ordered_node_id_sha256"]:
        raise ValueError("selection ordered node-ID digest mismatch")
    observed_classifications: dict[str, int] = {}
    for classification in classifications:
        observed_classifications[classification] = (
            observed_classifications.get(classification, 0) + 1
        )
    if observed_classifications != selection["classification_counts"]:
        raise ValueError("selection classification counts disagree")
    return node_ids, classifications


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--inventory", type=Path, required=True)
    parser.add_argument("--selection", choices=sorted(NAMED_SELECTIONS), required=True)
    expected = parser.add_mutually_exclusive_group(required=True)
    expected.add_argument("--expect-pass", type=int)
    expected.add_argument("--expect-fail", type=int)
    parser.add_argument("--expect-classification", action="append", default=[])
    args = parser.parse_args(argv)

    inventory = json.loads(args.inventory.read_text(encoding="utf-8"))
    unknown = set(inventory["selections"]) - NAMED_SELECTIONS
    if unknown:
        raise ValueError(f"inventory contains unsupported immutable selections: {sorted(unknown)}")
    selection = inventory["selections"][args.selection]
    node_ids, _classifications = _validate_inventory(selection)

    expected_classifications = _parse_expected_classifications(
        args.expect_classification
    )
    if expected_classifications and expected_classifications != selection[
        "classification_counts"
    ]:
        raise ValueError("requested classification counts do not match inventory")

    collect = subprocess.run(
        [sys.executable, "-m", "pytest", "--collect-only", "-q", *node_ids],
        check=False,
        capture_output=True,
        text=True,
    )
    if collect.returncode != 0:
        sys.stdout.write(collect.stdout)
        sys.stderr.write(collect.stderr)
        raise RuntimeError("frozen selection collection failed")
    collected = _collected_node_ids(collect.stdout)
    if collected != node_ids:
        raise ValueError("frozen selection is missing, extra, duplicate, or reordered")

    with tempfile.TemporaryDirectory(prefix="g1-node-inventory-") as temporary:
        junit = Path(temporary) / "result.xml"
        run = subprocess.run(
            [
                sys.executable,
                "-m",
                "pytest",
                "-q",
                "--tb=no",
                *node_ids,
                f"--junitxml={junit}",
            ],
            check=False,
            text=True,
        )
        if not junit.is_file():
            raise RuntimeError("pytest did not create JUnit output")
        counts, failures = _junit_summary(junit)

    expected_pass = args.expect_pass or 0
    expected_fail = args.expect_fail or 0
    required = {
        "passed": expected_pass,
        "failed": expected_fail,
        "error": 0,
        "skipped": 0,
    }
    if counts != required or counts != selection["junit_summary"]:
        raise AssertionError(f"outcome mismatch: observed={counts}, required={required}")
    if expected_fail and (
        run.returncode != 1
        or len(failures) != expected_fail
        or any(not _is_assertion_red(failure) for failure in failures)
    ):
        raise AssertionError("expected RED selection was not exact assertion failure output")
    if expected_pass and run.returncode != 0:
        raise AssertionError(f"expected GREEN selection returned {run.returncode}")
    print(
        json.dumps(
            {
                "selection": args.selection,
                "counts": counts,
                "classification_counts": selection["classification_counts"],
                "node_id_sha256": selection["node_id_sha256"],
                "ordered_node_id_sha256": selection["ordered_node_id_sha256"],
                "verified": True,
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
