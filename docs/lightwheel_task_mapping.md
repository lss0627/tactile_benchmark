# Lightwheel Task Mapping Plan

This is a planning document only. It does not add tasks or connect a real
Lightwheel runtime.

## Priority Order

Priority 1 remains the tactile-specific suites:

- Tactile-Contact: button press, soft press, slider push, force-regulated
  contact.
- Tactile-Assembly: peg insertion, plug/socket insertion, contact-rich
  alignment and insertion.

These tasks are the benchmark's tactile contribution and should stay ahead of
any broad LIBERO-compatible expansion.

## Base-LIBERO-Compatible Suite Candidates

Lightwheel-LIBERO style tasks that may later map cleanly into a compatibility
suite include:

- pick-and-place variants with simple object state;
- drawer or slider manipulation;
- button/switch interactions;
- container open/close tasks;
- simple insertion or placement tasks with clear success predicates.

The first compatibility target should be a small representative subset, not 30
tasks. Each task must preserve the existing action schema, observation schema,
dataset schema, tactile masks, and evaluation output format.

## Non-Goals Now

- Do not implement the 30-task expansion.
- Do not import Lightwheel task code.
- Do not download or redistribute Lightwheel assets.
- Do not claim real Lightwheel performance from mock runtime checks.
