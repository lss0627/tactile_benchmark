# Group Meeting Progress Deck Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Generate a verified, editable Chinese 16:9 PPTX that accurately reports the repository's current project progress and claim boundaries.

**Architecture:** A self-contained Python generator uses `python-pptx` to compose native text, shapes, diagrams, and selected repository screenshots. LibreOffice renders the deck to PDF, and PDF page images are visually inspected before delivery.

**Tech Stack:** Python 3, python-pptx, Pillow, LibreOffice Impress, Poppler utilities, existing PNG/JSON/Markdown artifacts.

---

### Task 1: Freeze slide evidence and narrative

**Files:**
- Read: `findings.md`
- Read: `specs/001-benchmark-reconstruction/spec.md`
- Read: `specs/001-benchmark-reconstruction/research.md`
- Read: `outputs/fr3_press_button_press_runtime/*.json`
- Create: `outputs/group_meeting_report/evidence_summary.md`

- [ ] **Step 1:** Record exact numerical claims, artifact paths, and their allowed claim class in `evidence_summary.md`.
- [ ] **Step 2:** Check every slide claim against at least one repository artifact.
- [ ] **Step 3:** Confirm that smoke, diagnostic, and benchmark labels are not mixed.

### Task 2: Build the editable PPTX

**Files:**
- Create: `outputs/group_meeting_report/generate_group_meeting_ppt.py`
- Create: `outputs/group_meeting_report/组会汇报_项目进展_2026-07-10.pptx`

- [ ] **Step 1:** Create a temporary virtual environment and install `python-pptx` and Pillow.
- [ ] **Step 2:** Implement reusable helpers for slide title, footer, cards, tags, arrows, diagrams, image crop, and charts.
- [ ] **Step 3:** Implement the eleven-slide narrative defined in the design document.
- [ ] **Step 4:** Run the generator and confirm the PPTX opens as a valid ZIP/Open XML package.

### Task 3: Render and visually validate

**Files:**
- Create: `outputs/group_meeting_report/rendered/组会汇报_项目进展_2026-07-10.pdf`
- Create: `outputs/group_meeting_report/rendered/slide-*.png`

- [ ] **Step 1:** Convert the PPTX to PDF with LibreOffice headless mode.
- [ ] **Step 2:** Render all PDF pages to PNG with `pdftoppm`.
- [ ] **Step 3:** Create and inspect a contact sheet; check clipping, font fallback, overlap, image crop, chart scale, and footers.
- [ ] **Step 4:** Revise the generator and repeat rendering if any visual defect is found.

### Task 4: Final verification and handoff

**Files:**
- Verify: `outputs/group_meeting_report/组会汇报_项目进展_2026-07-10.pptx`
- Verify: `outputs/group_meeting_report/rendered/组会汇报_项目进展_2026-07-10.pdf`

- [ ] **Step 1:** Run a PPTX structure check for slide count, embedded media, and non-empty titles.
- [ ] **Step 2:** Run `pdfinfo` and confirm 11 pages and 16:9 page geometry.
- [ ] **Step 3:** Compare the final deck against the design's content-boundary checklist.
- [ ] **Step 4:** Deliver clickable paths to the PPTX and PDF preview.
