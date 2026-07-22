# Automatic Inner-Wall Initialization for Side-View Robustness

**Date:** 2026-07-22  
**Status:** Approved for implementation

## Goal

Run the same conservative upper/lower inner-wall extraction pipeline over every remaining side-view image group without hand-tuned per-group wall locations, and export a full stitched-image review figure for each group.

## Context

The current detector uses fixed outer-wall baselines from the development image group to define narrow initialization windows. Five image groups contain the same physical wall transitions but at substantially shifted ROI rows. Their stitching diagnostics remain usable; the failure occurs before tracking because the hard-coded initial windows do not include the appropriate Canny edges.

## Options considered

1. **Per-group manual baselines.** Fast for the current data but not a robustness improvement and does not generalize.
2. **Widen fixed initialization windows.** Removes some failures but can admit unrelated transitions, making the initial wall choice less deterministic.
3. **Automatic profile-based initialization (selected).** Detect polarity-correct, physically plausible upper and lower edge candidates from the median longitudinal profile, then use the existing narrow Canny/dynamic-programming tracking bands. This removes dependence on image-family-specific baseline positions while keeping the existing continuity and evidence gates.

## Design

### Initialization

- Compute the existing smoothed median longitudinal profile and its vertical gradient.
- Search the upper and lower halves of the ROI independently for polarity-correct edge candidates.
- Score candidates by gradient magnitude and, when configured baselines are informative, proximity to the expected inner-wall location.
- Form a top/bottom pair only if it is ordered and has a plausible opening; otherwise fail closed with a diagnostic error.
- Retain configured initial windows as a preferred prior, but automatically fall back to globally valid candidates when those windows lack adequate evidence.

### Tracking and quality gates

- Leave Canny polarity checks, dynamic-programming path tracking, seam penalties, direct-observation accounting, interpolation limits, and long-gap rejection unchanged.
- Do not lower `minObservedFraction`, `maxInvalidGapCols`, or any conservative uncertainty terms to obtain a pass.

### Review artifacts

- Re-run `review_side_robustness` across all 13 groups.
- For every successful group, export the existing two-panel review image: full stitched strip with red/green walls, followed by the complete side ROI with unsupported points marked in orange.
- For any group that remains rejected, export the raw full stitched strip and record the exact rejection reason.
- Save a CSV summary containing observation coverage and stitching quality per group.

## Acceptance criteria

- No group-specific hard-coded baseline values are added.
- Existing baseline groups keep their conservative quality behavior.
- The five previously failed groups are re-evaluated with the same evidence gates.
- All generated figures and the summary are visually inspected before reporting results.
