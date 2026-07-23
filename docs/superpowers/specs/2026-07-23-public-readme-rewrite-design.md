# Public README rewrite design

**Date:** 2026-07-23  
**Status:** Approved for documentation design; pending user review before README implementation.

## Goal

Rewrite the project README as a public-shareable technical overview based on the current glass-tube analysis workflow. The document must explain the pipeline and responsibilities of the MATLAB scripts without disclosing credentials, raw-data details, sample identifiers, absolute paths, calibration constants, or measured production outcomes.

## Audience

Engineers and reviewers who need to understand the project architecture, intended usage, quality controls, and test coverage without access to the experimental environment.

## Scope

- Rewrite `README.md` only after this design is reviewed.
- Preserve the project name and MATLAB-oriented usage.
- Describe the processing pipeline at a conceptual level.
- Include a script-responsibility table covering the main entry point, configuration, calibration, end-face analysis, stitching, side-wall tracking, rod optimization, diagnostics, review utilities, and tests.
- Provide generic quick-start and test commands that do not embed data locations or file names.
- State conservative measurement principles and quality gates.
- State that raw image data, credentials, tokens, and environment-specific settings are intentionally not documented.

## Explicit exclusions

The rewritten README must not disclose:

- API keys, access tokens, passwords, hashes, or credentials;
- raw image names, data-set names, acquisition batches, or customer/sample identifiers;
- absolute filesystem paths, host/user names, or machine-specific directories;
- hard-coded calibration constants, dates tied to experimental measurements, or production result values;
- cached result contents, benchmark timings, or detailed historical failure-group naming.

## Proposed structure

1. **Project overview** — the problem solved and the conservative objective.
2. **Workflow** — end-face validation, side-view stitching, inner-wall tracking, uncertainty contraction, and straight-rod optimization.
3. **Core scripts** — a concise responsibility table with source-file names only.
4. **Quick start** — generic configuration, calibration, analysis, and test sequence.
5. **Quality and conservatism** — no enlargement of aperture, direct-observation semantics, gap handling, and failure-closed behavior.
6. **Outputs** — generic description of result structures, diagnostic graphics, and optional exports.
7. **Data and security** — explicit statement that sensitive data and credentials are excluded.

## Writing style

- Chinese technical documentation.
- Concise, reproducible, and independent of a specific local environment.
- Do not claim certification or report a real measured diameter.
- Use generic placeholders such as “独立左右端面图像” and “侧视序列图像”.

## Acceptance criteria

- A reader understands what every MATLAB script is for without reading the code.
- The README gives a safe generic execution path.
- No sensitive data, credentials, sample-specific names, raw data paths, measured calibration values, or real performance/result figures remain in the README.
- Only `README.md` is changed during implementation.
