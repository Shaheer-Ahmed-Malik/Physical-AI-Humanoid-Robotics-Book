# Implementation Plan: Update Book Word Count

**Branch**: `001-update-book-wordcount` | **Date**: 2025-12-19 | **Spec**: [specs/001-update-book-wordcount/spec.md](spec.md)
**Input**: Feature specification from `specs/001-update-book-wordcount/spec.md`

## Summary

The feature requires expanding the book's word count to a minimum of 80,000 words. This plan adheres to the user's constraints to only expand the internal depth of existing chapters, without adding new chapters or reordering existing ones. This goal explicitly overrides the 25,000-40,000 word count constraint currently listed in the project constitution, as the feature specification notes that the previous limit was a mistake.

## Technical Context

**Language/Version**: Markdown, Node.js/NPM (for tooling)
**Primary Dependencies**: Docusaurus
**Storage**: Markdown files (`.md`, `.mdx`) in the git repository.
**Testing**: A custom Node.js script will be used to verify word count across all documents.
**Target Platform**: Web (via Docusaurus build)
**Project Type**: Web Application
**Performance Goals**: N/A
**Constraints**: A custom Node.js script (`count-words.js`) will be used to count words across all relevant markdown files to ensure the 80,000-word target is met.
**Scale/Scope**: Expand existing content to a total of at least 80,000 words.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- **VIOLATION**: The feature's 80,000-word minimum requirement violates the constitution's constraint of `Word Count: Total manuscript length must be between 25,000 and 40,000 words.` This violation is justified in the Complexity Tracking section below as the core premise of the feature.

## Project Structure

### Documentation (this feature)

```text
specs/001-update-book-wordcount/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
└── contracts/           # (Not applicable for this feature)
```

### Source Code (repository root)

```text
physical-ai-humanoid-robotics-book/
├── docs/
│   ├── intro.md
│   └── ... (existing book chapters)
├── src/
└── ... (docusaurus project structure)
```

**Structure Decision**: The project is a standard Docusaurus web application. All content to be modified resides in the `physical-ai-humanoid-robotics-book/docs/` directory. No changes to the structure are planned.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Word count exceeds constitutional limit. | The feature spec explicitly states the 80,000-word count is a new, hard requirement and supersedes the previous specification. | The feature's primary success criterion is non-negotiable and cannot be met while adhering to the outdated constraint. The constitution requires an update. |