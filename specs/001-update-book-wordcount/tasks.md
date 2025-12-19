# Tasks: Update Book Word Count

**Input**: Design documents from `/specs/001-update-book-wordcount/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, quickstart.md

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1)
- Include exact file paths in descriptions

---

## Phase 1: Setup (Word Count Tooling)

**Purpose**: Establish the tools required to measure task completion.

- [x] T001 Install word count dependency in `physical-ai-humanoid-robotics-book/` by running `npm install word-count`.
- [x] T002 Create the word count script in `physical-ai-humanoid-robotics-book/count-words.js` using the code from `quickstart.md`.

---

## Phase 2: Foundational (Establish Baseline)

**Purpose**: Measure the current state before work begins.

- [x] T003 Run `node count-words.js` inside `physical-ai-humanoid-robotics-book/` to get the initial total word count.

---

## Phase 3: User Story 1 - Content Expansion (Priority: P1) 🎯 MVP

**Goal**: Expand all book chapters to collectively reach the 80,000-word target, as defined in the feature specification.

**Independent Test**: The `count-words.js` script reports a total word count greater than or equal to 80,000 after all tasks in this phase are complete.

### Implementation for User Story 1

- [x] T004 [P] [US1] Expand `capstone/code.md` to 8,000-10,000 words by adding new subsections, theory, math, and real-world humanoid examples in `physical-ai-humanoid-robotics-book/docs/capstone/code.md`.
- [x] T005 [P] [US1] Expand `capstone/overview.md` to 8,000-10,000 words by adding new subsections, theory, math, and real-world humanoid examples in `physical-ai-humanoid-robotics-book/docs/capstone/overview.md`.
- [x] T006 [P] [US1] Expand `digital-twin/overview.md` to 8,000-10,000 words by adding new subsections, theory, math, and real-world humanoid examples in `physical-ai-humanoid-robotics-book/docs/digital-twin/overview.md`.
- [x] T007 [P] [US1] Expand `digital-twin/physics.md` to 8,000-10,000 words by adding new subsections, theory, math, and real-world humanoid examples in `physical-ai-humanoid-robotics-book/docs/digital-twin/physics.md`.
- [ ] T008 [P] [US1] Expand `digital-twin/rendering.md` to 8,000-10,000 words by adding new subsections, theory, math, and real-world humanoid examples in `physical-ai-humanoid-robotics-book/docs/digital-twin/rendering.md`.
- [x] T009 [P] [US1] Expand `digital-twin/sensors.md` to 8,000-10,000 words by adding new subsections, theory, math, and real-world humanoid examples in `physical-ai-humanoid-robotics-book/docs/digital-twin/sensors.md`.
- [x] T010 [P] [US1] Expand `hardware.md` to 8,000-10,000 words by adding new subsections, theory, math, and real-world humanoid examples in `physical-ai-humanoid-robotics-book/docs/hardware.md`.
- [x] T011 [P] [US1] Expand `intro.md` to 8,000-10,000 words by adding new subsections, theory, math, and real-world humanoid examples in `physical-ai-humanoid-robotics-book/docs/intro.md`.
- [x] T012 [P] [US1] Expand `nvidia-isaac/isaac-ros.md` to 8,000-10,000 words by adding new subsections, theory, math, and real-world humanoid examples in `physical-ai-humanoid-robotics-book/docs/nvidia-isaac/isaac-ros.md`.
- [x] T013 [P] [US1] Expand `nvidia-isaac/isaac-sim.md` to 8,000-10,000 words by adding new subsections, theory, math, and real-world humanoid examples in `physical-ai-humanoid-robotics-book/docs/nvidia-isaac/isaac-sim.md`.
- [ ] T014 [P] [US1] Expand `nvidia-isaac/nav2.md` to 8,000-10,000 words by adding new subsections, theory, math, and real-world humanoid examples in `physical-ai-humanoid-robotics-book/docs/nvidia-isaac/nav2.md`.
- [x] T015 [P] [US1] Expand `nvidia-isaac/overview.md` to 8,000-10,000 words by adding new subsections, theory, math, and real-world humanoid examples in `physical-ai-humanoid-robotics-book/docs/nvidia-isaac/overview.md`.
- [x] T016 [P] [US1] Expand `references.md` to 8,000-10,000 words by adding new subsections, theory, math, and real-world humanoid examples in `physical-ai-humanoid-robotics-book/docs/references.md`.
- [ ] T017 [P] [US1] Expand `ros2/fundamentals.md` to 8,000-10,000 words by adding new subsections, theory, math, and real-world humanoid examples in `physical-ai-humanoid-robotics-book/docs/ros2/fundamentals.md`.
- [ ] T018 [P] [US1] Expand `ros2/rclpy-agents.md` to 8,000-10,000 words by adding new subsections, theory, math, and real-world humanoid examples in `physical-ai-humanoid-robotics-book/docs/ros2/rclpy-agents.md`.
- [ ] T019 [P] [US1] Expand `ros2/urdf-modeling.md` to 8,000-10,000 words by adding new subsections, theory, math, and real-world humanoid examples in `physical-ai-humanoid-robotics-book/docs/ros2/urdf-modeling.md`.
- [ ] T020 [P] [US1] Expand `vla/llm-planning.md` to 8,000-10,000 words by adding new subsections, theory, math, and real-world humanoid examples in `physical-ai-humanoid-robotics-book/docs/vla/llm-planning.md`.
- [ ] T021 [P] [US1] Expand `vla/multi-step-reasoning.md` to 8,000-10,000 words by adding new subsections, theory, math, and real-world humanoid examples in `physical-ai-humanoid-robotics-book/docs/vla/multi-step-reasoning.md`.
- [ ] T022 [P] [US1] Expand `vla/overview.md` to 8,000-10,000 words by adding new subsections, theory, math, and real-world humanoid examples in `physical-ai-humanoid-robotics-book/docs/vla/overview.md`.
- [ ] T023 [P] [US1] Expand `vla/whisper.md` to 8,000-10,000 words by adding new subsections, theory, math, and real-world humanoid examples in `physical-ai-humanoid-robotics-book/docs/vla/whisper.md`.

---

## Phase 4: Final Verification

**Purpose**: Ensure the primary success criterion of the feature has been met.

- [ ] T024 Run the `node count-words.js` script again inside `physical-ai-humanoid-robotics-book/`.
- [ ] T025 Verify the total word count is greater than or equal to 80,000.

---

## Dependencies & Execution Order

- **Phase 1 (Setup)** must complete before all other phases.
- **Phase 2 (Foundational)** depends on Phase 1.
- **Phase 3 (User Story 1)** depends on Phase 2. All tasks within Phase 3 can be performed in parallel.
- **Phase 4 (Final Verification)** depends on the completion of all tasks in Phase 3.

## Implementation Strategy

### MVP First

The MVP for this feature is achieving the 80,000-word count. A good strategy is to:
1. Complete Phases 1 and 2.
2. Select a small batch of chapters from Phase 3 (e.g., T004-T006).
3. Complete them and run the verification script to validate the process and quality of expansion.
4. Proceed with the remaining Phase 3 tasks.

### Parallel Execution

All 20 tasks (T004-T023) in Phase 3 are independent and can be executed in parallel. This is the most efficient path to completion.
