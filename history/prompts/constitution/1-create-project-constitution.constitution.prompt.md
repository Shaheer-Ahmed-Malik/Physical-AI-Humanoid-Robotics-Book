---
id: 1
title: Create Project Constitution
stage: constitution
date: 2025-12-10
surface: agent
model: gemini
feature: none
branch: main
user: 
command: /sp.constitution
labels: [constitution]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - none
---

## Prompt

/sp.constitution

Project: Book on Physical AI & Humanoid Robotics (built using Docusaurus, GitHub Pages, Spec-Kit Plus)

Core principles:
- Scientific accuracy grounded in modern AI, robotics, and embodied intelligence research
- Reader engagement through clear, accessible explanations without losing technical correctness
- Conceptual clarity for non-experts while remaining credible for professionals
- Future-focused insight into real-world humanoid applications
- Transparency and reproducibility for all factual claims

Key standards:
- Citation format: APA or IEEE (consistent throughout)
- Minimum 40% peer-reviewed sources (ICRA, IROS, NeurIPS, Nature Robotics, etc.)
- Industry whitepapers allowed for cutting-edge developments (Boston Dynamics, Tesla, Figure, OpenAI, NVIDIA, Sanctuary)
- Professional but easy tone; reading level Flesch-Kincaid Grade 9–11
- Zero tolerance for plagiarism
- Complex topics simplified using examples, diagrams, and analogies

Constraints:
- Total word count: 25,000–40,000 words
- 10–14 chapters, each with clear learning objectives
- Format: Docusaurus MDX files
- Visuals/diagrams stored in `/static/img/`
- Must build and deploy cleanly on GitHub Pages

Success criteria:
- All claims traceable to reliable sources
- Content remains engaging and reader-friendly while preserving scientific accuracy
- Passes plagiarism and fact-checking review
- Logical narrative flow across all chapters
- Book builds solid foundational understanding of Physical AI & Humanoid Robotics
- Docusaurus deploys successfully without build errors

## Response snapshot

Created the project constitution at `.specify/memory/constitution.md` with version 1.0.0. The constitution includes sections for Core Principles, Key Standards, Constraints, Success Criteria, and Governance, based on the user's input.

## Outcome

- ✅ Impact: Created the initial project constitution, defining the core principles and standards for the project.
- 🧪 Tests: None
- 📁 Files: 
  - .specify/memory/constitution.md
- 🔁 Next prompts: None
- 🧠 Reflection: The process was smooth. The user provided clear and detailed information, which made it easy to generate the constitution.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): None
- Prompt variant (if applicable): None
- Next experiment (smallest change to try): None
