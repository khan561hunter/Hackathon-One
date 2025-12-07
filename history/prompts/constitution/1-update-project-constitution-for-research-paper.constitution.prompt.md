---
id: 1
title: Update Project Constitution for Research Paper
stage: constitution
date: 2025-12-05
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: user
command: /sp.constitution
labels: ["constitution", "update"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests: null
---

## Prompt

# Project Constitution: Research Paper on AI-Native Software Development

## Article I: Project Overview
**Project Name:** AI-Native Software Development Research Paper
**Objective:** To produce a rigorously researched, academically clear, and reproducible research paper exploring AI-native software development practices, using Spec-Kit Plus and Claude Code for creation, and deploying the final book via Docusaurus to GitHub Pages.

---

## Article II: Core Principles
The following principles shall govern all aspects of this project:

1. **Accuracy:**
   All factual statements must be verified against primary sources to ensure correctness.

2. **Clarity:**
   The paper must be written for an academic audience with a computer science background, ensuring readability and understanding.

3. **Reproducibility:**
   Every claim, method, and result must be traceable and verifiable. Citations must allow readers to reproduce the research.

4. **Rigor:**
   Preference shall be given to peer-reviewed sources, ensuring the highest academic standard.

---

## Article III: Key Standards
All work produced must adhere to the following standards:

1. **Factual Traceability:**
   Every claim must be backed by credible sources.

2. **Citation Format:**
   All sources must be cited in **APA style**.

3. **Source Requirements:**
   At least **50% of sources** must be peer-reviewed journal articles.

4. **Plagiarism Policy:**
   A **0% plagiarism tolerance** is enforced prior to submission.

5. **Writing Clarity:**
   Writing should achieve **Flesch-Kincaid grade level 10–12**, ensuring readability without oversimplification.

---

## Article IV: Constraints
The project shall comply with the following constraints:

1. **Word Count:** 5,000–7,000 words.
2. **Source Count:** Minimum of 15 references.
3. **Document Format:** PDF with embedded citations.

---

## Article V: Success Criteria
The project shall be considered successful if it meets the following criteria:

1. **Source Verification:**
   All factual claims are verified against cited sources.

2. **Plagiarism Check:**
   No plagiarism is detected in the final submission.

3. **Fact-Checking:**
   The final work passes an internal or external fact-checking review.

---

## Article VI: Tools and Deployment
1. **Writing Tools:** Spec-Kit Plus and Claude Code shall be used for drafting and editing.
2. **Book Deployment:** The final document shall be published using **Docusaurus** and deployed to **GitHub Pages**.
3. **Review Cycle:** Iterative reviews shall ensure compliance with the constitution, including source verification, clarity, and rigor.

---

## Article VII: Amendment Procedure
Amendments to this constitution may be made if:

1. They improve academic rigor or clarity.
2. They maintain adherence to primary source verification.
3. They do not compromise reproducibility or ethical standards.

All amendments must be documented and justified.

---

**Ratified:** [Insert Date]
**Author/Project Lead:** [Insert Your Name]

## Response snapshot

The project constitution has been updated to version 2.0.0, reflecting a major structural change to focus on the research paper. The previous constitution was replaced with the new content provided by the user, with dates and version updated.

## Outcome

- ✅ Impact: Major structural update to project constitution.
- 🧪 Tests: No tests run.
- 📁 Files: .specify/memory/constitution.md updated.
- 🔁 Next prompts: User needs to update dependent templates.
- 🧠 Reflection: Successfully updated the constitution based on user input, and handled the PHR creation when the script failed.

## Evaluation notes (flywheel)

- Failure modes observed: `create-phr.sh` script not found, falling back to agent-native PHR creation.
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): Ensure `create-phr.sh` script exists or handle gracefully.
