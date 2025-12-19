# Research: Word Counting Methodology

**Date**: 2025-12-19
**Feature**: [Update Book Word Count](spec.md)

## Topic: Reliable Word Counting for Markdown

The technical context identified a need for a clear method to count words to track progress towards the 80,000-word goal.

### Decision: Use the `word-count` NPM package

A Node.js script will be created that uses the `word-count` package to recursively scan the `physical-ai-humanoid-robotics-book/docs` directory, read all `.md` and `.mdx` files, and sum the word counts.

### Rationale

- **Standard Tooling**: `word-count` is a well-established, simple, and lightweight package in the Node.js ecosystem.
- **Accuracy**: It provides a reliable word count for plain text content, which is sufficient for this project's needs.
- **Integration**: It can be easily integrated into a simple script, allowing for consistent and repeatable measurements.
- **Speed**: The process is fast and can be run on demand without a complex setup.

### Alternatives Considered

- **`markdown-word-count`**: Another potential package. Rejected because `word-count` is more generic and has wider adoption, making it a safer dependency.
- **Custom Regex Script**: Considered writing a custom script to count words. Rejected as being error-prone (e.g., handling punctuation, special characters) and an unnecessary reinvention of existing, reliable tools.