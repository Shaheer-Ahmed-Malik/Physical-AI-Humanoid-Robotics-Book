# Data Model: Book Content

**Date**: 2025-12-19
**Feature**: [Update Book Word Count](spec.md)

This feature does not introduce or modify any data models. It only expands the content within the existing structure.

## Existing Content Structure

The book's content is modeled directly on the file system within the `physical-ai-humanoid-robotics-book/docs/` directory.

- **Entity: Chapter/Section**
  - **Representation**: A Markdown file (`.md` or `.mdx`) or a directory.
  - **Hierarchy**: The folder structure within `docs/` defines the nesting of chapters and sub-sections. Docusaurus uses `_category_.json` files to manage sidebar labels and positions for directories.

- **Entity: Content**
  - **Representation**: The Markdown text inside the content files.
  - **Fields**: The content itself is unstructured text from the perspective of this feature.

No changes are required to this implicit data model.
