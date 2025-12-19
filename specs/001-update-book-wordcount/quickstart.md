# Quickstart: Measuring Book Word Count

**Date**: 2025-12-19
**Feature**: [Update Book Word Count](spec.md)

This guide provides instructions to set up and run a script that calculates the total word count for the book.

## Setup and Execution

1.  **Navigate to the Book Directory**:
    Open a terminal and change into the book's root directory.
    ```sh
    cd physical-ai-humanoid-robotics-book
    ```

2.  **Install Dependency**:
    Install the `word-count` package from npm.
    ```sh
    npm install word-count
    ```

3.  **Create Word Count Script**:
    Create a new file in this directory named `count-words.js` and add the following code:

    ```javascript
    const fs = require('fs');
    const path = require('path');
    const wordCount = require('word-count');

    const docsDir = './docs';
    let totalWords = 0;

    function countWordsInDir(directory) {
      const files = fs.readdirSync(directory);
      for (const file of files) {
        const fullPath = path.join(directory, file);
        const stat = fs.statSync(fullPath);
        if (stat.isDirectory()) {
          countWordsInDir(fullPath);
        } else if (fullPath.endsWith('.md') || fullPath.endsWith('.mdx')) {
          const content = fs.readFileSync(fullPath, 'utf8');
          const count = wordCount(content);
          totalWords += count;
          console.log(`${fullPath}: ${count} words`);
        }
      }
    }

    try {
      countWordsInDir(docsDir);
      console.log('----------------------------------');
      console.log(`Total word count: ${totalWords}`);
    } catch (error) {
      console.error('Error counting words:', error);
    }
    ```

4.  **Run the Script**:
    Execute the script using Node.js to see the total word count.
    ```sh
    node count-words.js
    ```