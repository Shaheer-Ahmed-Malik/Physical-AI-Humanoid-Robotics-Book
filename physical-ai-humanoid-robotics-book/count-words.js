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
