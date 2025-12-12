# Quickstart Guide

This guide provides instructions on how to set up the Docusaurus environment and build the "Physical AI & Humanoid Robotics" book locally.

## Prerequisites

- [Node.js](https://nodejs.org/en/) version 18.0 or higher
- [Yarn](https://yarnpkg.com/) or [npm](https://www.npmjs.com/)

## Installation

1.  **Clone the repository:**
    ```bash
    git clone https://github.com/<your-username>/humanoid-ai-book.git
    cd humanoid-ai-book
    ```

2.  **Install dependencies:**
    ```bash
    yarn install
    # or
    npm install
    ```

## Running the Development Server

1.  **Start the development server:**
    ```bash
    yarn start
    # or
    npm start
    ```

2.  **Open your browser:**
    Open your web browser and navigate to `http://localhost:3000`. The site will automatically reload when you make changes to the source files.

## Building the Site

1.  **Build the static site:**
    ```bash
    yarn build
    # or
    npm run build
    ```
    The static files will be generated in the `build` directory.

## Deployment

The site is automatically deployed to GitHub Pages when changes are pushed to the `main` branch.
