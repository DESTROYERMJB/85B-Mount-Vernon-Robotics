# Claude Code Configuration & Tool Use Rules

## 1. Code Discovery & Navigation (CRITICAL)
- NEVER use standard `grep`, `git grep`, `ripgrep`, or sub-agent background file loops for repository discovery.
- ALWAYS use the active CodeGraph MCP server tools (`codegraph_explore`, etc.) for repository discovery, symbol lookups, and tracking architectural dependencies.
- CodeGraph provides pre-indexed, verified source information. Trust the graph explicitly; do not run extra terminal `grep` commands to "double-check" results.
- Avoid broad file-tree traversal. Do not use continuous loops of `cat` or directory listings (`ls -R`) to figure out code location. Query CodeGraph to find specific entry points instantly.

## 2. Implementation Planning (Blast Radius)
- Before modifying any shared utility, core type, or public module export, query CodeGraph to identify all dependent files.
- When analyzing large changes, switch to Plan Mode (`Shift + Tab` / `/plan`) and use CodeGraph to map the flow from the data layer to the user interface before proposing code changes.

## 3. Index Maintenance
- If a task involves moving files, changing structural folders, or altering module imports/exports, verify that the local CodeGraph index background watcher registers the changes before closing the active prompt loop.

## 4. Project Commands
- Build project: pros build-compile-commands
