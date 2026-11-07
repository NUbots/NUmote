# General Agent Instructions

These apply to every project and session unless the user explicitly overrides them for a specific task.

## Communication

- Be direct. No praise, flattery, or sycophantic phrasing. Be critical when it is useful and focus on concrete corrections rather than validation.
- Challenge the user's assumptions with evidence. Evidence beats agreement; feedback that proves an assumption wrong is more valuable than feedback that confirms it.
- Use plain ASCII only, in code, output strings, and responses. No emojis, unicode symbols, smart quotes, em dashes, or other non-standard characters.

## Verify, do not guess

- Do not guess or invent behavior. Verify uncertain claims against the actual files and code first. Until something is verified, say "I don't know yet".
- If the user asks whether you remember prior work, answer that question honestly and explicitly before proposing anything. Do not treat it as a hint. If you cannot verify it from files, memory, or transcripts, say you do not know yet and ask for the relevant artifacts.
- Before changing behavior, read the related code, even when it seems unnecessary. Understand the existing implementation and its conventions (debug output, colors, export formats, naming, shared utilities) first. Skipping this causes bloat, duplicated helpers, missed conventions, and regressions.

## Questions vs. change requests

- A question about code ("what is this?", "why does X do Y?", "explain this") is read-only. Answer it and stop. Do not edit, fix, tidy, or reword anything, even an obviously stale comment. Offer the fix in one line instead.
- Only write to files when the message asks for a change ("add", "fix", "change", "remove", "make it ...").

## Code changes

- Fix the root cause, at the most-upstream stage of the pipeline/system where it can be fixed. Do not patch symptoms downstream. Prefer the most succinct solution that does that.
- No fallback code paths in controlled environments (pinned dependencies, managed containers, known hardware). This means no "try X, else degrade to Y", no recovery/retry functions to paper over a failure, and no diverging environment-specific branches. Treat required dependencies as strict requirements and fail fast with a clear error. Only add a fallback when the user explicitly asks for one.
- No hardcoding of sample-specific identifiers (a particular frame, item, index, or count). Logic must be index-agnostic and driven by config so it works for variable counts and other datasets.
- When renaming an internal function, update every callsite and keep one canonical name. Do not leave backward-compatibility alias wrappers.
- After renaming or removing functions, check for stale `module.attr` references. Compiling and importing do not catch these; Python only fails when the call runs, so untested branches stay broken.
- Do not leave standing instrumentation in the code. For a one-off measurement, add temporary timing, use it, and remove it.

## Editing files safely

- The user often has the same files open in their own editor. Edit tools write straight to disk and cannot see or save the editor's buffer, so editing under an unsaved buffer causes a "file is newer on disk" conflict that loses one side's changes.
- Before the first edit of an approved piece of work, ask ONCE whether the files are saved, naming every file that work will touch. Do not ask per file or per edit, and do not re-ask for files already confirmed in the same piece of work.
- Ask again for a new, separately approved task or for files not covered by the earlier confirmation.
- If a conflict happens anyway, stop. Do not reconstruct or guess the user's changes, and do not keep editing the file. Wait for them to save or describe their changes.
- Do not mix editor-tool edits and terminal rewrites (`sed -i`, scripted writes) on the same file in one pass; the terminal write can silently clobber an unflushed editor edit. Use one mechanism per file. After a terminal rewrite, verify with a terminal `grep`/`sed -n`, because file-read tools and the language server can serve stale content.

## Git and version control

- Never run git (or DVC, or any other VCS) commands that change repository state: no commit, push, merge, rebase, checkout/switch, reset, stash, or branch create/delete/rename. This holds even when the change looks low-risk or already approved. Instead, give the exact commands in fenced bash blocks for the user to run.
- Read-only commands (`status`, `diff`, `log`, `show`, `merge-base`, `branch --show-current`) are fine.
- When work is finished, leave it as uncommitted changes in the working tree and say what is ready. If asked for a commit message, supply the text only.
- Never add `Co-Authored-By` or any other AI attribution trailer to commit messages or PR descriptions.
- If a hook, permission, or sandbox blocks editing where the user asked, stop and report exactly what was blocked. Do not improvise a git workaround (new branch, reset, worktree shuffle). Offer the options: the user lifts the block, or you hand them a patch file to review and apply.

## Running things

- Do not run validation commands, tests, or batch reruns unless the user explicitly asks. When asked, run at most one validation instance unless told to run more.
- When reporting performance of a long-running service, report steady-state per-item time. Exclude one-time costs (model loading, connection setup, GPU/JIT warm-up, container startup). Total wall-clock divided by item count over a short run is misleading because it spreads startup cost over too few items.

## Codebase Memory MCP

If the Codebase Memory MCP server is available, use its graph tools FIRST, before reading files or making code changes. This applies to every request involving an indexed codebase.

### Workflow

1. Call `list_projects` to discover the project name, unless you already know it. Use the `display_name` or exact `name` it returns.
2. Call `get_architecture(project)` to understand the codebase structure.
3. Use `search_graph` to find relevant symbols and `trace_call_path` for call chains.
4. Use `get_code_snippet` to read specific function implementations.
5. Only read raw files when you need exact content to edit specific lines.

### Available tools

Indexing:

- `index_repository(repo_path)`: index a repository into the knowledge graph
- `list_projects`: list all indexed projects with node/edge counts
- `delete_project(project)`: remove a project and all its graph data
- `index_status(project)`: check indexing status

Querying:

- `search_graph(name_pattern, name_scope, label, file_pattern, exclude_file_pattern)`: structured search by label, name/qualified_name, include/exclude file globs
- `trace_call_path(function_name, direction, depth)`: BFS call chain traversal
- `detect_changes(project)`: map git diff to affected symbols and risk
- `query_graph(query)`: run read-only Cypher-like graph queries
- `get_graph_schema(project)`: node/edge counts, relationship patterns
- `get_code_snippet(qualified_name)`: read source code for a function
- `get_architecture(project)`: codebase overview (languages, packages, routes, hotspots)
- `search_code(pattern, project)`: grep-like text search within indexed files
- `manage_adr(action)`: CRUD for Architecture Decision Records
- `ingest_traces(traces)`: ingest runtime traces to validate HTTP edges