(coding-agent-documentation)=

# Writing DimOS Documentation

DimOS documentation is authored as MyST Markdown in `docs/` and built with
Sphinx. Add user-facing guides under `docs/usage` or
`docs/capabilities` and contributor-only material under
`docs/development`. Include each new page in the nearest `toctree`.

Use Sphinx roles for references so renamed pages and Python symbols are
checked during the build:

```md
See {doc}`/usage/configuration` for configuration precedence.
Streams use {class}`dimos.core.stream.In` and
{class}`dimos.core.stream.Out`.
Jump to {ref}`a-stable-section-label`.
```

Prefer checked-in examples with `literalinclude` over copied snippets.
See {doc}`codeblocks` for code examples and {doc}`doclinks` for links.

Before submitting a documentation change, run:

```bash
uv sync --only-group docs
uv run make -C docs html
uv run make -C docs spelling
```

Both commands treat warnings as errors in CI.

```{toctree}
:hidden: true
:maxdepth: 1

codeblocks
doclinks
```
