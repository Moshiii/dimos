# Pi code-policy adapter

This source-checkout package runs the pinned Pi session used by `dimos eval run`.
It disables Pi built-in tools and exposes exactly one host-brokered tool,
`python_exec`.

```bash
npm ci --prefix packages/pi-code-policy-adapter
npm test --prefix packages/pi-code-policy-adapter
npm run build --prefix packages/pi-code-policy-adapter
```

The compiled entrypoint is `dist/code-policy-main.js`. Standard output is
reserved for newline-delimited protocol frames; diagnostics use standard error.
Credentials are supplied by the Python parent process through the supported
environment binding and must never be placed in command-line arguments.
