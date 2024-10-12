# lib3818-rs

## Notes

If receiving an error message, `` `called `Result::unwrap()`on an`Err`value: MissingBinutils` ``:

```bash
rustup component add llvm-tools
cargo install cargo-binutils
```

If `rust-analyzer` shows errors for `not a supported ABI for the current target`:

- delete target wasm32-unknown-unknown in `.cargo/config.toml`.
- delete wasm32-unknown-unknown from `"rust-analyzer.check.targets"` in `.vscode/settings.json`.

the2nake
