# lib3818-rs

A modular library for v5rc using vexide.

## Features

- Tank drive control
- Tank drive physical model

## Planned

- [ ] 2d motion profile
  - [ ] 2nd order limited: trapezoid
  - [ ] 3rd order limited: s-curve
- [ ] inverse kinematics
  - [ ] path following
  - [ ] constant angle joints
  - [ ] variable angle joints

## Notes

If receiving an error message, `` `called `Result::unwrap()`on an`Err`value: MissingBinutils` ``:

```bash
rustup component add llvm-tools
cargo install cargo-binutils
```

If `rust-analyzer` shows errors for `not a supported ABI for the current target`:

- delete target wasm32-unknown-unknown in `.cargo/config.toml`.
- delete wasm32-unknown-unknown from `"rust-analyzer.check.targets"` in `.vscode/settings.json`.

## Contributors

the2nake
