# Contributing to canopen-rs

Thanks for your interest — contributions of all sizes are welcome, from fixing a
typo to implementing a new service.

## Where to start

- Browse the [open issues](https://github.com/KarpagamKarthikeyan/canopen-rs/issues).
  Anything labelled **`good first issue`** is scoped to be approachable without
  deep knowledge of the codebase; **`help wanted`** marks things we'd love a hand
  with.
- Have an idea that isn't filed? Please **open an issue first** so we can agree on
  the approach before you write code — it saves everyone rework.
- Questions or "how would I use this for X?" — open a
  [Discussion](https://github.com/KarpagamKarthikeyan/canopen-rs/discussions);
  no need to file a formal issue.

## Developer Certificate of Origin (DCO)

This project uses the [Developer Certificate of Origin](https://developercertificate.org/)
instead of a CLA. It's a lightweight statement that you have the right to submit
your contribution under the project's license.

**Every commit must be signed off.** Add a `Signed-off-by` line with:

```bash
git commit -s -m "your message"
```

which appends, using your `git config` name and email:

```
Signed-off-by: Your Name <you@example.com>
```

By signing off you certify the DCO. PRs whose commits aren't signed off can't be
merged. To fix an existing commit: `git commit --amend -s` (or
`git rebase --signoff` for several).

## Building and testing

```bash
cargo test --workspace                                     # unit + integration + doctests
cargo clippy --workspace --all-targets --all-features -- -D warnings
cargo fmt --all --check
cargo build -p canopen-rs --target thumbv7em-none-eabihf   # confirm the core stays no_std
```

Two extra checks the CI runs that are worth running locally when relevant:

```bash
# Independent wire-format cross-check against python-canopen:
python3 -m pip install canopen && python3 tools/interop/python_canopen_oracle.py

# On-bus loopback over a virtual CAN interface (Linux):
sudo tools/vcan_setup.sh && cargo run -p canopen-host --example vcan_loopback
```

## Where code belongs

The workspace has two crates, and it matters which one your change goes in:

- **`core` (`canopen-rs`)** — `#![no_std]`, allocation-free, transport-agnostic.
  All protocol logic (object dictionary, SDO/PDO/NMT/SYNC/EMCY/LSS codecs and
  state machines) lives here. New protocol features almost always go here.
- **`host` (`canopen-host`)** — `std`. The Linux SocketCAN transport and EDS/DCF
  parsing. Anything that needs `std`, an allocator, or an OS goes here (or behind
  a Cargo feature on the core).

If you're unsure, ask in the issue — "core vs host" is the most common question.

## What a good PR looks like

- **Focused** — one logical change per PR.
- **Tested** — unit tests for logic; for any wire-format codec, assert against
  **known-good byte sequences** (see existing tests for the style), and add a
  cross-check to `tools/interop/python_canopen_oracle.py` where it applies.
- **Clean** — `cargo fmt` and `clippy -D warnings` pass; the core still builds for
  `thumbv7em-none-eabihf`.
- **Documented** — public APIs have doc comments; a runnable doctest is a plus.
- **Signed off** — DCO (`git commit -s`).

CI enforces the test/lint/fmt/no_std/MSRV/interop gates, so you'll get fast
feedback.

## License

By contributing, you agree that your work is dual-licensed under
[MIT](LICENSE-MIT) or [Apache-2.0](LICENSE-APACHE), at the user's option, matching
the rest of the project.
