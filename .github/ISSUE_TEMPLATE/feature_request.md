---
name: Feature request
about: Propose a new capability or improvement
title: ""
labels: enhancement
assignees: ""
---

## What

A clear description of the feature or improvement.

## Why

The use case — what does this unblock, and for whom (embedded node, host master,
tooling)?

## Spec reference

If this implements part of CANopen, link the relevant CiA 301/305 section.

## Where it belongs

- [ ] `no_std` core (`canopen-rs`) — protocol logic
- [ ] host (`canopen-host`) — transport / EDS / std tooling
- [ ] not sure (happy to discuss)

## Notes on approach

Which existing module would this extend? Any API or design considerations
(especially anything affecting `no_std` / `Copy` / allocation)? What would the
tests look like?
