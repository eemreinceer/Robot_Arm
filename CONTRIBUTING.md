# Contributing

This repository combines robotics software, firmware and physical-system
evidence. A change is acceptable only when its behavior, evidence class,
safety boundary and reproduction path are clear.

## Before changing code

1. Confirm the target branch is current and the working tree is clean.
2. Search existing issues for the same problem.
3. Read the affected package's `package.xml`, build file, README and tests.
4. If the change touches physical motion, servo power, UART or firmware, read
   the relevant safety and bring-up documents first.

## Change discipline

- Keep the scope small; avoid unrelated refactors and dependencies.
- Evaluate ROS 2 interface changes with every publisher, subscriber, service
  or action consumer.
- Do not treat simulation, mock tests, hardware integration and physical
  measurements as interchangeable evidence.
- Record the command, environment, version, date and artifact path for a
  measurement claim.
- Never commit secrets, credentials, private network addresses, local user
  names or machine-specific absolute paths.
- Keep generated model weights local unless their provenance, dataset rights
  and redistribution terms are documented.

## Verification

Run the smallest shared gate:

```bash
./scripts/verify_workspace.sh --quick
```

For ROS 2 packages or native firmware changes:

```bash
./scripts/verify_workspace.sh --full
```

The web console also requires:

```bash
cd src/arm_perception/web/robot-arm-console
npm ci
npm run build
npm audit --audit-level=high
```

Run narrower tests for the changed package. Report failures and physical steps
that could not be executed instead of hiding them.

## Commits and pull requests

- Prefer small, logically complete commits.
- Describe the outcome in the commit message, not only the filename.
- Avoid force-pushing shared work.
- A pull request should state the problem, solution, risk, verification result
  and rollback approach.

## Simulation and physical safety

- Start simulation only through `./start_simulation.sh <mode>`.
- Run one simulator instance at a time and stop all child processes afterward.
- A success log is not proof of simulation state; compare pose data with the
  rendered scene.
- Physical motion, servo-rail power, firmware flashing and wiring changes
  require an operator-controlled preflight and a reachable power cutoff.
- If `/joint_states` is an open-loop command echo, it is not measured position.

## Definition of done

A contribution is complete when it satisfies its acceptance criteria, passes
the relevant tests, has a clean `git diff --check`, updates documentation and
states physical-evidence limitations accurately.
