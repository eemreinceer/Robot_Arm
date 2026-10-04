# Security policy

Robot Arm is a prototype engineering portfolio, not a supported production or
safety-certified product. Security findings are still handled as engineering
defects, especially when they affect command validation, arming, watchdogs,
deployment units, credentials or dependency integrity.

## Reporting

Do not open a public issue containing credentials, exploit details or private
hardware/network information. Send a concise report to `inceer22@gmail.com`
with:

- affected file, component and commit;
- reproduction prerequisites and minimal steps;
- expected and observed behavior;
- impact and any known safe workaround;
- whether physical hardware or actuator power is involved.

For non-sensitive hardening suggestions, use a normal GitHub issue.

## Scope and response

The default branch is the only maintained public line; no released versions
currently receive backports. Reports will be acknowledged when reviewed, but
this repository does not promise a production support SLA.

Never reproduce a report by energizing actuators, flashing firmware or
changing wiring without an operator-controlled preflight and a reachable power
cutoff. CI and mock tests cannot establish physical safety.
