# Security policy

## Reporting a vulnerability

Do not open a public issue for a suspected vulnerability. Use GitHub's private
vulnerability reporting for this repository. Include affected versions,
reproduction steps, impact, and any suggested mitigation.

## Deployment boundary

Forge Controls provides command-line Dora controller nodes. It performs no
authentication, and its controllers command physical actuators.

- Run controller nodes only on trusted hosts within an isolated robot network.
- Treat controller YAML configuration as safety-relevant input; validate joint
  limits and velocities before deployment to hardware.
- These controllers do not provide hard real-time motor control. Hardware
  firmware remains responsible for the low-level servo loop and safety limits.
- Do not expose controller processes or their configuration to untrusted
  networks.
- Validate safety controls independently before connecting a physical robot,
  and keep an operator-accessible emergency stop available.

Security fixes are supported on the latest released version.
