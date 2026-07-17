# Public repository boundary

## Included

- original portable algorithm implementations;
- public measurement/output contracts;
- host-side tests, simulation, replay, metrics, and reports;
- mathematical conventions and references;
- sanitized replay datasets when publication is explicitly approved.

## Kept private

- CubeMX and IDE projects;
- MCU startup, HAL, CMSIS, RTOS, networking, USB, and filesystems;
- sensor/actuator drivers, register configuration, buses, pins, and board identity;
- private publish/subscribe topics and binary HIL transports;
- calibration assets tied to a product or serial number;
- control laws, actuators, safety logic, bootloaders, deployment, and manufacturing code;
- internal logs and machine-specific build/test scripts.

The public ULog converter is allowed because it exports only the hardware-neutral contract. Raw ULogs, absolute GNSS coordinates, identifiers, private manifests, and generated per-flight reports remain private by default.

## Admission rule

A source file belongs here only when it can be built or exercised on a normal PC without Aerakia hardware. Third-party code additionally requires documented provenance and a compatible license. Mathematical reimplementations cite the original publication and must not copy code of uncertain origin.

## Ownership rule

This repository is the sole source of truth for Aerakia's public flight algorithms. The private flight stack may consume a pinned revision through an adapter, but must not develop or maintain a second editable copy. Any algorithm correction is made and tested here first, then integrated into the private stack by updating the pinned revision.

The temporary legacy copies in the private stack are migration-only artifacts: their hashes are locked, they must not receive algorithm changes, and they are removed after the FCOne adapter has switched to this repository.

`python validation/check_public_boundary.py` enforces the most important structural rules in CI. It rejects common MCU project trees and artifacts, raw ULogs, startup files, and direct platform/driver includes from the public algorithm sources.
