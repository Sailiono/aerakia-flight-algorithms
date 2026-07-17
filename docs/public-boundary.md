# Public repository boundary

## Included

- original portable algorithm implementations;
- public measurement/output contracts;
- host-side tests, simulation, replay, metrics, and reports;
- mathematical conventions and references;
- sanitized recorded datasets when publication is approved.

## Kept private

- CubeMX and IDE projects;
- MCU startup, HAL, CMSIS, RTOS, networking, USB, and filesystems;
- sensor/actuator drivers, register configuration, buses, pins, and board identity;
- private publish/subscribe topics and binary HIL transports;
- calibration assets tied to a product or serial number;
- control laws, actuators, safety logic, bootloaders, deployment, and manufacturing code;
- internal logs and machine-specific build/test scripts.

## Admission rule

A source file belongs here only when it can be built or exercised on a normal PC without Aerakia hardware. Third-party code additionally requires documented provenance and a compatible license. Mathematical reimplementations cite the original publication and must not copy code of uncertain origin.
