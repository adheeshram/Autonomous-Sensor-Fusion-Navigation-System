# Architecture and design decisions

## Processing pipeline

Each 50 ms cycle reads two ultrasonic and two infrared channels. A channel owns its filter history and health score; invalid readings never enter the filter window.

1. **Validate** — reject timeouts and measurements outside the configured 4–200 cm operating range.
2. **Filter** — compute the rolling median, reject a new sample outside `max(12 cm, 3 × robust sigma)`, then smooth accepted medians with an exponential filter.
3. **Assess health** — a valid sample moves health toward 1.0; a timeout or rejected spike reduces it. This makes intermittent channels contribute less before they are removed entirely.
4. **Fuse by sector** — left and right are computed independently from modality reliability, health, and sample freshness. The navigation range is the nearer valid sector.
5. **Estimate motion** — the rate of change of fused range produces a low-pass closing-speed estimate and TTC.
6. **Decide** — distance, TTC, and confidence drive a state machine. Escalation is immediate; recovery uses consecutive observations and a release margin.

## Why sector fusion

A single weighted mean across all four channels can hide a close obstacle. For example, averaging a 20 cm left obstacle with three 150 cm measurements produces a deceptively safe result. This implementation first estimates each sector and then selects the smaller range, retaining sensor redundancy without losing spatial risk.

## Deterministic embedded behavior

- Fixed-size arrays; no heap allocation in the navigation core
- Bounded seven-element insertion sort for robust statistics
- Explicit stale-data deadline
- Fixed 20 Hz acquisition target
- Hardware-independent core types for native testing
- Structured serial records instead of free-form debug strings

## Failure behavior

| Failure | Response |
|---|---|
| One sensor times out | Drop its health; continue with the paired modality |
| Repeated spikes | Reject samples and progressively down-weight the channel |
| One complete sector is unavailable | Continue at reduced confidence using the remaining sector |
| All measurements invalid/stale | Enter `DEGRADED`, cap speed recommendation, enable fault alert |
| Threshold oscillation | Hold the more severe state until the recovery count and margin are met |

## Extension points

`RawMeasurement` separates the estimator from the physical bus. A time-of-flight sensor, lidar, CAN range packet, or simulated input can be added by mapping it into the same type. A future motor controller should consume `NavigationDecision` rather than reading sensors directly, keeping safety policy in one place.
