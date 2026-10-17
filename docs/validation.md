# Validation protocol

Use this protocol when producing new performance claims. Preserve the raw JSONL log, environment notes, firmware commit, and summarized CSV.

## Static range accuracy

1. Place a flat target at 20, 30, 50, 75, 100, and 150 cm.
2. Collect at least 20 seconds per channel and distance.
3. Report median absolute error and 95th-percentile absolute error by modality.

## Outlier and dropout tolerance

1. Introduce brief occlusions to one sensor while its paired sensor remains visible.
2. Record time to health degradation, fused-range continuity, and state transitions.
3. Disconnect each channel individually, then each complete sector, and verify degraded behavior matches the failure table.

## Dynamic approach

1. Move a target toward the rig at repeatable slow and fast rates.
2. Compare logged closing speed and TTC against video timestamps or a calibrated motion reference.
3. Verify that TTC can trigger caution/emergency before the static range boundary.

## Boundary stability

Move a target repeatedly across the 25 cm and 60 cm boundaries. Confirm escalation is immediate and recovery does not occur until three consecutive readings exceed the configured release boundary.

## Timing

Use `t_ms` deltas from telemetry to report median and 95th-percentile cycle intervals. Run the test with both normal measurements and ultrasonic timeouts, since `pulseIn` timeout behavior is the worst-case acquisition path.
