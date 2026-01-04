# Implementation Notes

## Sensor Noise and Filtering
Ultrasonic sensors exhibited fluctuating readings due to surface reflectivity and angle of incidence. A median filter was selected over an averaging filter due to its robustness against outliers and spurious spikes.

Filtering multiple samples per cycle improved stability without significantly increasing latency.

---

## Sensor Fusion Strategy
Rather than relying on a single sensor, distance estimates from multiple sensors were fused using a confidence-weighted approach. Sensors reporting consistent, close-range values were assigned higher confidence to prioritize safety-critical detections.

This approach improved reliability when individual sensors temporarily failed or returned invalid readings.

---

## Timing and Real-Time Constraints
A non-blocking timing loop using `millis()` was implemented to ensure consistent update intervals while avoiding unnecessary delays. Loop timing was selected to balance responsiveness with sensor reliability.

---

## Threshold Selection
Distance thresholds were empirically tuned through repeated testing across varying distances. The selected safe distance prioritized early alerts while minimizing false positives.

---

## Limitations
- Sensor fusion is limited to distance-based sensing
- Environmental factors such as soft surfaces can reduce accuracy
- System does not yet account for object velocity

---

## Future Improvements
- Adaptive thresholds based on environmental conditions
- Statistical confidence modeling
- Integration with motor control for autonomous motion
- Data logging for extended performance analysis
