# Hardware integration

## Default pin map

| Device | Signal | Arduino Uno pin |
|---|---|---|
| Left ultrasonic | Trigger / Echo | D2 / D3 |
| Right ultrasonic | Trigger / Echo | D4 / D5 |
| Left infrared | Analog output | A0 |
| Right infrared | Analog output | A1 |
| Buzzer | Signal | D9 |
| Status LED | Anode through resistor | D10 |

All devices must share ground. Check the chosen board's input-voltage limits; use a divider or level shifter when a sensor echo voltage exceeds them. Do not power high-current peripherals from a microcontroller regulator without verifying its budget.

## Sensor placement

Mount each ultrasonic/IR pair so their fields of view overlap within a left or right sector. Keep ultrasonic transducers far enough apart to reduce acoustic cross-talk. The firmware triggers the ultrasonic channels sequentially, but mechanical placement still matters.

## Calibration

The ultrasonic conversion uses the nominal speed of sound. Temperature compensation can be added if the operating range demands it. The IR conversion is only a generic inverse-distance approximation:

```text
distance_cm = 6787 / (ADC - 3) - 4
```

Build a model-specific curve by recording ADC output at known distances, fitting the manufacturer's recommended relationship, and replacing `readInfraredCm`. Validate every channel independently before evaluating fusion behavior.

## Output integration

`NavigationDecision::speedLimitPercent` is a policy output, not a motor PWM value. A motor-control layer should combine it with its own closed-loop speed command and watchdog. The LED/buzzer behavior in `src/main.cpp` is a demonstrator for the state machine.
