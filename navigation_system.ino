// Autonomous Sensor Fusion Navigation System v2.0
// Author: Adheesh Ramanath
// Platform: Arduino
// Description: Real-time multi-sensor obstacle detection with confidence-based sensor fusion, direction detection, and alert system

// -------------------- Pin Definitions --------------------
const int trigPins[2] = {2, 4};   // Ultrasonic sensor trigger pins
const int echoPins[2] = {3, 5};   // Ultrasonic sensor echo pins
const int irPins[2] = {A0, A1};   // Infrared distance sensors
const int buzzerPin = 9;          // Buzzer for alerts
const int ledPin = 10;            // LED for visual alerts

// -------------------- System Parameters --------------------
const int safeDistance = 30;          // cm, distance considered unsafe
const int maxDistance = 200;          // cm, max reliable distance
const int samplesPerSensor = 7;       // number of readings for filtering
const unsigned long loopInterval = 50; // ms, time between main loop updates
bool debugMode = true;                // print detailed info to Serial

// Timing
unsigned long lastLoopTime = 0;

// -------------------- Utility Functions --------------------

// Read raw ultrasonic distance in cm
long readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 25000); // max 25 ms timeout
  long distance = duration * 0.034 / 2;

  if (distance <= 0 || distance > maxDistance) return -1; // invalid reading
  return distance;
}

// Read raw IR sensor distance (simplified approximation)
long readIR(int pin) {
  int value = analogRead(pin);
  if (value < 50) return -1; // too low = out of range
  long distance = 6787 / (value - 3) - 4; // rough formula for Sharp IR
  if (distance > maxDistance) return -1;
  return distance;
}

// Median filter with simple bubble sort
long medianFilter(long readings[], int size) {
  for (int i = 0; i < size - 1; i++) {
    for (int j = i + 1; j < size; j++) {
      if (readings[j] < readings[i]) {
        long temp = readings[i];
        readings[i] = readings[j];
        readings[j] = temp;
      }
    }
  }
  return readings[size / 2];
}

// Standard deviation check to reject outliers
long filteredDistance(long (*readFunc)(int, int), int sensorIndex) {
  long readings[samplesPerSensor];
  int validCount = 0;

  for (int i = 0; i < samplesPerSensor; i++) {
    long d = readFunc(trigPins[sensorIndex], echoPins[sensorIndex]);
    if (d > 0) readings[validCount++] = d;
    delayMicroseconds(200);
  }

  if (validCount < samplesPerSensor / 2) return -1;

  long med = medianFilter(readings, validCount);

  // reject readings too far from median
  long filtered = 0, count = 0;
  for (int i = 0; i < validCount; i++) {
    if (abs(readings[i] - med) < 15) { // 15 cm threshold
      filtered += readings[i];
      count++;
    }
  }
  return count > 0 ? filtered / count : -1;
}

// Compute confidence based on distance and sensor type
float computeConfidence(long d) {
  if (d < 0) return 0.0;
  if (d <= safeDistance) return 1.0;       // very confident if too close
  if (d <= safeDistance * 2) return 0.7;   // moderately confident
  return 0.4;                               // low confidence
}

// Sensor fusion between multiple sensors
long fuseDistances(long d1, long d2, long d3, long d4) {
  float c1 = computeConfidence(d1);
  float c2 = computeConfidence(d2);
  float c3 = computeConfidence(d3);
  float c4 = computeConfidence(d4);

  float totalC = c1 + c2 + c3 + c4;
  if (totalC == 0) return -1;

  return (long)((d1 * c1 + d2 * c2 + d3 * c3 + d4 * c4) / totalC);
}

// Determine obstacle direction based on sensor distances
String obstacleDirection(long dLeft, long dRight) {
  if (dLeft <= safeDistance && dRight <= safeDistance) return "CENTER";
  if (dLeft <= safeDistance) return "LEFT";
  if (dRight <= safeDistance) return "RIGHT";
  return "NONE";
}

// Alert system
void triggerAlert(long fusedDistance, String direction) {
  if (fusedDistance > 0 && fusedDistance <= safeDistance) {
    digitalWrite(ledPin, HIGH);
    tone(buzzerPin, 1000); // continuous tone

    if (debugMode) {
      Serial.print("ALERT! Obstacle at ");
      Serial.print(direction);
      Serial.print(" | Distance: ");
      Serial.println(fusedDistance);
    }
  } else {
    digitalWrite(ledPin, LOW);
    noTone(buzzerPin);
  }
}

// -------------------- Setup --------------------
void setup() {
  for (int i = 0; i < 2; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }
  pinMode(irPins[0], INPUT);
  pinMode(irPins[1], INPUT);

  pinMode(buzzerPin, OUTPUT);
  pinMode(ledPin, OUTPUT);

  Serial.begin(9600);
  Serial.println("Autonomous Sensor Fusion Navigation System v2.0 Initialized");
}

// -------------------- Main Loop --------------------
void loop() {
  if (millis() - lastLoopTime < loopInterval) return;
  lastLoopTime = millis();

  // Read all sensors
  long dUltrasonicLeft = filteredDistance(readUltrasonic, 0);
  long dUltrasonicRight = filteredDistance(readUltrasonic, 1);
  long dIRLeft = readIR(irPins[0]);
  long dIRRight = readIR(irPins[1]);

  // Fuse distances
  long fusedDistance = fuseDistances(dUltrasonicLeft, dUltrasonicRight, dIRLeft, dIRRight);

  // Determine obstacle direction
  String direction = obstacleDirection(dUltrasonicLeft, dUltrasonicRight);

  // Trigger alerts
  triggerAlert(fusedDistance, direction);

  // Serial debug
  if (debugMode) {
    Serial.print("Ultrasonic L: "); Serial.print(dUltrasonicLeft); Serial.print(" cm | ");
    Serial.print("Ultrasonic R: "); Serial.print(dUltrasonicRight); Serial.print(" cm | ");
    Serial.print("IR L: "); Serial.print(dIRLeft); Serial.print(" cm | ");
    Serial.print("IR R: "); Serial.print(dIRRight); Serial.print(" cm | ");
    Serial.print("Fused: "); Serial.print(fusedDistance); Serial.print(" cm | Direction: ");
    Serial.println(direction);
  }
}
