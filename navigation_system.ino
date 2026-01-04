// Autonomous Sensor Fusion Navigation System
// Author: Adheesh Ramanath
// Platform: Arduino
// Description: Real-time multi-sensor obstacle detection with confidence-based fusion

// -------------------- Pin Definitions --------------------
const int trigPins[2] = {2, 4};
const int echoPins[2] = {3, 5};
const int alertPin = 9;  // Buzzer or LED

// -------------------- System Parameters --------------------
const int safeDistance = 30;          // cm
const int maxDistance = 200;           // cm
const int samplesPerSensor = 5;        // median filter window
const unsigned long loopInterval = 50; // ms

unsigned long lastLoopTime = 0;

// -------------------- Utility Functions --------------------
long readRawDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 25000);
  long distance = duration * 0.034 / 2;

  if (distance <= 0 || distance > maxDistance) return -1;
  return distance;
}

long medianFilter(long readings[], int size) {
  // Simple bubble sort for small arrays
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

long getFilteredDistance(int sensorIndex) {
  long readings[samplesPerSensor];
  int validCount = 0;

  for (int i = 0; i < samplesPerSensor; i++) {
    long d = readRawDistance(trigPins[sensorIndex], echoPins[sensorIndex]);
    if (d > 0) {
      readings[validCount++] = d;
    }
    delayMicroseconds(200);
  }

  if (validCount < samplesPerSensor / 2) return -1;
  return medianFilter(readings, validCount);
}

// Confidence score based on consistency
float computeConfidence(long d) {
  if (d < 0) return 0.0;
  if (d <= safeDistance) return 1.0;
  if (d <= safeDistance * 2) return 0.7;
  return 0.4;
}

long fuseDistances(long d1, long d2) {
  float c1 = computeConfidence(d1);
  float c2 = computeConfidence(d2);

  if (c1 == 0 && c2 == 0) return -1;
  if (c1 == 0) return d2;
  if (c2 == 0) return d1;

  return (long)((d1 * c1 + d2 * c2) / (c1 + c2));
}

// -------------------- Setup --------------------
void setup() {
  for (int i = 0; i < 2; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }
  pinMode(alertPin, OUTPUT);

  Serial.begin(9600);
  Serial.println("Autonomous Sensor Fusion Navigation System Initialized");
}

// -------------------- Main Loop --------------------
void loop() {
  if (millis() - lastLoopTime < loopInterval) return;
  lastLoopTime = millis();

  long d1 = getFilteredDistance(0);
  long d2 = getFilteredDistance(1);
  long fused = fuseDistances(d1, d2);

  Serial.print("D1: ");
  Serial.print(d1);
  Serial.print(" cm | D2: ");
  Serial.print(d2);
  Serial.print(" cm | Fused: ");
  Serial.print(fused);
  Serial.println(" cm");

  if (fused > 0 && fused <= safeDistance) {
    digitalWrite(alertPin, HIGH);
  } else {
    digitalWrite(alertPin, LOW);
  }
}
