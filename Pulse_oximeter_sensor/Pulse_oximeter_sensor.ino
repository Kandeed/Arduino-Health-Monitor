#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

MAX30105 particleSensor;

// Buffer to store heart rate and SpO2 values
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;

// Variables for heart rate and SpO2 calculation
float beatsPerMinute;
int beatAvg;
long irValue;
int spO2;

void setup() {
  Serial.begin(115200);
  
  // Initialize the sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 was not found. Please check wiring/power.");
    while (1);
  }
  
  // Configure sensor settings
  particleSensor.setup(); // Use default settings
  particleSensor.setPulseAmplitudeRed(0x0A); // Lowest power mode
  particleSensor.setPulseAmplitudeGreen(0); // Turn off Green LED
}

void loop() {
  irValue = particleSensor.getIR();

  if (detectBeat(irValue) == true) {
    // We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20) {
      rates[rateSpot++] = (byte)beatsPerMinute;
      rateSpot %= RATE_SIZE;

      // Calculate average
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }

  // Calculate SpO2 (simplified method)
  if (irValue > 50000) {
    spO2 = estimateSpO2(irValue);
  }

  // Print results
  Serial.print("IR=");
  Serial.print(irValue);
  Serial.print(", BPM=");
  Serial.print(beatsPerMinute);
  Serial.print(", Avg BPM=");
  Serial.print(beatAvg);
  Serial.print(", SpO2=");
  Serial.print(spO2);
  Serial.println("%");

  delay(100);
}

// Renamed function to avoid conflict
bool detectBeat(long irValue) {
  static long lastPeak = 0;
  static int threshold = 2000;
  
  if (irValue > threshold) {
    if (millis() - lastPeak > 300) { // Debounce
      lastPeak = millis();
      return true;
    }
  }
  return false;
}

// Renamed function to avoid conflict
int estimateSpO2(long irValue) {
  // This is a very basic estimation and should not be used for medical diagnostics
  int estimatedSpO2 = map(irValue, 50000, 100000, 95, 100);
  estimatedSpO2 = constrain(estimatedSpO2, 90, 100);
  return estimatedSpO2;
}