#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <Adafruit_MLX90614.h>
#include <WiFiS3.h>
#include <ArduinoHttpClient.h>

// WiFi credentials - REPLACE WITH YOUR ACTUAL VALUES
const char* ssid = "KAND33D";
const char* password = "kandeed33";

// ThingSpeak settings - REPLACE WITH YOUR ACTUAL VALUES
const char* thingSpeakServer = "api.thingspeak.com";
const String writeAPIKey = "Z3L428HM4K3D2YR6";

WiFiClient wifiClient;
HttpClient httpClient(wifiClient, thingSpeakServer, 80);

MAX30105 particleSensor;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

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

// Variable for temperature readings
float ambientTemp;
float objectTemp;

// Timer for ThingSpeak updates (ThingSpeak has a 15 second update limit)
unsigned long lastThingSpeakUpdate = 0;
const unsigned long updateInterval = 15000;  // 15 seconds

void setup() {
  Serial.begin(115200);
  
  // Initialize MAX30102 Pulse Oximeter
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 was not found. Please check wiring/power.");
    while (1);
  }
  
  // Initialize MLX90614 Temperature Sensor
  if (!mlx.begin()) {
    Serial.println("MLX90614 was not found. Please check wiring/power.");
    while (1);
  }
  
  // Configure pulse oximeter settings
  particleSensor.setup(); // Use default settings
  particleSensor.setPulseAmplitudeRed(0x0A); // Lowest power mode
  particleSensor.setPulseAmplitudeGreen(0); // Turn off Green LED
  
  // Connect to WiFi
  connectWiFi();
  
  Serial.println("Health Monitor with ThingSpeak initialized!");
}

void loop() {
  // Read data from sensors
  readPulseOximeter();
  readTemperature();
  
  // Print results to Serial Monitor
  printSensorData();
  
  // Send data to ThingSpeak at regular intervals
  if (millis() - lastThingSpeakUpdate > updateInterval) {
    sendToThingSpeak();
    lastThingSpeakUpdate = millis();
  }
  
  delay(100);
}

void connectWiFi() {
  Serial.print("Connecting to WiFi");
  
  // Connect to WPA/WPA2 network
  WiFi.begin(ssid, password);
  
  // Wait for connection (with timeout)
  int connectionAttempts = 0;
  while (WiFi.status() != WL_CONNECTED && connectionAttempts < 20) {
    delay(500);
    Serial.print(".");
    connectionAttempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("Connected to WiFi. IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println();
    Serial.println("WiFi connection failed. Check credentials or try again.");
  }
}

void readPulseOximeter() {
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
  } else {
    // If no finger is detected
    spO2 = 0;
  }
}

void readTemperature() {
  ambientTemp = mlx.readAmbientTempC();
  objectTemp = mlx.readObjectTempC();
}

void printSensorData() {
  Serial.print("IR=");
  Serial.print(irValue);
  Serial.print(", BPM=");
  Serial.print(beatsPerMinute);
  Serial.print(", Avg BPM=");
  Serial.print(beatAvg);
  Serial.print(", SpO2=");
  Serial.print(spO2);
  Serial.print("%, Ambient Temp=");
  Serial.print(ambientTemp);
  Serial.print("°C, Object Temp=");
  Serial.print(5 + objectTemp);
  Serial.println("°C");
}

void sendToThingSpeak() {
  // Check WiFi connection
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected. Reconnecting...");
    connectWiFi();
    if (WiFi.status() != WL_CONNECTED) {
      return; // Skip this update if reconnection failed
    }
  }
  
  // Prepare the data string
  String dataString = "api_key=" + writeAPIKey;
  dataString += "&field1=" + String(beatAvg);
  dataString += "&field2=" + String(spO2);
  dataString += "&field3=" + String(objectTemp);
  dataString += "&field4=" + String(ambientTemp);
  
  // Send the HTTP POST request
  httpClient.beginRequest();
  httpClient.post("/update");
  httpClient.sendHeader("Content-Type", "application/x-www-form-urlencoded");
  httpClient.sendHeader("Content-Length", dataString.length());
  httpClient.beginBody();
  httpClient.print(dataString);
  httpClient.endRequest();
  
  // Read the status code and body of the response
  int statusCode = httpClient.responseStatusCode();
  String response = httpClient.responseBody();
  
  if (statusCode == 200) {
    Serial.println("Data sent to ThingSpeak successfully!");
  } else {
    Serial.print("Problem sending to ThingSpeak. HTTP error code: ");
    Serial.println(statusCode);
    Serial.print("Response: ");
    Serial.println(response);
  }
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

// Simplified SpO2 estimation
int estimateSpO2(long irValue) {
  // This is a very basic estimation and should not be used for medical diagnostics
  int estimatedSpO2 = map(irValue, 50000, 100000, 95, 100);
  estimatedSpO2 = constrain(estimatedSpO2, 90, 100);
  return estimatedSpO2;
}