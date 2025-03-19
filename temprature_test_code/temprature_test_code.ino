#include <Wire.h>
#include <Adafruit_MLX90614.h>

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

unsigned long previousMillis = 0;
const long interval = 2000;  // Reading interval in milliseconds (2 seconds)

void setup() {
  Serial.begin(9600);
  
  // Initialize the MLX90614 sensor
  if (!mlx.begin()) {
    Serial.println("Error connecting to MLX sensor. Check wiring.");
    while (1);
  }
  
  Serial.println("MLX90614 IR Temperature Sensor");
  Serial.println("------------------------------");
  Serial.println("Time(ms), Ambient Temp(C), Object Temp(C)");
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Check if it's time to take a reading
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    
    // Read ambient (room) and object temperatures
    float ambientTemp = mlx.readAmbientTempC();
    float objectTemp = mlx.readObjectTempC();
    
    // Print time and temperatures to serial monitor
    Serial.print(currentMillis);
    Serial.print(", ");
    Serial.print(ambientTemp);
    Serial.print(", ");
    Serial.println(objectTemp);
  }
}
