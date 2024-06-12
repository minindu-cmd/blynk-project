#define BLYNK_TEMPLATE_ID ""
#define BLYNK_TEMPLATE_NAME "Vehicle Sensor System"
#define BLYNK_AUTH_TOKEN ""

// The ESP32 pin connected to the temperature sensor
#define TEMP_PIN 4

// ULTRASONIC SENSOR 1 (OIL LEVEL)
#define TRIG_PIN_1 5  // ESP32 pin GPIO5 connected to Ultrasonic Sensor's TRIG pin
#define ECHO_PIN_1 18 // ESP32 pin GPIO18 connected to Ultrasonic Sensor's ECHO pin

// ULTRASONIC SENSOR 2 (WATER LEVEL)
#define TRIG_PIN_2 22 // ESP32 pin GPIO22 connected to Ultrasonic Sensor's TRIG pin
#define ECHO_PIN_2 23 // ESP32 pin GPIO23 connected to Ultrasonic Sensor's ECHO pin

#define ANALOG_IN_PIN 32 // Voltage Sensor Pin

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

char auth[] = ""; // Get this from the Blynk App
char ssid[] = ""; // Your WiFi SSID
char pass[] = ""; // Your WiFi password

float duration_us1, duration_us2, oilLevel, waterLevel;

float adc_voltage = 0.0; // Floats for ADC voltage & Input voltage
float in_voltage = 0.0;
float R1 = 30000.0; // Floats for resistor values in divider (in ohms)
float R2 = 7500.0;
float ref_voltage = 3.3; // Corrected Reference Voltage for ESP32 (0-3.3V)
int adc_value = 0;       // Integer for ADC value

void setup()
{
  Serial.begin(115200);
  Blynk.begin(auth, ssid, pass);

  // Configure the TRIGGER and ECHO pin to output mode
  pinMode(TRIG_PIN_1, OUTPUT);
  pinMode(ECHO_PIN_1, INPUT);

  pinMode(TRIG_PIN_2, OUTPUT);
  pinMode(ECHO_PIN_2, INPUT);
}

void loop()
{
  // TEMPERATURE
  int tempRaw = analogRead(TEMP_PIN);
  // Convert raw analog value to temperature (depends on sensor specifics)
  // Assuming TMP36 sensor: Temp in Celsius = (analogValue * 3.3 / 4095.0 - 0.5) * 100.0
  float temp = (tempRaw * 3.3 / 4095.0 - 0.5) * 100.0;

  // OIL LEVEL
  digitalWrite(TRIG_PIN_1, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN_1, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN_1, LOW);
  duration_us1 = pulseIn(ECHO_PIN_1, HIGH); // Measure duration of pulse from ECHO pin
  oilLevel = (duration_us1 / 2.0) * 0.0343; // Calculate distance in cm

  // WATER LEVEL
  digitalWrite(TRIG_PIN_2, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN_2, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN_2, LOW);
  duration_us2 = pulseIn(ECHO_PIN_2, HIGH);   // Measure duration of pulse from ECHO pin
  waterLevel = (duration_us2 / 2.0) * 0.0343; // Calculate distance in cm

  // BATTERY VOLTAGE
  adc_value = analogRead(ANALOG_IN_PIN);            // Read the Analog Input
  adc_voltage = (adc_value * ref_voltage) / 4095.0; // Determine voltage at ADC input
  in_voltage = adc_voltage / (R2 / (R1 + R2));      // Calculate voltage at divider input

  // Print distances and sensor values to the serial monitor
  Serial.print("Oil Level: ");
  Serial.print(oilLevel);
  Serial.println(" cm");

  Serial.print("Water Level: ");
  Serial.print(waterLevel);
  Serial.println(" cm");

  Serial.print("Input Voltage = ");
  Serial.println(in_voltage, 2);

  // Send sensor values to Blynk app
  Blynk.virtualWrite(V0, temp);       // Send temperature to Blynk app
  Blynk.virtualWrite(V1, oilLevel);   // Send oil level to Blynk app
  Blynk.virtualWrite(V2, waterLevel); // Send water level to Blynk app
  Blynk.virtualWrite(V3, in_voltage); // Send input voltage to Blynk app

  Blynk.run();
  delay(500); // Adjust delay as needed
}