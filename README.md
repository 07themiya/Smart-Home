### Smart Home Project with Blynk and ESP32

This project implements a **Smart Home Automation System** using an **ESP32 microcontroller**. The system features temperature monitoring, motion detection, and remote control of devices through the **Blynk app**. It integrates a PIR sensor, DHT22 temperature sensor, and an LCD display.

---

#### **Features**

1. **Temperature Monitoring**:
   - Displays the current temperature on an LCD screen.
   - Sends the temperature data to the Blynk app.

2. **Motion Detection**:
   - Detects motion using a PIR sensor.
   - Displays motion status on the LCD.
   - Activates a virtual LED in the Blynk app when motion is detected.

3. **Device Control**:
   - Remote control of lights and fans through the Blynk app.
   - Supports multiple rooms (e.g., living room, bedroom, kitchen).

4. **WiFi Connectivity**:
   - Connects to a WiFi network to enable remote access via the Blynk app.

---

#### **Hardware Components**

- **ESP32 microcontroller**
- **PIR motion sensor**
- **DHT22 temperature sensor**
- **16x2 LCD with I2C interface**
- LEDs and relays (for lights and fans)
- Jumper wires and breadboard

---

#### **Software Tools**

- **Arduino IDE** for programming the ESP32
- **Blynk App** (available on Android/iOS)
- **GitHub** for version control and project sharing
- **LinkedIn** for showcasing the project

---

#### **Code Implementation**

Here is the complete code used for this project:

```cpp
#define BLYNK_TEMPLATE_ID ""
#define BLYNK_TEMPLATE_NAME ""
#define BLYNK_AUTH_TOKEN ""

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <BlynkSimpleEsp32.h>
#include <WiFi.h>

// WiFi credentials
char ssid[] = "Wokwi-GUEST";
char pass[] = "";

// Define Pins
#define LIVING_ROOM_LIGHT 12
#define BEDROOM1_LIGHT 14
#define BEDROOM2_LIGHT 26
#define KITCHEN_LIGHT 27

#define LIVING_ROOM_FAN 2
#define BEDROOM1_FAN 4
#define BEDROOM2_FAN 16

#define PIR_SENSOR_PIN 23
#define DHT_PIN 19

// Blynk Virtual Pins
#define V_LIVINGROOM_LIGHT V0
#define V_BEDROOM1_LIGHT V1
#define V_TEMPERATURE V2
#define V_MOTION_LED V3
#define V_LIVINGROOM_FAN V4

// Initialize LCD, DHT sensor, and variables
LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C address 0x27, 16 columns, 2 rows
DHT dht(DHT_PIN, DHT22);

BlynkTimer timer;
bool motionDetected = false;

// Function to check motion and update Blynk
void checkMotion() {
  motionDetected = digitalRead(PIR_SENSOR_PIN);

  if (motionDetected) {
    for (int i = 0; i < 5; i++) {
      Blynk.virtualWrite(V_MOTION_LED, 255); // Turn on the motion LED in Blynk
      delay(200);
      Blynk.virtualWrite(V_MOTION_LED, 0); // Turn off the motion LED in Blynk
    }
  } else {
    Blynk.virtualWrite(V_MOTION_LED, 0); // Turn off the motion LED in Blynk
  }

  lcd.setCursor(0, 1);
  if (motionDetected) {
    lcd.print("Motion Detected ");
  } else {
    lcd.print("No Motion       ");
  }
}

// Function to read and send temperature data
void sendTemperatureToBlynk() {
  float temp = dht.readTemperature();
  if (!isnan(temp)) {
    // Send temperature to Blynk virtual display
    Blynk.virtualWrite(V_TEMPERATURE, temp); // Send to Blynk

    // Display temperature on LCD
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.print(temp);
    lcd.print(" C   ");
  } else {
    lcd.setCursor(0, 0);
    lcd.print("DHT Error!      ");
  }
}

// Blynk handlers for controlling lights and fans
BLYNK_WRITE(V_LIVINGROOM_LIGHT) {
  digitalWrite(LIVING_ROOM_LIGHT, param.asInt());
}
BLYNK_WRITE(V_BEDROOM1_LIGHT) {
  digitalWrite(BEDROOM1_LIGHT, param.asInt());
}
BLYNK_WRITE(V_LIVINGROOM_FAN) {
  digitalWrite(LIVING_ROOM_FAN, param.asInt());
}

void setup() {
  Serial.begin(9600);

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Smart Home");

  // Connect to WiFi
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");

  // Initialize Blynk
  Blynk.config(BLYNK_AUTH_TOKEN);
  Blynk.connect();

  // Set Pin Modes
  pinMode(LIVING_ROOM_LIGHT, OUTPUT);
  pinMode(BEDROOM1_LIGHT, OUTPUT);
  pinMode(BEDROOM2_LIGHT, OUTPUT);
  pinMode(KITCHEN_LIGHT, OUTPUT);

  pinMode(LIVING_ROOM_FAN, OUTPUT);
  pinMode(BEDROOM1_FAN, OUTPUT);
  pinMode(BEDROOM2_FAN, OUTPUT);

  pinMode(PIR_SENSOR_PIN, INPUT);

  // Timers
  timer.setInterval(2000L, sendTemperatureToBlynk); // Send temperature every 2 seconds
  timer.setInterval(500L, checkMotion);            // Check motion every 500 ms
}

void loop() {
  Blynk.run();
  timer.run();
}
```


