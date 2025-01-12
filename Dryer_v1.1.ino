#define BLYNK_TEMPLATE_ID "TMPL6d8IGo5tc"
#define BLYNK_TEMPLATE_NAME "Grain Dryer Automation"
#define BLYNK_AUTH_TOKEN "GPbPe608U5zgMAR1DlAheGQjtg_ulun6"

#include <Wire.h>
#include <RTClib.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>
#include <PID_v1.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

// WiFi credentials
char ssid[] = "Changeme";       // Replace with your WiFi SSID
char pass[] = "Changeme";      // Replace with your WiFi password

// Pin Definitions
#define ONE_WIRE_BUS 15          // DS18B20 data pin
#define RELAY_PIN 25             // Relay control pin
#define BUTTON_INC_PIN 12        // Button to increase the setpoint
#define BUTTON_DEC_PIN 14        // Button to decrease the setpoint

// DS18B20 Sensor
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// RTC
RTC_DS3231 rtc;

// LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);

// PID Variables
double Setpoint = 65.0;          // Default target temperature
double Input, Output;            // Current temperature and PID output
double Kp = 2.0, Ki = 0.1, Kd = 0.05; // PID constants
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Button debounce variables
unsigned long lastIncPress = 0;
unsigned long lastDecPress = 0;
const unsigned long debounceDelay = 200; // 200ms debounce delay

// Countdown timer variables
unsigned long countdownSeconds; // Countdown in seconds
DateTime previousTime;          // Stores the last RTC timestamp

// Relay logic constants
const int RELAY_ON = LOW;
const int RELAY_OFF = HIGH;

void setup() {
  Serial.begin(115200);

  // Initialize LCD
  lcd.begin(16, 2);
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Connecting WiFi");

  // Initialize DS18B20
  sensors.begin();

  // Initialize RTC
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    lcd.print("RTC Error");
    while (1);
  }
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, setting the time...");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // Initialize PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255); // PID output range

  // Set initial countdown (1 hour 15 minutes)
  setCountdown();

  // Configure pin modes
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUTTON_INC_PIN, INPUT_PULLUP);
  pinMode(BUTTON_DEC_PIN, INPUT_PULLUP);

  // Start WiFi connection
  WiFi.begin(ssid, pass);
}

void loop() {
  // Check WiFi connection
  if (WiFi.status() == WL_CONNECTED) {
    if (!Blynk.connected()) {
      Serial.println("Connecting to Blynk...");
      Blynk.config(BLYNK_AUTH_TOKEN);
      Blynk.connect();
    }
    Blynk.run();
  } else {
    Serial.println("WiFi disconnected. Reconnecting...");
    WiFi.reconnect();
    delay(1000);  // Delay before retrying WiFi connection
    return; // Skip the rest of the loop if WiFi is disconnected
  }

  // Update other functionalities
  updateCountdown(); // Update countdown timer

  // Request temperature from DS18B20 sensor
  sensors.requestTemperatures();
  Input = sensors.getTempCByIndex(0);

  // Check for valid temperature reading
  if (Input == DEVICE_DISCONNECTED_C) {
    Serial.println("Error: DS18B20 not connected!");
    digitalWrite(RELAY_PIN, RELAY_OFF);  // Turn OFF relay
    delay(1000);
    return;
  }

  // Compute PID output
  myPID.Compute();

  // Control relay based on temperature
  if (Input >= Setpoint) {
    digitalWrite(RELAY_PIN, RELAY_ON);
    Serial.println("Heating ON");
  } else {
    digitalWrite(RELAY_PIN, RELAY_OFF);
    Serial.println("Heating OFF");
  }

  // Handle button presses to adjust Setpoint
  handleButtons();

  // Display information on LCD and send to Blynk
  displayInfo();
}

void handleButtons() {
  if (digitalRead(BUTTON_INC_PIN) == LOW && millis() - lastIncPress > debounceDelay) {
    Setpoint += 1.0; // Increase Setpoint by 1°C
    lastIncPress = millis(); // Update debounce timer
    Serial.println("Setpoint Increased");
    if (countdownSeconds > 60) countdownSeconds -= 60;
  }

  if (digitalRead(BUTTON_DEC_PIN) == LOW && millis() - lastDecPress > debounceDelay) {
    Setpoint -= 1.0; // Decrease Setpoint by 1°C
    lastDecPress = millis(); // Update debounce timer
    Serial.println("Setpoint Decreased");
    countdownSeconds += 60;
  }
}

void setCountdown() {
  countdownSeconds = 1 * 3600 + 15 * 60; // Default to 1 hour 15 minutes
}

void updateCountdown() {
  DateTime currentTime = rtc.now();
  unsigned long elapsedSeconds = currentTime.secondstime() - previousTime.secondstime();

  if (elapsedSeconds > 0) {
    previousTime = currentTime;
    if (countdownSeconds > elapsedSeconds) {
      countdownSeconds -= elapsedSeconds;
    } else {
      countdownSeconds = 0;
    }
  }
}

void displayInfo() {
  unsigned long hours = countdownSeconds / 3600;
  unsigned long minutes = (countdownSeconds % 3600) / 60;
  unsigned long seconds = countdownSeconds % 60;

  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(Input);
  lcd.print("C   ");  // Clear trailing characters
  lcd.setCursor(0, 1);
  lcd.printf("Set: %.0fC ", Setpoint);
  lcd.printf("T:%02lu:%02lu:%02lu", hours, minutes, seconds);

  Serial.printf("Temp: %.2fC | Set: %.2fC | Timer: %02lu:%02lu:%02lu\n", 
                Input, Setpoint, hours, minutes, seconds);

  // Send data to Blynk
  Blynk.virtualWrite(V0, Input);              // Temperature
  Blynk.virtualWrite(V1, Setpoint);           // Current setpoint
  Blynk.virtualWrite(V2, String(hours) + ":" +
                           String(minutes) + ":" +
                           String(seconds));  // Countdown timer
}
 
