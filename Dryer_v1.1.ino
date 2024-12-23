#include <Wire.h>
#include <RTClib.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>

// Pin Definitions
#define ONE_WIRE_BUS 15      // DS18B20 data pin
#define RELAY_PIN 2          // Relay control pin
#define BUTTON_INC_PIN 12    // Button to increase the setpoint
#define BUTTON_DEC_PIN 14    // Button to decrease the setpoint

// DS18B20 Sensor
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// RTC
RTC_DS3231 rtc;

// PID Variables
double Setpoint = 100.0;    // Default target temperature (100°C)
double Input, Output;       // Current temperature and PID output
double Kp = 2.0, Ki = 5.0, Kd = 1.0; // PID constants
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Button debounce variables
unsigned long lastIncPress = 0;
unsigned long lastDecPress = 0;
const unsigned long debounceDelay = 200; // 200ms debounce delay

// Countdown timer variables
unsigned long countdownSeconds; // Countdown in seconds
unsigned long lastCountdownUpdate = 0;

void setup() {
  Serial.begin(115200);

  // Initialize DS18B20
  sensors.begin();

  // Configure pin modes
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUTTON_INC_PIN, INPUT_PULLUP);
  pinMode(BUTTON_DEC_PIN, INPUT_PULLUP);

  // Initialize PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255); // PID output range for control

  // Initialize RTC
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, setting the time...");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // Set initial countdown (1 hour 15 minutes) and Setpoint (100°C)
  setCountdown();

  // Ensure initial state
  digitalWrite(RELAY_PIN, LOW);

  Serial.println("PID Temperature Controller Initialized");
}

void loop() {
  // Request temperature from DS18B20 sensor
  sensors.requestTemperatures();
  Input = sensors.getTempCByIndex(0);

  // Check for valid temperature reading
  if (Input == DEVICE_DISCONNECTED_C) {
    Serial.println("Error: DS18B20 not connected!");
    digitalWrite(RELAY_PIN, LOW);  // Turn OFF relay
    delay(1000);
    return;
  }

  // Control heating based on Input temperature ; Change this value later
  if (Input < 55.0) {
    digitalWrite(RELAY_PIN, HIGH); // Turn ON relay
    Serial.println("Heating ON");
  } else {
    digitalWrite(RELAY_PIN, LOW); // Turn OFF relay
    Serial.println("Heating OFF");
  }

  // Compute PID output (optional for other logic beyond relay control)
  myPID.Compute();

  // Handle button presses to adjust Setpoint
  handleButtons();

  // Update countdown timer
  updateCountdown();

  // Print information to Serial Monitor
  Serial.print("Current Temperature: ");
  Serial.println(Input);
  Serial.print("Setpoint: ");
  Serial.println(Setpoint);
  Serial.print("PID Output: ");
  Serial.println(Output);
  printCountdown();

  delay(100); // Delay for stability
}

void handleButtons() {
  // Increase Setpoint
  if (digitalRead(BUTTON_INC_PIN) == LOW && millis() - lastIncPress > debounceDelay) {
    Setpoint += 1.0; // Increase Setpoint by 1°C
    lastIncPress = millis(); // Update debounce timer
    Serial.println("Setpoint Increased");

    // Decrease countdown by 1 minute (60 seconds)
    if (countdownSeconds > 60) {
      countdownSeconds -= 60;
    }
  }

  // Decrease Setpoint
  if (digitalRead(BUTTON_DEC_PIN) == LOW && millis() - lastDecPress > debounceDelay) {
    Setpoint -= 1.0; // Decrease Setpoint by 1°C
    lastDecPress = millis(); // Update debounce timer
    Serial.println("Setpoint Decreased");

    // Increase countdown by 1 minute (60 seconds)
    countdownSeconds += 60;
  }
}

void setCountdown() {
  // Set the countdown to 1 hour 15 minutes (4500 seconds) if Setpoint is 100°C
  if (Setpoint == 100.0) {
    countdownSeconds = 1 * 3600 + 15 * 60; // 1 hour 15 minutes = 4500 seconds
  } else {
    countdownSeconds = 1 * 3600 + 15 * 60; // Default to 1 hour 15 minutes
  }
}

void updateCountdown() {
  // Decrease countdown by 1 second every second
  if (millis() - lastCountdownUpdate >= 1000 && countdownSeconds > 0) {
    countdownSeconds--;
    lastCountdownUpdate = millis();
  }
}

void printCountdown() {
  unsigned long hours = countdownSeconds / 3600;
  unsigned long minutes = (countdownSeconds % 3600) / 60;
  unsigned long seconds = countdownSeconds % 60;

  Serial.print("Countdown Timer: ");
  Serial.printf("%02lu:%02lu:%02lu\n", hours, minutes, seconds);
}
