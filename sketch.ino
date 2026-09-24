/**
 * @file automated_micro_climate_nursery.ino
 * @brief Automated Commercial Micro-Climate Nursery using ESP32.
 *
 * @details
 * This project implements an automated micro-climate nursery using
 * temperature, humidity and light sensors. The ESP32 controls a
 * ventilation servo, grow LED, buzzer and OLED display.
 *
 * System modes:
 * - Autonomous Mode
 * - Manual Override Mode
 * - Safety/Error Mode
 *
 * Safety/Error Mode has the highest priority, followed by Manual
 * Override Mode and then Autonomous Mode.
 *
 * @author Akshana Sriskandarajah
 */

// ============================================================
// LIBRARIES
// ============================================================

#include <Arduino.h>
#include <Wire.h>
#include <DHT.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>

// ============================================================
// PIN DEFINITIONS
// ============================================================

/**
 * @brief GPIO pin connected to the DHT22 data line.
 */
#define DHT_PIN 4

/**
 * @brief DHT sensor type.
 */
#define DHT_TYPE DHT22

/**
 * @brief ESP32 ADC pin connected to the LDR analogue output.
 */
#define LDR_PIN 34

/**
 * @brief GPIO pin connected to the manual override push button.
 */
#define BUTTON_PIN 27

/**
 * @brief GPIO pin used to control the ventilation servo.
 */
#define SERVO_PIN 18

/**
 * @brief I2C SDA pin used by the OLED display.
 */
#define OLED_SDA 21

/**
 * @brief I2C SCL pin used by the OLED display.
 */
#define OLED_SCL 22

/**
 * @brief GPIO pin connected to the grow LED.
 */
#define LED_PIN 25

/**
 * @brief GPIO pin connected to the buzzer.
 */
#define BUZZER_PIN 26

// ============================================================
// OLED CONFIGURATION
// ============================================================

/**
 * @brief OLED screen width in pixels.
 */
#define SCREEN_WIDTH 128

/**
 * @brief OLED screen height in pixels.
 */
#define SCREEN_HEIGHT 64

/**
 * @brief Default I2C address of the OLED display.
 */
#define OLED_ADDRESS 0x3C

/**
 * @brief OLED display object.
 */
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ============================================================
// SENSOR AND ACTUATOR OBJECTS
// ============================================================

/**
 * @brief DHT22 temperature and humidity sensor object.
 */
DHT dht(DHT_PIN, DHT_TYPE);

/**
 * @brief Servo object used to control the ventilation vent.
 */
Servo ventilationServo;

// ============================================================
// SYSTEM THRESHOLDS
// ============================================================

/**
 * @brief Temperature at which the ventilation system opens.
 */
float HIGH_TEMP_THRESHOLD = 30.0;

/**
 * @brief Temperature at which the ventilation system closes.
 */
float LOW_TEMP_THRESHOLD = 28.0;

/**
 * @brief LDR value below which the grow LED is switched on.
 */
int LOW_LIGHT_THRESHOLD = 1500;

// ============================================================
// SERVO POSITIONS
// ============================================================

/**
 * @brief Servo position representing a closed ventilation vent.
 */
const int VENT_CLOSED = 0;

/**
 * @brief Servo position representing an open ventilation vent.
 */
const int VENT_OPEN = 90;

// ============================================================
// SYSTEM MODES
// ============================================================

/**
 * @brief Defines the operating modes of the nursery.
 */
enum SystemMode
{
  AUTONOMOUS_MODE,
  MANUAL_MODE,
  ERROR_MODE
};

/**
 * @brief Stores the current operating mode.
 */
SystemMode currentMode = AUTONOMOUS_MODE;

// ============================================================
// SENSOR VARIABLES
// ============================================================

/**
 * @brief Current temperature measured by the DHT22.
 */
float temperature = 0.0;

/**
 * @brief Current humidity measured by the DHT22.
 */
float humidity = 0.0;

/**
 * @brief Current analogue value measured by the LDR.
 */
int lightLevel = 0;

// ============================================================
// SYSTEM STATES
// ============================================================

/**
 * @brief Stores whether the ventilation vent is currently open.
 */
bool ventIsOpen = false;

/**
 * @brief Stores whether the grow LED is currently switched on.
 */
bool growLightOn = false;

/**
 * @brief Stores whether a DHT22 sensor error has occurred.
 */
bool sensorError = false;

/**
 * @brief Stores whether the OLED display was detected successfully.
 *
 * An OLED failure does not place the complete system into
 * Safety/Error Mode. The main control system can continue operating.
 */
bool oledAvailable = false;

/**
 * @brief Stores the previous button state for edge detection.
 */
bool lastButtonState = HIGH;

// ============================================================
// NON-BLOCKING TIMERS
// ============================================================

/**
 * @brief Stores the previous sensor reading time.
 */
unsigned long previousSensorTime = 0;

/**
 * @brief Stores the previous OLED update time.
 */
unsigned long previousDisplayTime = 0;

/**
 * @brief Stores the previous status output time.
 */
unsigned long previousStatusTime = 0;

/**
 * @brief Sensor reading interval in milliseconds.
 */
const unsigned long SENSOR_INTERVAL = 2000;

/**
 * @brief OLED update interval in milliseconds.
 */
const unsigned long DISPLAY_INTERVAL = 1000;

/**
 * @brief Serial status output interval in milliseconds.
 */
const unsigned long STATUS_INTERVAL = 5000;

// ============================================================
// FUNCTION DECLARATIONS
// ============================================================

/**
 * @brief Initialises sensors, actuators, communication and display.
 */
void setup();

/**
 * @brief Main program loop.
 */
void loop();

/**
 * @brief Reads temperature, humidity and light-level sensors.
 */
void readSensors();

/**
 * @brief Checks the manual override push button.
 */
void checkButton();

/**
 * @brief Controls the nursery automatically using sensor readings.
 */
void autonomousControl();

/**
 * @brief Controls the nursery during manual override.
 */
void manualControl();

/**
 * @brief Activates Safety/Error Mode.
 */
void safetyControl();

/**
 * @brief Opens the ventilation servo.
 */
void openVent();

/**
 * @brief Closes the ventilation servo.
 */
void closeVent();

/**
 * @brief Switches the grow LED on or off.
 *
 * @param state true to switch the LED on, false to switch it off.
 */
void setGrowLight(bool state);

/**
 * @brief Switches the buzzer on or off.
 *
 * @param state true to switch the buzzer on, false to switch it off.
 */
void setBuzzer(bool state);

/**
 * @brief Updates information shown on the OLED display.
 */
void updateDisplay();

/**
 * @brief Prints current system information to the Serial Monitor.
 */
void printStatus();

/**
 * @brief Processes UART commands entered through Serial Monitor.
 */
void processUART();

// ============================================================
// SETUP
// ============================================================

/**
 * @brief Initialises the complete micro-climate nursery system.
 *
 * Initialises the DHT22, button, LED, buzzer, I2C communication,
 * OLED display and ventilation servo.
 */
void setup()
{
  Serial.begin(115200);

  delay(500);

  Serial.println();
  Serial.println("======================================");
  Serial.println(" AUTOMATED MICRO-CLIMATE NURSERY");
  Serial.println(" Scenario 2 - ESP32");
  Serial.println("======================================");
  Serial.println();

  // Initialise DHT22
  dht.begin();

  // Initialise manual override button
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Initialise grow LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Initialise buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // Initialise I2C communication
  Wire.begin(OLED_SDA, OLED_SCL);

  // Initialise OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS))
  {
    /**
     * OLED failure is treated as a warning rather than a
     * complete system failure.
     */
    oledAvailable = false;

    Serial.println("WARNING: OLED not detected!");
    Serial.println("System will continue without OLED.");
  }
  else
  {
    oledAvailable = true;

    Serial.println("OLED detected successfully.");

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);

    display.println("MICRO-CLIMATE");
    display.println("NURSERY");
    display.println();
    display.println("System Starting...");

    display.display();

    delay(1500);
  }

  // Initialise ventilation servo
  ventilationServo.attach(SERVO_PIN);

  // Start with the ventilation vent closed
  ventilationServo.write(VENT_CLOSED);
  ventIsOpen = false;

  Serial.println();
  Serial.println("System started.");
  Serial.println("Mode: AUTONOMOUS");
  Serial.println();

  // Display available UART commands
  Serial.println("UART COMMANDS:");
  Serial.println("AUTO");
  Serial.println("MANUAL");
  Serial.println("STATUS");
  Serial.println("TEMP=30");
  Serial.println("LIGHT=1500");
  Serial.println();

  // Perform an initial sensor reading
  readSensors();
}

// ============================================================
// MAIN LOOP
// ============================================================

/**
 * @brief Continuously runs the nursery control system.
 *
 * The control priority is:
 * 1. Safety/Error Mode
 * 2. Manual Override Mode
 * 3. Autonomous Mode
 *
 * Non-blocking millis() timers are used for regular sensor,
 * display and status updates.
 */
void loop()
{
  // Check manual override button
  checkButton();

  // Read sensors at the defined interval
  if (millis() - previousSensorTime >= SENSOR_INTERVAL)
  {
    previousSensorTime = millis();

    readSensors();
  }

  // ----------------------------------------------------------
  // SYSTEM PRIORITY
  // ----------------------------------------------------------

  if (sensorError)
  {
    currentMode = ERROR_MODE;

    safetyControl();
  }
  else if (currentMode == MANUAL_MODE)
  {
    manualControl();
  }
  else
  {
    currentMode = AUTONOMOUS_MODE;

    autonomousControl();
  }

  // Update OLED
  if (millis() - previousDisplayTime >= DISPLAY_INTERVAL)
  {
    previousDisplayTime = millis();

    updateDisplay();
  }

  // Print system status
  if (millis() - previousStatusTime >= STATUS_INTERVAL)
  {
    previousStatusTime = millis();

    printStatus();
  }

  // Process UART commands
  processUART();
}

// ============================================================
// SENSOR READING
// ============================================================

/**
 * @brief Reads temperature, humidity and light level.
 *
 * The DHT22 provides temperature and humidity measurements.
 * The LDR provides an analogue light-level measurement through
 * ESP32 ADC GPIO 34.
 *
 * If the DHT22 returns an invalid reading, the system enters
 * Safety/Error Mode.
 */
void readSensors()
{
  float newTemperature = dht.readTemperature();
  float newHumidity = dht.readHumidity();

  // Check for invalid DHT22 readings
  if (isnan(newTemperature) || isnan(newHumidity))
  {
    sensorError = true;

    Serial.println();
    Serial.println("ERROR: DHT22 reading failed!");
    Serial.println("System entering SAFETY / ERROR MODE.");

    return;
  }

  // Store valid readings
  temperature = newTemperature;
  humidity = newHumidity;

  // Read analogue LDR value
  lightLevel = analogRead(LDR_PIN);

  // DHT22 is operating correctly
  sensorError = false;

  Serial.print("Temperature: ");
  Serial.print(temperature);

  Serial.print(" C | Humidity: ");
  Serial.print(humidity);

  Serial.print(" % | LDR: ");
  Serial.println(lightLevel);
}

// ============================================================
// BUTTON CONTROL
// ============================================================

/**
 * @brief Detects a button press and toggles Manual Override Mode.
 *
 * The button uses INPUT_PULLUP, therefore a LOW signal indicates
 * that the button is pressed.
 */
void checkButton()
{
  bool currentButtonState = digitalRead(BUTTON_PIN);

  // Detect HIGH-to-LOW transition
  if (lastButtonState == HIGH && currentButtonState == LOW)
  {
    // Manual mode is unavailable while a sensor error exists
    if (!sensorError)
    {
      if (currentMode == MANUAL_MODE)
      {
        currentMode = AUTONOMOUS_MODE;

        Serial.println();
        Serial.println("Manual Override OFF");
        Serial.println("Mode: AUTONOMOUS");
      }
      else
      {
        currentMode = MANUAL_MODE;

        Serial.println();
        Serial.println("Manual Override ON");
        Serial.println("Ventilation forced OPEN");
      }
    }

    // Simple button debounce
    delay(50);
  }

  lastButtonState = currentButtonState;
}

// ============================================================
// AUTONOMOUS CONTROL
// ============================================================

/**
 * @brief Performs automatic environmental control.
 *
 * The grow LED is controlled according to the LDR light level.
 * The ventilation servo is controlled according to temperature.
 */
void autonomousControl()
{
  // Control grow light according to LDR
  if (lightLevel < LOW_LIGHT_THRESHOLD)
  {
    setGrowLight(true);
  }
  else
  {
    setGrowLight(false);
  }

  // Control ventilation according to temperature
  if (temperature >= HIGH_TEMP_THRESHOLD)
  {
    openVent();
  }
  else if (temperature <= LOW_TEMP_THRESHOLD)
  {
    closeVent();
  }
}

// ============================================================
// MANUAL CONTROL
// ============================================================

/**
 * @brief Performs Manual Override control.
 *
 * During Manual Override Mode, the ventilation vent is forced
 * open. The grow LED continues to respond to the LDR reading.
 */
void manualControl()
{
  // Force ventilation open
  openVent();

  // Grow LED continues to respond to light level
  if (lightLevel < LOW_LIGHT_THRESHOLD)
  {
    setGrowLight(true);
  }
  else
  {
    setGrowLight(false);
  }

  // Buzzer remains off during normal manual operation
  setBuzzer(false);
}

// ============================================================
// SAFETY / ERROR CONTROL
// ============================================================

/**
 * @brief Performs Safety/Error Mode actions.
 *
 * When the DHT22 fails, the system opens the ventilation,
 * switches on the grow LED and activates the buzzer.
 */
void safetyControl()
{
  Serial.println("!!! SAFETY / ERROR MODE !!!");

  // Open ventilation for safety
  openVent();

  // Turn grow LED on
  setGrowLight(true);

  // Activate buzzer
  setBuzzer(true);
}

// ============================================================
// OPEN VENT
// ============================================================

/**
 * @brief Opens the ventilation vent using the servo.
 *
 * The servo is moved to the predefined VENT_OPEN position.
 */
void openVent()
{
  if (!ventIsOpen)
  {
    ventilationServo.write(VENT_OPEN);

    ventIsOpen = true;

    Serial.println("Ventilation: OPEN");
  }
}

// ============================================================
// CLOSE VENT
// ============================================================

/**
 * @brief Closes the ventilation vent using the servo.
 *
 * The servo is moved to the predefined VENT_CLOSED position.
 */
void closeVent()
{
  if (ventIsOpen)
  {
    ventilationServo.write(VENT_CLOSED);

    ventIsOpen = false;

    Serial.println("Ventilation: CLOSED");
  }
}

// ============================================================
// GROW LED CONTROL
// ============================================================

/**
 * @brief Controls the grow LED.
 *
 * @param state true turns the grow LED on; false turns it off.
 */
void setGrowLight(bool state)
{
  if (growLightOn != state)
  {
    growLightOn = state;

    digitalWrite(LED_PIN, state ? HIGH : LOW);

    if (state)
    {
      Serial.println("Grow LED: ON");
    }
    else
    {
      Serial.println("Grow LED: OFF");
    }
  }
}

// ============================================================
// BUZZER CONTROL
// ============================================================

/**
 * @brief Controls the buzzer.
 *
 * @param state true turns the buzzer on; false turns it off.
 */
void setBuzzer(bool state)
{
  digitalWrite(BUZZER_PIN, state ? HIGH : LOW);
}

// ============================================================
// OLED DISPLAY
// ============================================================

/**
 * @brief Updates the OLED with current nursery information.
 *
 * Displays temperature, humidity, light level, ventilation state,
 * operating mode and system condition.
 *
 * If the OLED is unavailable, the function exits without affecting
 * the remaining control system.
 */
void updateDisplay()
{
  // Skip display update if OLED is unavailable
  if (!oledAvailable)
  {
    return;
  }

  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Temperature
  display.setCursor(0, 0);
  display.print("Temp: ");
  display.print(temperature, 1);
  display.println(" C");

  // Humidity
  display.setCursor(0, 10);
  display.print("Humidity: ");
  display.print(humidity, 1);
  display.println("%");

  // Light level
  display.setCursor(0, 20);
  display.print("Light: ");
  display.println(lightLevel);

  // Ventilation
  display.setCursor(0, 30);
  display.print("Vent: ");

  if (ventIsOpen)
  {
    display.println("OPEN");
  }
  else
  {
    display.println("CLOSED");
  }

  // Operating mode
  display.setCursor(0, 40);
  display.print("Mode: ");

  if (currentMode == AUTONOMOUS_MODE)
  {
    display.println("AUTO");
  }
  else if (currentMode == MANUAL_MODE)
  {
    display.println("MANUAL");
  }
  else
  {
    display.println("ERROR");
  }

  // System status
  display.setCursor(0, 50);

  if (sensorError)
  {
    display.println("SAFETY ERROR!");
  }
  else if (currentMode == MANUAL_MODE)
  {
    display.println("MANUAL OVERRIDE");
  }
  else
  {
    display.println("SYSTEM NORMAL");
  }

  display.display();
}

// ============================================================
// SERIAL STATUS
// ============================================================

/**
 * @brief Prints the current system status to Serial Monitor.
 *
 * Provides sensor values, actuator states, OLED availability,
 * error status and current operating mode.
 */
void printStatus()
{
  Serial.println();
  Serial.println("----------- SYSTEM STATUS -----------");

  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" C");

  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");

  Serial.print("LDR: ");
  Serial.println(lightLevel);

  Serial.print("Ventilation: ");

  if (ventIsOpen)
  {
    Serial.println("OPEN");
  }
  else
  {
    Serial.println("CLOSED");
  }

  Serial.print("Grow LED: ");

  if (growLightOn)
  {
    Serial.println("ON");
  }
  else
  {
    Serial.println("OFF");
  }

  Serial.print("OLED: ");

  if (oledAvailable)
  {
    Serial.println("AVAILABLE");
  }
  else
  {
    Serial.println("NOT DETECTED");
  }

  Serial.print("Error: ");

  if (sensorError)
  {
    Serial.println("YES");
  }
  else
  {
    Serial.println("NO");
  }

  Serial.print("Mode: ");

  if (currentMode == AUTONOMOUS_MODE)
  {
    Serial.println("AUTONOMOUS");
  }
  else if (currentMode == MANUAL_MODE)
  {
    Serial.println("MANUAL OVERRIDE");
  }
  else
  {
    Serial.println("SAFETY / ERROR");
  }

  Serial.println("------------------------------------");
}

// ============================================================
// UART COMMAND PROCESSING
// ============================================================

/**
 * @brief Processes commands received through UART/Serial Monitor.
 *
 * Supported commands:
 * - AUTO
 * - MANUAL
 * - STATUS
 * - TEMP=30
 * - LIGHT=1500
 */
void processUART()
{
  if (Serial.available())
  {
    String command = Serial.readStringUntil('\n');

    command.trim();
    command.toUpperCase();

    // --------------------------------------------------------
    // AUTO COMMAND
    // --------------------------------------------------------

    if (command == "AUTO")
    {
      if (!sensorError)
      {
        currentMode = AUTONOMOUS_MODE;

        Serial.println("UART: Autonomous Mode selected.");
      }
      else
      {
        Serial.println("UART: Cannot enter Autonomous Mode.");
        Serial.println("DHT22 sensor error is active.");
      }
    }

    // --------------------------------------------------------
    // MANUAL COMMAND
    // --------------------------------------------------------

    else if (command == "MANUAL")
    {
      if (!sensorError)
      {
        currentMode = MANUAL_MODE;

        Serial.println("UART: Manual Override selected.");
        Serial.println("Ventilation forced OPEN.");
      }
      else
      {
        Serial.println("UART: Manual mode unavailable during Safety/Error.");
      }
    }

    // --------------------------------------------------------
    // STATUS COMMAND
    // --------------------------------------------------------

    else if (command == "STATUS")
    {
      printStatus();
    }

    // --------------------------------------------------------
    // TEMPERATURE THRESHOLD
    // Example: TEMP=30
    // --------------------------------------------------------

    else if (command.startsWith("TEMP="))
    {
      String value = command.substring(5);

      float newThreshold = value.toFloat();

      if (newThreshold > 0)
      {
        HIGH_TEMP_THRESHOLD = newThreshold;

        // Maintain a 2-degree hysteresis gap
        if (LOW_TEMP_THRESHOLD >= HIGH_TEMP_THRESHOLD)
        {
          LOW_TEMP_THRESHOLD = HIGH_TEMP_THRESHOLD - 2.0;
        }

        Serial.print("High temperature threshold changed to: ");
        Serial.print(HIGH_TEMP_THRESHOLD);
        Serial.println(" C");
      }
      else
      {
        Serial.println("Invalid temperature value.");
      }
    }

    // --------------------------------------------------------
    // LIGHT THRESHOLD
    // Example: LIGHT=1500
    // --------------------------------------------------------

    else if (command.startsWith("LIGHT="))
    {
      String value = command.substring(6);

      int newThreshold = value.toInt();

      if (newThreshold >= 0)
      {
        LOW_LIGHT_THRESHOLD = newThreshold;

        Serial.print("Low light threshold changed to: ");
        Serial.println(LOW_LIGHT_THRESHOLD);
      }
      else
      {
        Serial.println("Invalid light threshold.");
      }
    }

    // --------------------------------------------------------
    // UNKNOWN COMMAND
    // --------------------------------------------------------

    else if (command.length() > 0)
    {
      Serial.println("Unknown command.");
      Serial.println(
        "Use: AUTO, MANUAL, STATUS, TEMP=30, LIGHT=1500"
      );
    }
  }
}