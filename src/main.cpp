#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>

// Include sensor libraries based on the board type
#if defined(ARDUINO_RASPBERRY_PI_PICO)
#define ONBOARD_LED_PIN 21
#include <SI7021.h>
SI7021 sensor;
#elif defined(ARDUINO_ESP32_DEV)
#define ONBOARD_LED_PIN 2
#include <Adafruit_Si7021.h>
Adafruit_Si7021 sensor = Adafruit_Si7021();
#endif

// OLED and pin definitions
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define RELAY_PIN 27
#define MOISTURE_PIN 34
#define BUTTON_PIN 4

// Sensor reading and display timing
#define MOISTURE_THRESHOLD 50.0 // Percentage threshold for moisture
#define DISPLAY_TOGGLE_MS 2500  // 2.5 seconds

// Moisture sensor calibration values
#define MOISTURE_DRY 2800
#define MOISTURE_WET 1080

// Temperature calibration value
#define TEMPERATURE_CALIBRATION -7.12

// Create OLED display object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Timing and state variables
unsigned long lastReadTime = 0;
unsigned long lastDisplayToggleTime = 0;
bool autoModeEnabled = false;
bool relayForcedState = false;
bool relayIsForced = false;

// Sensor value holders
float temperature = 0.0;
float humidityPct = 0.0;
float moisturePct = 0.0;
unsigned long readIntervalMs = 300000; // Default to 5 minutes

// WiFi  credentials
const char *ssid = "Artemis";
const char *password = "8012018955";

// MQTT topics and network credentials
const char *device_name = "water_plant_1";
const char *mqtt_prefix = "homeassistant";
const char *state_topic_temp = "home/water_plant_1/sensor/temperature";
const char *state_topic_humidity = "home/water_plant_1/sensor/humidity";
const char *state_topic_moisture = "home/water_plant_1/sensor/moisture";
const char *relay_state_topic = "home/water_plant_1/relay/state";
const char *relay_command_topic = "home/water_plant_1/relay/set";
const char *read_interval_topic = "home/water_plant_1/interval/set";
const char *auto_mode_command_topic = "home/water_plant_1/auto_mode/set";
const char *auto_mode_state_topic = "home/water_plant_1/auto_mode/state";
const char *availability_topic = "home/water_plant_1/status";
const char *notify_topic = "home/water_plant_1/notify";
const char *mqtt_server = "192.168.11.54";
const char *mqtt_user = "mqttuser";
const char *mqtt_pass = "mqttuser";

// Create Wi-Fi and MQTT clients
WiFiClient espClient;
PubSubClient client(espClient);

// Display state enumeration
enum DisplayState
{
  AUTO,
  PUMP,
  TEMPERATURE,
  HUMIDITY,
  MOISTURE
};
DisplayState displayState = AUTO;

// Button interrupt variables
volatile bool buttonChanged = false;
volatile unsigned long buttonLastChange = 0;
const unsigned long debounceDelay = 50; // ms

// Notification variables
unsigned long lastWateringTime = 0;
unsigned long lastLowMoistureAlert = 0;
bool lowMoistureAlertSent = false;
bool wateringInProgress = false;
const unsigned long NOTIFICATION_COOLDOWN = 300000; // 5 minutes between similar notifications
const unsigned long LOW_MOISTURE_ALERT_INTERVAL = 1800000; // 30 minutes between low moisture alerts

// Notification types
enum NotificationType {
  WATERING_STARTED,
  WATERING_COMPLETED,
  LOW_MOISTURE_ALERT,
  SENSOR_ERROR,
  SYSTEM_STATUS
};

// Interrupt service routine for button press
void IRAM_ATTR handleButtonInterrupt()
{
  unsigned long now = millis();
  if (now - buttonLastChange > debounceDelay)
  {
    buttonChanged = true;
    buttonLastChange = now;
  }
}

// Set relay state and publish to MQTT
void update_relay_state(bool is_on)
{
  digitalWrite(RELAY_PIN, is_on ? HIGH : LOW);
  digitalWrite(ONBOARD_LED_PIN, is_on ? HIGH : LOW);
  const char *state = is_on ? "ON" : "OFF";
  client.publish(relay_state_topic, state, true);
}

// Send MQTT notification
void sendNotification(NotificationType type, String message = "") {
  if (!client.connected()) return;
  
  String notification;
  String timestamp = String(millis() / 1000); // Simple timestamp in seconds
  
  switch (type) {
    case WATERING_STARTED:
      notification = "{\"type\":\"watering_started\",\"message\":\"Plant watering started - Moisture: " + String(moisturePct, 1) + "%\",\"timestamp\":" + timestamp + ",\"moisture\":" + String(moisturePct, 2) + "}";
      wateringInProgress = true;
      lastWateringTime = millis();
      break;
      
    case WATERING_COMPLETED:
      notification = "{\"type\":\"watering_completed\",\"message\":\"Plant watering completed\",\"timestamp\":" + timestamp + ",\"duration\":" + String((millis() - lastWateringTime) / 1000) + "}";
      wateringInProgress = false;
      break;
      
    case LOW_MOISTURE_ALERT:
      if (millis() - lastLowMoistureAlert < LOW_MOISTURE_ALERT_INTERVAL && lowMoistureAlertSent) return;
      notification = "{\"type\":\"low_moisture\",\"message\":\"Low soil moisture detected: " + String(moisturePct, 1) + "%\",\"timestamp\":" + timestamp + ",\"moisture\":" + String(moisturePct, 2) + "}";
      lastLowMoistureAlert = millis();
      lowMoistureAlertSent = true;
      break;
      
    case SENSOR_ERROR:
      notification = "{\"type\":\"sensor_error\",\"message\":\"" + message + "\",\"timestamp\":" + timestamp + "}";
      break;
      
    case SYSTEM_STATUS:
      notification = "{\"type\":\"system_status\",\"message\":\"" + message + "\",\"timestamp\":" + timestamp + ",\"temperature\":" + String(temperature, 1) + ",\"humidity\":" + String(humidityPct, 1) + ",\"moisture\":" + String(moisturePct, 1) + "}";
      break;
  }
  
  Serial.println("Sending notification: " + notification);
  client.publish(notify_topic, notification.c_str(), false);
}

void readSensorData()
{
  // Read sensor data
  bool sensorError = false;

  // Read Humidity Percentage
  float humidityRaw = sensor.readHumidity();
  if (!isnan(humidityRaw))
  {
    humidityPct = humidityRaw;
  }
  else
  {
    sensorError = true;
    sendNotification(SENSOR_ERROR, "Failed to read humidity sensor");
  }

  // Read Temperature in Fahrenheit
  float temperatureRaw = sensor.readTemperature();
  if (!isnan(temperatureRaw))
  {
    temperature = (temperatureRaw * 9.0 / 5.0 + 32.0) + TEMPERATURE_CALIBRATION; // Convert to Fahrenheit and apply calibration
  }
  else
  {
    sensorError = true;
    sendNotification(SENSOR_ERROR, "Failed to read temperature sensor");
  }

  // Read Moisture Percentage
  int moistureRaw = analogRead(MOISTURE_PIN);
  if (!isnan(moistureRaw))
  {
    // Calculate inverted percentage
    float percentage = (MOISTURE_DRY - moistureRaw) * 100.0 / (MOISTURE_DRY - MOISTURE_WET);
    // Clamp the result between 0 and 100
    moisturePct = constrain(percentage, 0.0, 100.0);
  }
  else
  {
    sensorError = true;
    sendNotification(SENSOR_ERROR, "Failed to read moisture sensor");
  }

  Serial.printf("Humidity: %.2f%%, Moisture: %.2f%% (%d), Temperature: %.2f °F\n", humidityPct, moisturePct, moistureRaw, temperature);

  // Check for low moisture alert (only if not currently watering)
  if (moisturePct < MOISTURE_THRESHOLD && !wateringInProgress && autoModeEnabled) {
    sendNotification(LOW_MOISTURE_ALERT);
  }

  // Reset low moisture alert flag if moisture is adequate
  if (moisturePct >= MOISTURE_THRESHOLD + 5.0) { // Add hysteresis
    lowMoistureAlertSent = false;
  }

  // Control relay automatically unless overridden
  if (!relayIsForced && autoModeEnabled)
  {
    bool currentRelayState = digitalRead(RELAY_PIN);
    
    if (moisturePct < MOISTURE_THRESHOLD && !currentRelayState)
    {
      update_relay_state(true);
      sendNotification(WATERING_STARTED);
      Serial.println("Relay ON (auto)");
    }
    else if (moisturePct >= MOISTURE_THRESHOLD && currentRelayState)
    {
      update_relay_state(false);
      sendNotification(WATERING_COMPLETED);
      Serial.println("Relay OFF (auto)");
    }
  }

  // Publish sensor readings (publish temperature in Fahrenheit)
  client.publish(state_topic_temp, String(temperature).c_str(), true);
  client.publish(state_topic_humidity, String(humidityPct).c_str(), true);
  client.publish(state_topic_moisture, String(moisturePct).c_str(), true);
}

// Handle incoming MQTT messages
void callback(char *topic, byte *payload, unsigned int length)
{
  payload[length] = '\0';
  String command = String((char *)payload);

  if (String(topic) == relay_command_topic)
  {
    if (command == "ON")
    {
      relayForcedState = true;
      relayIsForced = true;
      update_relay_state(true);
      sendNotification(WATERING_STARTED);
      Serial.println("Relay turned ON via MQTT.");
    }
    else if (command == "OFF")
    {
      relayForcedState = false;
      relayIsForced = true;
      update_relay_state(false);
      if (wateringInProgress) {
        sendNotification(WATERING_COMPLETED);
      }
      Serial.println("Relay turned OFF via MQTT.");
    }
  }
  else if (String(topic) == read_interval_topic)
  {
    long interval = command.toInt();
    if (interval >= 10000 && interval <= 3600000)
    { // 10 sec to 1 hour
      readIntervalMs = interval;
      sendNotification(SYSTEM_STATUS, "Read interval updated to " + String(interval / 1000) + " seconds");
      Serial.printf("Updated read interval to %lu ms\n", readIntervalMs);
    }
    else
    {
      Serial.println("Invalid interval received via MQTT.");
    }
  }
  else if (String(topic) == auto_mode_command_topic)
  {
    if (command == "ON")
    {
      autoModeEnabled = true;
      readSensorData();
      sendNotification(SYSTEM_STATUS, "Auto watering mode enabled");
      Serial.println("Auto watering ENABLED via MQTT");
    }
    else if (command == "OFF")
    {
      autoModeEnabled = false;
      if (wateringInProgress) {
        sendNotification(WATERING_COMPLETED);
      }
      sendNotification(SYSTEM_STATUS, "Auto watering mode disabled");
      client.publish(relay_command_topic, "OFF", true);
      Serial.println("Auto watering DISABLED via MQTT");
    }
    client.publish(auto_mode_state_topic, autoModeEnabled ? "ON" : "OFF", true);
  }
}

// Connect to Wi-Fi network
void setup_wifi()
{
  delay(10);
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected.");
  Serial.println(WiFi.localIP());
}

// Publish MQTT auto-discovery config for Home Assistant
void publish_discovery()
{
  String device_json = String("\"device\": {\"identifiers\": [\"") + device_name +
                       "\"], \"name\": \"Water Plant 1\"," +
                       "\"manufacturer\": \"Tom's Plant Co.\"," +
                       "\"model\": \"ESP32 Plant Monitor\"," +
                       "\"sw_version\": \"1.1\"}";

  // Temperature config (change unit to °F)
  String temp_config = String("{") +
                       "\"name\": \"Plant Temperature\"," +
                       "\"state_topic\": \"" + state_topic_temp + "\"," +
                       "\"state_class\": \"measurement\"," +
                       "\"unit_of_measurement\": \"°F\"," + // <-- changed
                       "\"device_class\": \"temperature\"," +
                       "\"object_id\": \"temperature\"," +
                       "\"unique_id\": \"" + String(device_name) + "_temperature\"," +
                       "\"availability_topic\": \"" + availability_topic + "\"," +
                       device_json +
                       "}";

  // Humidity config
  String humidity_config = String("{") +
                           "\"name\": \"Plant Humidity\"," +
                           "\"state_topic\": \"" + state_topic_humidity + "\"," +
                           "\"state_class\": \"measurement\"," +
                           "\"unit_of_measurement\": \"%\"," +
                           "\"device_class\": \"humidity\"," +
                           "\"object_id\": \"humidity\"," +
                           "\"unique_id\": \"" + String(device_name) + "_humidity\"," +
                           "\"availability_topic\": \"" + availability_topic + "\"," +
                           device_json +
                           "}";

  // Moisture config
  String moisture_config = String("{") +
                           "\"name\": \"Soil Moisture\"," +
                           "\"state_topic\": \"" + state_topic_moisture + "\"," +
                           "\"state_class\": \"measurement\"," +
                           "\"unit_of_measurement\": \"%\"," +
                           "\"device_class\": \"moisture\"," +
                           "\"object_id\": \"moisture\"," +
                           "\"unique_id\": \"" + String(device_name) + "_moisture\"," +
                           "\"availability_topic\": \"" + availability_topic + "\"," +
                           device_json +
                           "}";

  // Read Interval config
  String read_interval_config = String("{") +
                                "\"name\": \"Read Interval\"," +
                                "\"unique_id\": \"" + String(device_name) + "_interval\"," +
                                "\"object_id\": \"interval\"," +
                                "\"command_topic\": \"" + read_interval_topic + "\"," +
                                "\"min\": 10000," +
                                "\"max\": 3600000," +
                                "\"step\": 1000," +
                                "\"unit_of_measurement\": \"ms\"," +
                                "\"mode\": \"box\"," +
                                "\"availability_topic\": \"" + availability_topic + "\"," +
                                device_json +
                                "}";

  // Relay config
  String relay_config = String("{") +
                        "\"name\": \"Water Pump Relay\"," +
                        "\"state_topic\": \"" + relay_state_topic + "\"," +
                        "\"command_topic\": \"" + relay_command_topic + "\"," +
                        "\"availability_topic\": \"" + availability_topic + "\"," +
                        "\"payload_on\": \"ON\"," +
                        "\"payload_off\": \"OFF\"," +
                        "\"object_id\": \"relay\"," +
                        "\"unique_id\": \"" + String(device_name) + "_relay\"," +
                        "\"device\": {\"identifiers\": [\"" + String(device_name) + "\"], \"name\": \"Water Plant 1\"}" +
                        "}";

  String auto_mode_config = String("{") +
                            "\"name\": \"Auto Watering Mode\"," +
                            "\"state_topic\": \"" + auto_mode_state_topic + "\"," +
                            "\"command_topic\": \"" + auto_mode_command_topic + "\"," +
                            "\"payload_on\": \"ON\"," +
                            "\"payload_off\": \"OFF\"," +
                            "\"unique_id\": \"" + String(device_name) + "_auto_mode\"," +
                            "\"availability_topic\": \"" + availability_topic + "\"," +
                            "\"device\": {\"identifiers\": [\"" + device_name + "\"], \"name\": \"Water Plant 1\"}" +
                            "}";

  // Notify sensor config for Home Assistant
  String notify_config = String("{") +
                              "\"name\": \"Plant Notify\"," +
                              "\"state_topic\": \"" + notify_topic + "\"," +
                              "\"value_template\": \"{{ value_json.message }}\"," +
                              "\"json_attributes_topic\": \"" + notify_topic + "\"," +
                              "\"object_id\": \"notify\"," +
                              "\"unique_id\": \"" + String(device_name) + "_notify\"," +
                              "\"availability_topic\": \"" + availability_topic + "\"," +
                              "\"icon\": \"mdi:bell-alert\"," +
                              device_json +
                              "}";

  // Publish discovery messages
  client.setBufferSize(1024); // Increase buffer size for larger messages
  client.publish("homeassistant/sensor/water_plant_1/temperature/config", temp_config.c_str(), true);
  client.publish("homeassistant/sensor/water_plant_1/humidity/config", humidity_config.c_str(), true);
  client.publish("homeassistant/sensor/water_plant_1/moisture/config", moisture_config.c_str(), true);
  client.publish("homeassistant/sensor/water_plant_1/notify/config", notify_config.c_str(), true);
  client.publish("homeassistant/number/water_plant_1/read_interval/config", read_interval_config.c_str(), true);
  client.publish("homeassistant/switch/water_plant_1/relay/config", relay_config.c_str(), true);
  client.publish("homeassistant/switch/water_plant_1_auto_mode/config", auto_mode_config.c_str(), true);
}

// Reconnect to MQTT broker if disconnected
void reconnect_mqtt()
{
  while (!client.connected())
  {
    Serial.print("Connecting to MQTT...");
    if (client.connect(
            "water_plant_1",
            mqtt_user,
            mqtt_pass,
            availability_topic,
            1,
            true,
            "offline"))
    {
      client.publish(availability_topic, "online", true);
      Serial.println("connected.");

      // Publish Home Assistant discovery topics
      publish_discovery();

      // Subscribe to command topics
      client.subscribe(relay_command_topic);
      client.subscribe(auto_mode_command_topic);
      client.subscribe(read_interval_topic);

      // Publish initial states
      client.publish(relay_state_topic, digitalRead(RELAY_PIN) ? "ON" : "OFF", true);
      client.publish(auto_mode_state_topic, autoModeEnabled ? "ON" : "OFF", true);
      
      // Send startup notification
      sendNotification(SYSTEM_STATUS, "Plant monitoring system started");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" retrying in 5 sec...");
      delay(5000);
    }
  }
}

// Setup function runs once at boot
void setup()
{
  Serial.begin(115200);

  // Initialize pins
  Wire.begin(21, 22);
  pinMode(MOISTURE_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLDOWN);
  pinMode(ONBOARD_LED_PIN, OUTPUT);
  digitalWrite(ONBOARD_LED_PIN, LOW);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  setup_wifi();

  // Setup OTA
  ArduinoOTA.setHostname("waterplant"); // Optional: match this in platformio.ini
  ArduinoOTA.setPassword("Bannor1.");
  ArduinoOTA.onStart([]()
                     { Serial.println("OTA Start"); });
  ArduinoOTA.onEnd([]()
                   { Serial.println("\nOTA End"); });
  ArduinoOTA.onError([](ota_error_t error)
                     { Serial.printf("OTA Error [%u]: ", error); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                        { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); });
  ArduinoOTA.begin();
  Serial.println("OTA Ready");

  // Setup MQTT client
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleButtonInterrupt, CHANGE);

  // Initialize sensor
  sensor.setHeatLevel(0); // Set heater level to 0
  sensor.heater(false);   // Disable heater by default
  if (!sensor.begin())
  {
    Serial.println("SI7021 not found!");
    delay(1000);
  }

  // Initialize OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    Serial.println("SSD1306 init failed!");
    delay(1000);
  }
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Init...");
  display.display();
}

// Main loop
void loop()
{
  ArduinoOTA.handle(); // Keep OTA active

  unsigned long now = millis();

  // Button hold logic: relay ON while button held, OFF when released
  static bool lastButtonState = LOW;
  bool buttonState = digitalRead(BUTTON_PIN);

  if (buttonChanged)
  {
    buttonChanged = false;
    if (buttonState == HIGH)
    {
      relayForcedState = true;
      relayIsForced = true;
      update_relay_state(true);
      sendNotification(WATERING_STARTED);
      Serial.println("Button held: Relay ON");
    }
    else
    {
      relayForcedState = false;
      relayIsForced = true;
      update_relay_state(false);
      if (wateringInProgress) {
        sendNotification(WATERING_COMPLETED);
      }
      Serial.println("Button released: Relay OFF");
    }
  }

  // Maintain MQTT connection
  if (!client.connected())
  {
    reconnect_mqtt();
  }
  client.loop();

  // Read and publish sensor data periodically
  if (now - lastReadTime >= readIntervalMs || lastReadTime == 0)
  {
    lastReadTime = now;

    readSensorData();
  }

  // Toggle display content periodically
  if (now - lastDisplayToggleTime >= DISPLAY_TOGGLE_MS || lastDisplayToggleTime == 0)
  {
    lastDisplayToggleTime = now;

    display.clearDisplay();
    display.setCursor(0, 16);
    display.setTextSize(2);

    if (displayState == AUTO)
    {
      display.printf("Auto:\n%s", autoModeEnabled ? "ON" : "OFF");
      displayState = PUMP;
    }
    else if (displayState == PUMP)
    {
      display.printf("Pump:\n%s", relayForcedState ? "ON" : "OFF");
      displayState = TEMPERATURE;
    }
    else if (displayState == TEMPERATURE)
    {
      display.printf("Temp:\n%04.1f F", temperature);
      displayState = HUMIDITY;
    }
    else if (displayState == HUMIDITY)
    {
      display.printf("Humidity:\n%.1f%%", humidityPct);
      displayState = MOISTURE;
    }
    else if (displayState == MOISTURE)
    {
      display.printf("Moisture:\n%.1f%%", moisturePct);
      displayState = AUTO;
    }

    display.display();
  }

  delay(100); // Small delay to reduce CPU usage
}
