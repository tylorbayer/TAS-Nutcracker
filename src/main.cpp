#include <Arduino.h>
#include <esp_now.h>
#include "WiFi.h"

// --- ESP-NOW broadcast address ---
const uint8_t broadcastAddress[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Message sent over ESP-NOW when dance starts/stops
typedef struct __attribute__((packed)) {
  uint8_t dance;     // 1 = dancing, 0 = stopped
} dance_msg_t;

// Message to trigger dance from another device
typedef struct __attribute__((packed)) {
  uint8_t discriminator;   // 0xFF: identifies this as a button-triggered message to ignore
  uint8_t dance_trigger;   // 1 = trigger dance, 0 = stop dance
} dance_trigger_msg_t;

// Flag set by receive callback when a trigger message arrives
volatile bool receivedDanceTrigger = false;

// Motor state flag
bool hit_himit = false;
int drive_motor_direction = -1; // 1 forward, -1 backward

// Send callback for debug
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

// Receive callback to listen for dance trigger messages
void onDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  if (len != (int)sizeof(dance_trigger_msg_t)) {
    return;
  }

  // Only set the flag if we're not already dancing
  if (hit_himit && drive_motor_direction == -1) {
    receivedDanceTrigger = true;
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("Received dance trigger message");
  }
}

// Helper to send the dance state via ESP-NOW (broadcast)
void sendDanceState(bool dancing) {
  dance_msg_t msg;
  msg.dance = dancing ? 1 : 0;

  esp_err_t res = esp_now_send(broadcastAddress, (uint8_t *)&msg, sizeof(msg));
  if (res != ESP_OK) {
    Serial.print("esp_now_send failed: ");
    Serial.println(res);
  } else {
    Serial.print("Sent dance state: ");
    Serial.println(msg.dance ? "true" : "false");
  }
}

// Pin mapping (ESP32). Adjust these if you prefer different pins.
const int DRIVE_IN1 = 26; // drive motor forward
const int DRIVE_IN2 = 27; // drive motor backward
const int DRIVE_EN  = 14; // drive motor PWM (enable)

const int DANCE_IN1 = 33; // dance motor forward
const int DANCE_IN2 = 32; // dance motor backward
const int DANCE_EN  = 35; // dance motor PWM (enable)

const int LIMIT_PIN = 25; // limit switch (use INPUT_PULLUP)

// LEDC (ESP32 PWM) configuration
const int DRIVE_LEDC_CH = 0;
const int DANCE_LEDC_CH = 1;
const int LEDC_FREQ = 5000;
const int LEDC_RES  = 8; // 8-bit resolution (0-255)

// Speeds in 0.0 - 1.0 range
float drive_motor_speed = 1.0; // drive speed (1.0 == full duty)

float dance_motor_speed = 0.8; // dance motor speed
unsigned long dance_time_ms = 12000; // dance duration in milliseconds

// Helpers: convert float speed to 0..255 duty
static inline int speedToDuty(float s) {
  s = constrain(s, 0.0f, 1.0f);
  return (int)round(s * ((1 << LEDC_RES) - 1));
}

void driveForward(float speed) {
  digitalWrite(LED_BUILTIN, HIGH);

  digitalWrite(DRIVE_IN1, LOW);
  digitalWrite(DRIVE_IN2, HIGH);
  ledcWrite(DRIVE_LEDC_CH, speedToDuty(speed));
}

void driveBackward(float speed) {
  digitalWrite(LED_BUILTIN, HIGH);

  digitalWrite(DRIVE_IN1, HIGH);
  digitalWrite(DRIVE_IN2, LOW);
  ledcWrite(DRIVE_LEDC_CH, speedToDuty(speed));
}

void driveStop() {
  digitalWrite(LED_BUILTIN, LOW);

  ledcWrite(DRIVE_LEDC_CH, 0);
  digitalWrite(DRIVE_IN1, LOW);
  digitalWrite(DRIVE_IN2, LOW);
}

void danceStart(float speed) {
  digitalWrite(DANCE_IN1, HIGH);
  digitalWrite(DANCE_IN2, LOW);
  ledcWrite(DANCE_LEDC_CH, speedToDuty(speed));
  // notify other devices that dance started
  sendDanceState(true);
}

void danceStop() {
  ledcWrite(DANCE_LEDC_CH, 0);
  digitalWrite(DANCE_IN1, LOW);
  digitalWrite(DANCE_IN2, LOW);
  // notify other devices that dance stopped
  sendDanceState(false);
}

void setup() {
  // Serial.begin(115200);
  delay(10);

  // Test LED light
  pinMode(LED_BUILTIN, OUTPUT);

  // Motor direction pins
  pinMode(DRIVE_IN1, OUTPUT);
  pinMode(DRIVE_IN2, OUTPUT);
  pinMode(DANCE_IN1, OUTPUT);
  pinMode(DANCE_IN2, OUTPUT);

  // Limit switch with internal pull-up
  pinMode(LIMIT_PIN, INPUT_PULLUP);

  // Setup LEDC channels for PWM
  ledcSetup(DRIVE_LEDC_CH, LEDC_FREQ, LEDC_RES);
  ledcAttachPin(DRIVE_EN, DRIVE_LEDC_CH);

  ledcSetup(DANCE_LEDC_CH, LEDC_FREQ, LEDC_RES);
  ledcAttachPin(DANCE_EN, DANCE_LEDC_CH);

  // --- Initialize WiFi and ESP-NOW ---
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
  } else {
    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataRecv);

    // register a broadcast peer so we can send to all
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Failed to add broadcast peer");
    }
  }

  // Initialize motors stopped
  driveStop();
  danceStop();

  // Serial.println("Starting drive_motor control with limit switch...");
  delay(2000); // initial delay

  // Start drive motor moving forward initially (matches original script)
  driveBackward(drive_motor_speed);
}

void loop() {
  bool limitPressed = (digitalRead(LIMIT_PIN) == LOW); // pressed => LOW because of INPUT_PULLUP

  if (limitPressed && !hit_himit) {
    // Serial.println("Limit switch is PRESSED - Stopping drive_motor");
    driveStop();
    hit_himit = true;
    
    if (drive_motor_direction == 1) {
      // Serial.println("Running dance sequence...");
      delay(2000); // brief delay before dance
      danceStart(dance_motor_speed);
      delay(dance_time_ms);
      // Serial.println("Stopping dance sequence.");
      danceStop();
      delay(2000); // brief delay after stopping dance
    } else {
      // Serial.println("Skipping dance sequence on reverse direction.");
      // Wait for ESP-NOW trigger message instead of fixed delay
      receivedDanceTrigger = false;  // Reset flag
      while (!receivedDanceTrigger) {
        delay(100);
      }
    }

    // Reverse direction for next run
    drive_motor_direction *= -1;

    if (drive_motor_direction == 1) {
      driveForward(drive_motor_speed);
    } else {
      // small speed bump like original (+0.05)
      float s = min(drive_motor_speed + 0.05f, 1.0f);
      driveBackward(s);
    }

    delay(2000); // allow motor to move away from limit

  } else if (!limitPressed && hit_himit) {
    // Serial.println("Limit switch is NOT pressed - Resuming drive_motor");
    hit_himit = false;
  }

  delay(100);
}
