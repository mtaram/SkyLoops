#include <FastLED.h>
#include <esp_now.h>
#include <WiFi.h>

#define LED_PIN 16  // Data pin for WS2812B LEDs
#define NUM_LEDS 90
#define BUZZER_PIN 4  // Number of LEDs on your ring

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

typedef struct struct_message {
  char a[32];
  int b;
  float c;
  bool d;
} struct_message;

// Create a struct_message called myData
struct_message myData;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}


const int trigPin1 = 27;  // Use any available GPIO pin
const int echoPin1 = 13;  // Use any available GPIO pin
int melody[] = { 660, 1320, 1760, 3520, 3135, 2794 };

// Note durations (in milliseconds)
int noteDurations[] = { 100, 100, 100, 100, 100, 300 };
// Define pins for the second ultrasonic sensor
const int trigPin2 = 14;  // Use any available GPIO pin
const int echoPin2 = 18;  // Use any available GPIO pin

// Variables for storing the distance readings
long duration1, distance1;
long duration2, distance2;


// Timing variables
unsigned long previousMillis1 = 0;  // Last time the first sensor was triggered
unsigned long previousMillis2 = 0;  // Last time the second sensor was triggered
const long sensorDelay = 100;       // Delay to ensure pulses don't interfere
CRGB leds[NUM_LEDS];

void setup() {
  delay(3000);
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

    FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
    fill(CRGB::Black);
    FastLED.show();
    // Initialize serial communication


    // Set the trig and echo pins as outputs and inputs
    pinMode(trigPin1, OUTPUT);
    pinMode(echoPin1, INPUT);
    pinMode(trigPin2, OUTPUT);
    pinMode(echoPin2, INPUT);
    pinMode(BUZZER_PIN, OUTPUT);  // Set buzzer pin as output
    digitalWrite(BUZZER_PIN, LOW);
  }
  void fill(CRGB color) {
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = color;
    }
  }
  void loop() {
        unsigned long currentMillis = millis();

    // Trigger Sensor 1, ensuring enough time between readings
    if (currentMillis - previousMillis1 >= sensorDelay) {
      previousMillis1 = currentMillis;
      distance1 = getDistance(trigPin1, echoPin1);
      Serial.print("Sensor 1 Distance: ");
      Serial.print(distance1);
      Serial.print(" cm  ");

      // Small delay to avoid interference
      delay(50);
    }

    // Trigger Sensor 2 after Sensor 1, with a longer interval to avoid interference
    if (currentMillis - previousMillis2 >= sensorDelay + 50) {  // 50ms extra delay for safety
      previousMillis2 = currentMillis;
      distance2 = getDistance(trigPin2, echoPin2);
      Serial.print("Sensor 2 Distance: ");
      Serial.print(distance2);
      Serial.println(" cm");

      // Small delay to avoid interference
      delay(50);
    }
    if ((distance1 > 0 && distance1 < 32) || (distance2 > 0 && distance2 < 32)) {
      changeLEDColor();
      senddata();

    } else {
      resetLEDs();
    }

    delay(10);
  }
  void changeLEDColor() {
    fill(CRGB::Green);
    FastLED.show();
    playTune();
    delay(1000);
  }

  void resetLEDs() {
    fill(CRGB::Blue);
    deactivateBuzzer();
    FastLED.show();
  }

  // Function to get distance from the ultrasonic sensor
  long getDistance(int trigPin, int echoPin) {
    // Clear the trmigPin
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);

    // Set the trigPin HIGH for 10 microseconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Read the echoPin, returns the time in microseconds
    long duration = pulseIn(echoPin, HIGH, 30000);  // 30ms timeout for distant objects

    // Calculate the distance (speed of sound is 0.034 cm/microsecond)
    long distance = duration * 0.034 / 2;

    // Return -1 if timeout occurs (no valid signal)
    if (duration == 0) {
      return -1;  // Signal didn't return, object too far or interference
    }

    return distance;
  }
  void playTune() {
    for (int thisNote = 0; thisNote < 8; thisNote++) {
      int noteDuration = noteDurations[thisNote];
      tone(BUZZER_PIN, melody[thisNote], noteDuration);  // Play the note
      delay(noteDuration * 1.3);                         // Slight delay between notes
    }
    noTone(BUZZER_PIN);  // Turn off the buzzer after the melody
  }

  // Function to deactivate the buzzer
  void deactivateBuzzer() {
    noTone(BUZZER_PIN);  // Ensure buzzer is off
  }

  void senddata() {
  // Set values to send
  strcpy(myData.a, "this works like a charm");
  myData.b = random(1,20);
  myData.c = 1.2;
  myData.d = false;
  
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  delay(2000);
}