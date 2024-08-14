#include "Arduino.h"
#include "DFRobotDFPlayerMini.h"
#include <SoftwareSerial.h>

// Define pins for Software Serial and MOSFET
#define RX_PIN 10
#define TX_PIN 11
#define MOSFET_RELAY 3
#define MOSFET_MAGNET 6
#define BUTTON_ENTRANCE 5
#define BUTTON_EXIT 4

int playDuration = 30000;  // Duration for which the relay and magnet are activated (in milliseconds)
int debounceDelay = 50;    // Debounce time in milliseconds

SoftwareSerial softSerial(RX_PIN, TX_PIN);
#define FPSerial softSerial

DFRobotDFPlayerMini myDFPlayer;

void printDetail(uint8_t type, int value);

void setup() {
  pinMode(MOSFET_RELAY, OUTPUT);
  pinMode(MOSFET_MAGNET, OUTPUT);
  pinMode(BUTTON_ENTRANCE, INPUT_PULLUP);
  pinMode(BUTTON_EXIT, INPUT_PULLUP);

  digitalWrite(MOSFET_MAGNET, HIGH);  // Magnet initially energized
  digitalWrite(MOSFET_RELAY, LOW);    // Relay initially off

  FPSerial.begin(9600);
  Serial.begin(115200);

  Serial.println();
  Serial.println(F("DFRobot DFPlayer Mini Demo"));
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));

  if (!myDFPlayer.begin(FPSerial, true, true)) {  // Use serial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1. Please recheck the connection!"));
    Serial.println(F("2. Please insert the SD card!"));
    while (true) {
      delay(0); // Code to compatible with ESP8266 watch dog.
    }
  }
  Serial.println(F("DFPlayer Mini online."));

  myDFPlayer.volume(20);  // Set volume value. From 0 to 30
  Serial.println(F("Initial setup complete. Magnet is ON, Relay is OFF."));
}

void loop() {
  // Check if the entrance button is pressed
  if (isButtonPressed(BUTTON_ENTRANCE)) {
    Serial.println(F("Entrance button pressed"));
    handleEntranceButton();
  }

  // Check if the exit button is pressed
  if (isButtonPressed(BUTTON_EXIT)) {
    Serial.println(F("Exit button pressed"));
    handleExitButton();
  }

  // Check for messages from DFPlayer
  if (myDFPlayer.available()) {
    printDetail(myDFPlayer.readType(), myDFPlayer.read()); // Print the detail message from DFPlayer to handle different errors and states.
  }
}

bool isButtonPressed(int buttonPin) {
  Serial.print(F("Checking button on pin "));
  Serial.println(buttonPin);
  if (digitalRead(buttonPin) == LOW) {  // Check if button is initially pressed
    Serial.println(F("Button pressed, waiting for debounce..."));
    delay(debounceDelay);               // Wait for debounce delay
    if (digitalRead(buttonPin) == LOW) {  // Check again if button is still pressed
      Serial.println(F("Button press confirmed after debounce"));
      return true;
    } else {
      Serial.println(F("Button press was noise, ignoring..."));
    }
  }
  return false;
}

void handleEntranceButton() {
  Serial.println(F("Handling entrance button action: Deactivating magnet, activating relay, and playing audio."));
  // Deactivate magnet, activate relay, and start playing audio
  digitalWrite(MOSFET_MAGNET, LOW);   // Deactivate magnet
  digitalWrite(MOSFET_RELAY, HIGH);   // Activate relay
  myDFPlayer.play(1);                 // Start playing the first mp3

  Serial.println(F("Audio playing, waiting for duration..."));
  delay(playDuration);                // Wait for the duration of the play

  // Revert to the initial state
  Serial.println(F("Reverting to initial state: Reactivating magnet, deactivating relay."));
  digitalWrite(MOSFET_MAGNET, HIGH);  // Reactivate magnet
  digitalWrite(MOSFET_RELAY, LOW);    // Deactivate relay
  myDFPlayer.stop();                  // Stop playback
  Serial.println(F("Entrance button action completed."));
}

void handleExitButton() {
  Serial.println(F("Handling exit button action: Deactivating magnet permanently until reset."));
  // Deactivate the magnet permanently (until reset)
  digitalWrite(MOSFET_MAGNET, LOW);   // Deactivate magnet
  digitalWrite(MOSFET_RELAY, LOW);    // Ensure relay is deactivated
  myDFPlayer.stop();                  // Stop playback (if any)
  Serial.println(F("Magnet deactivated, controller will stay in this state until reset."));
  
  while (true) {
    delay(1000);
  }
}

void printDetail(uint8_t type, int value) {
  Serial.print(F("DFPlayer event: "));
  switch (type) {
    case TimeOut:
      Serial.println(F("Time Out!"));
      break;
    case WrongStack:
      Serial.println(F("Stack Wrong!"));
      break;
    case DFPlayerCardInserted:
      Serial.println(F("Card Inserted!"));
      break;
    case DFPlayerCardRemoved:
      Serial.println(F("Card Removed!"));
      break;
    case DFPlayerCardOnline:
      Serial.println(F("Card Online!"));
      break;
    case DFPlayerUSBInserted:
      Serial.println("USB Inserted!");
      break;
    case DFPlayerUSBRemoved:
      Serial.println("USB Removed!");
      break;
    case DFPlayerPlayFinished:
      Serial.print(F("Number:"));
      Serial.print(value);
      Serial.println(F(" Play Finished!"));
      break;
    case DFPlayerError:
      Serial.print(F("DFPlayerError: "));
      switch (value) {
        case Busy:
          Serial.println(F("Card not found"));
          break;
        case Sleeping:
          Serial.println(F("Sleeping"));
          break;
        case SerialWrongStack:
          Serial.println(F("Get Wrong Stack"));
          break;
        case CheckSumNotMatch:
          Serial.println(F("Check Sum Not Match"));
          break;
        case FileIndexOut:
          Serial.println(F("File Index Out of Bound"));
          break;
        case FileMismatch:
          Serial.println(F("Cannot Find File"));
          break;
        case Advertise:
          Serial.println(F("In Advertise"));
          break;
        default:
          Serial.println(F("Unknown error"));
          break;
      }
      break;
    default:
      Serial.println(F("Unknown event"));
      break;
  }
}
