#include <Adafruit_GPS.h>
#include <HardwareSerial.h>

const int buzzerPin = 5; 
const int PushButton = 4;
unsigned long buzzStartTime = 0;  //tracks when the buzzing starts
const unsigned long BuzzDuration = 5000;  //5 seconds in milliseconds (can be changed)
bool isBuzzing = false;  //tracks if we're currently in a buzz cycle

// Create a HardwareSerial instance for UART1 on the ESP32-C3.
HardwareSerial GPSSerial(1);  // Use UART1

// Connect the GPSSerial instance to the Adafruit_GPS library.
Adafruit_GPS GPS(&GPSSerial);

uint32_t timer = millis();

void setup() {
  // Start the Serial Monitor for debugging.
  Serial.begin(115200);
  
  pinMode(buzzerPin, OUTPUT); //makes pin5 output
  pinMode(PushButton, INPUT); //makes pin4 input

  // Serial.println("ESP32-C3 GPS Debugging using UART1: Printing parsed data");

  // Initialize UART1 for the GPS module.
  // Change RX and TX pins as needed. Here, RX is GPIO7 and TX is GPIO8.
  GPSSerial.begin(9600, SERIAL_8N1, 2, 1);

  // Initialize the GPS module at 9600 baud.
  GPS.begin(9600);
  
  // Configure the GPS to output RMC and GGA sentences.
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  
  // Set the update rate to 1 Hz (once per second).
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  
  // (Optional) Request updates on antenna status.
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);

  // Request firmware version (optional).
  GPSSerial.println(PMTK_Q_RELEASE);
}

void loop() {
  // Read any incoming data from the GPS module.
  char c = GPS.read();

  // When a full NMEA sentence is received, parse it.
  if (GPS.newNMEAreceived()) {
    char* nmeaSentence = GPS.lastNMEA();
    GPS.parse(nmeaSentence);
  }

  // Every 2 seconds, print the parsed GPS data.
  if (millis() - timer > 2000) {
    timer = millis();

    // Print Time in hh:mm:ss.mmm format.
    Serial.print("Time: ");
    if (GPS.hour < 10) Serial.print('0');
    Serial.print(GPS.hour); Serial.print(":");
    if (GPS.minute < 10) Serial.print('0');
    Serial.print(GPS.minute); Serial.print(":");
    if (GPS.seconds < 10) Serial.print('0');
    Serial.print(GPS.seconds); Serial.print(".");
    Serial.println(GPS.milliseconds);

    // Print Latitude.
    Serial.print("Latitude: ");
    if (GPS.latitude) {
      Serial.print(GPS.latitude, 4);
      Serial.print(" ");
      Serial.println(GPS.lat);  // 'N' or 'S'
    } else {
      Serial.println("no data");
    }

    // Print Longitude.
    Serial.print("Longitude: ");
    if (GPS.longitude) {
      Serial.print(GPS.longitude, 4);
      Serial.print(" ");
      Serial.println(GPS.lon);    // 'E' or 'W'
    } else {
      Serial.println("no data");
    }

    // Print Altitude.
    Serial.print("Altitude: ");
    if (GPS.fix) {
      Serial.println(GPS.altitude);
    } else {
      Serial.println("no data");
    }

    // Print Speed (in knots).
    Serial.print("Speed (knots): ");
    if (GPS.fix) {
      Serial.println(GPS.speed);
    } else {
      Serial.println("no data");
    }

    // Print Satellites.
    Serial.print("Satellites: ");
    if (GPS.fix) {
      Serial.println((int)GPS.satellites);
    } else {
      Serial.println("no data");
    }

    Serial.println();  // Blank line for readability
  }

  int ButtonState = digitalRead(PushButton);

  if (ButtonState == true && !isBuzzing){ //this if checks if button is pressed and buzzer is not buzzing
    buzzStartTime = millis();
    isBuzzing = true;
  }

  if (isBuzzing) { 
    if ((millis() - buzzStartTime) < BuzzDuration){ //this if statement checks if it's still in the 5second window
      // Create a square wave for buzzer tone
      digitalWrite(buzzerPin, HIGH);
      delayMicroseconds(1000); // For ~500Hz tone
      digitalWrite(buzzerPin, LOW);
      delayMicroseconds(1000);
    }
    else {
      digitalWrite(buzzerPin, LOW);
      isBuzzing = false;
    }
  }
}


