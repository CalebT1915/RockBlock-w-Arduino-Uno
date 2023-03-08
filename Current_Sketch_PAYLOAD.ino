#include <IridiumSBD.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h> // NMEA parsing: http://arduiniana.org
#include <PString.h> // String buffer formatting: http://arduiniana.org

#define BEACON_INTERVAL 600 // Time between transmissions
#define ROCKBLOCK_RX_PIN 10
#define ROCKBLOCK_TX_PIN 11
#define ROCKBLOCK_SLEEP_PIN 14
#define ROCKBLOCK_BAUD 19200
#define GPS_RX_PIN 5
#define GPS_TX_PIN 6
#define GPS_BAUD 9600
#define CONSOLE_BAUD 115200
#define DIAGNOSTICS true // Change this to see diagnostics

SoftwareSerial ssIridium(ROCKBLOCK_RX_PIN, ROCKBLOCK_TX_PIN);
SoftwareSerial ssGPS(GPS_RX_PIN, GPS_TX_PIN);
IridiumSBD isbd(ssIridium, ROCKBLOCK_SLEEP_PIN);
TinyGPSPlus tinygps;

void setup()
{
  // Start the serial ports
  Serial.begin(CONSOLE_BAUD);

  // Setup the RockBLOCK

  isbd.setPowerProfile(1);
}

void loop()
{
  bool fixFound = false;
  unsigned long loopStartTime = millis();

  // Step 0: Start the serial ports
  ssIridium.begin(ROCKBLOCK_BAUD);
  ssGPS.begin(GPS_BAUD);

  // Step 1: Reset TinyGPS++ and begin listening to the GPS
  Serial.println("Beginning to listen for GPS traffic...");
  tinygps = TinyGPSPlus();
  ssGPS.listen();

  // Step 2: Look for GPS signal for up to 7 minutes
  for (unsigned long now = millis(); !fixFound && millis() - now < 7UL * 60UL * 1000UL;)
    if (ssGPS.available())
    {
      tinygps.encode(ssGPS.read());
      fixFound = tinygps.location.isValid() && tinygps.date.isValid() &&
        tinygps.time.isValid() && tinygps.altitude.isValid();
    }

  Serial.println(fixFound ? F("A GPS fix was found!") : F("No GPS fix was found."));

  // Step 3: Start talking to the RockBLOCK and power it up
  Serial.println("Beginning to talk to the RockBLOCK...");
  ssIridium.listen();
  if (isbd.begin() == ISBD_SUCCESS)
  {
    char outBuffer[60]; // Always try to keep message short
    if (fixFound)
    {
      sprintf(outBuffer, "%d%02d%02d%02d%02d%02d,",
        tinygps.date.year(), tinygps.date.month(), tinygps.date.day(),
        tinygps.time.hour(), tinygps.time.minute(), tinygps.time.second());
      int len = strlen(outBuffer);
      PString str(outBuffer, sizeof(outBuffer) - len);
      str.print(tinygps.location.lat(), 6);
      str.print(",");
      str.print(tinygps.location.lng(), 6);
      str.print(",");
      str.print(tinygps.altitude.meters());
      str.print(",");
      str.print(tinygps.speed.knots(), 1);
      str.print(",");
      str.print(tinygps.course.value() / 100);
    }
    else
    {
      sprintf(outBuffer, "No GPS fix found!");
    }

    Serial.print("Transmitting message: ");
    Serial.println(outBuffer);
    isbd.sendSBDText(outBuffer);
  }

  // Sleep
  Serial.println("Going to sleep mode for about 10 minutes...");
  isbd.sleep();
  ssIridium.end();
  ssGPS.end();
  int elapsedSeconds = (int)((millis() - loopStartTime) / 1000);
  while (elapsedSeconds++ < BEACON_INTERVAL)
    delay(1000);
}