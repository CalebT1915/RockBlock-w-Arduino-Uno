#include <IridiumSBD.h> // Click here to get the library: http://librarymanager/All#IridiumSBDI2C
#include <TinyGPS++.h> // NMEA parsing: http://arduiniana.org
#include <SoftwareSerial.h>
/*
 * Beacon
 *
 * This sketch shows how you might use a GPS with the satellite modem
 * to create a beacon device that periodically transmits a location
 * message to the configured endpoints.
 *
 * Assumptions
 *
 * The sketch assumes an Arduino Mega or other Arduino-like device with
 * multiple HardwareSerial ports.  It assumes the satellite modem is
 * connected to Serial1.  Change this as needed.  SoftwareSerial on an Uno
 * works fine as well.  This assumes a 9600 baud GPS is connected to Serial2
 */

#define Iridium_RX 10
#define Iridium_TX 11
SoftwareSerial IridiumSerial(Iridium_RX,Iridium_TX);
#define Arduino_GPS_RX 5
#define Arduino_GPS_TX 6 
SoftwareSerial ssGPS(Arduino_GPS_RX,Arduino_GPS_TX);
#define GPSBaud 9600
#define DIAGNOSTICS true // Change this to see diagnostics

// Time between transmissions (seconds)
#define BEACON_INTERVAL 360

IridiumSBD modem(IridiumSerial);
TinyGPSPlus tinygps;


void setup()
{
  int signalQuality = -1;
  int err;
  // Start the serial ports
  Serial.begin(115200);
  while (!Serial);
  IridiumSerial.begin(19200);
  modem.setPowerProfile(IridiumSBD::DEFAULT_POWER_PROFILE);
  
  Serial.println("Starting modem...");
  err = modem.begin();
  if (err != ISBD_SUCCESS)
  {
    Serial.print("Begin failed: error ");
    Serial.println(err);
    if (err == ISBD_NO_MODEM_DETECTED)
      Serial.println("No modem detected: check wiring.");
    return;
  }
  err = modem.getSignalQuality(signalQuality);
  if (err != ISBD_SUCCESS)
  {
    Serial.print("SignalQuality failed: error ");
    Serial.println(err);
    return;
  }

  Serial.print("On a scale of 0 to 5, signal quality is currently ");
  Serial.print(signalQuality);
  Serial.println(".");
  ssGPS.begin(GPSBaud);
  
}

void loop()
{
  int err; 
  unsigned long loopStartTime = millis();

  // Begin listening to the GPS
  Serial.println(F("Beginning to listen for GPS traffic..."));

  // Look for GPS signal for up to 7 minutes
  while ((!tinygps.location.isValid() || !tinygps.date.isValid()) &&
    millis() - loopStartTime < 7UL * 60UL * 1000UL)
  {
    if (ssGPS.available())
      tinygps.encode(ssGPS.read());
  }

  // Did we get a GPS fix?
  if (!tinygps.location.isValid())
  {
    Serial.println(F("Could not get GPS fix."));
    Serial.print(F("GPS characters seen = "));
    Serial.println(tinygps.charsProcessed());
    Serial.print(F("Checksum errors = "));
    Serial.println(tinygps.failedChecksum());
    return;
  }

  Serial.println(F("A GPS fix was found!"));

  // Step 3: Start talking to the RockBLOCK and power it up
  Serial.println(F("Beginning to talk to the RockBLOCK..."));
  char outBuffer[60]; // Always try to keep message short
  sprintf(outBuffer, "%s%u.%09lu,%s%u.%09lu",
    tinygps.location.rawLat().negative ? "-" : "",
    tinygps.location.rawLat().deg,
    tinygps.location.rawLat().billionths,
    tinygps.location.rawLng().negative ? "-" : "",
    tinygps.location.rawLng().deg,
    tinygps.location.rawLng().billionths);

    Serial.print(F("Transmitting message '"));
    Serial.print(outBuffer);
    Serial.println(F("'"));

  err = modem.sendSBDText(outBuffer);
  if (err != ISBD_SUCCESS)
  {
    Serial.print("sendSBDText failed: error ");
    Serial.println(err);
    if (err == ISBD_SENDRECEIVE_TIMEOUT)
      Serial.println("Try again with a better view of the sky.");
  }

  // Sleep
  int elapsedSeconds = (int)((millis() - loopStartTime) / 1000);
  if (elapsedSeconds < BEACON_INTERVAL)
  {
    int delaySeconds = BEACON_INTERVAL - elapsedSeconds;
    Serial.print(F("Waiting for "));
    Serial.print(delaySeconds);
    Serial.println(F(" seconds"));
    delay(1000UL * delaySeconds);
  }

  // Wake
  Serial.println(F("Wake up!"));
}

// void blinkLED()
// {
  // digitalWrite(ledPin, (millis() / 1000) % 2 == 1 ? HIGH : LOW);
// }

// bool ISBDCallback()
// {
//  blinkLED();
//  return true;
//}

#if DIAGNOSTICS
void ISBDConsoleCallback(IridiumSBD *device, char c)
{
  Serial.write(c);
}

void ISBDDiagsCallback(IridiumSBD *device, char c)
{
  Serial.write(c);
}
#endif

