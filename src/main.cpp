/*
  main.cpp

  Created by Ian Thompson on Wed Jun 29 2022
  ianthompson@nicelion.com
  https://www.nicelion.com


 */

#include <M5StickC.h>
#include <Arduino.h>
#include <assert.h>
#include <IRrecv.h>
#include <IRremoteESP8266.h>
#include <IRac.h>
#include <IRtext.h>
#include <IRutils.h>
#include <WiFi.h>
#include "network.h"

const bool DEBUG = true;

// Sending Code
const uint16_t kIrLed = M5_IR; // ESP8266 GPIO pin to use. Recommended: 4 (D2).

IRsend irsend(kIrLed); // Set the GPIO to be used to sending the message.

// Data captured by IRrecvDumpV2.ino
uint16_t onOffCommand[71] = {9054, 4510, 570, 562, 568, 562, 570, 1698, 570, 560, 568, 564, 568, 564, 566, 566, 566, 562, 570, 1690, 572, 1690, 572, 562, 572, 1696, 566, 1696, 568, 1694, 570, 1694, 568, 1698, 566, 564, 570, 560, 570, 562, 568, 1696, 566, 562, 570, 564, 568, 562, 570, 564, 566, 1696, 568, 1696, 570, 1690, 574, 560, 570, 1696, 566, 1692, 572, 1688, 574, 1688, 572, 39234, 9048, 2256, 570};

// Receiving Setup
const uint16_t kRecvPin = G26;

const uint32_t kBaudRate = 115200;
const uint16_t kCaptureBufferSize = 1024;

const uint16_t kMinUnknownSize = 12;
const uint8_t kTolerancePercentage = kTolerance; // kTolerance is normally 25%

// Use turn on the save buffer feature for more complete capture coverage.
IRrecv irrecv(kRecvPin, kCaptureBufferSize, 50, true); // or 15
decode_results results;                                // Somewhere to store the results

// Wifi SSID and password
// const char *networkSSID = SSID;
// const char *networkPass = password;

const uint ServerPort = 5457;
WiFiServer Server(ServerPort);
WiFiClient RemoteClient;
bool networkConnected = false;

void setupIR()
{
  // Perform a low level sanity checks that the compiler performs bit field
  // packing as we expect and Endianness is as we expect.
  assert(irutils::lowLevelSanityCheck() == 0);

  Serial.printf("\n" D_STR_IRRECVDUMP_STARTUP "\n", kRecvPin);

#if DECODE_HASH
  // Ignore messages with less than minimum on or off pulses.
  irrecv.setUnknownThreshold(kMinUnknownSize);
#endif                                       // DECODE_HASH
  irrecv.setTolerance(kTolerancePercentage); // Override the default tolerance.
  irrecv.enableIRIn();                       // Start the receiver
}

void WiFiEvent(WiFiEvent_t event)
{
  switch (event)
  {
  case SYSTEM_EVENT_STA_GOT_IP:
    M5.Lcd.setTextSize(2);
    M5.Lcd.println(WiFi.localIP());
    Serial.println(WiFi.localIP());
    networkConnected = true;
    break;
  case SYSTEM_EVENT_STA_DISCONNECTED:
    M5.Lcd.println("Network connection lost!");
    M5.Lcd.println("Restarting...");
    Serial.println("Network connection lost! Restarting...");
    delay(2000);
    ESP.restart();
    networkConnected = false;
    break;
  }
}

void ConnectToNetwork()
{
  M5.Lcd.println("Connecting to SSID: " + String(networkSSID));

  WiFi.disconnect(true);
  WiFi.onEvent(WiFiEvent);

  WiFi.mode(WIFI_STA); // station
  WiFi.setSleep(false);

  WiFi.begin(networkSSID, networkPass);
}
void setup()
{
  M5.begin();
  M5.Lcd.setRotation(3);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.setTextSize(1);

  pinMode(kIrLed, OUTPUT);

  irsend.begin();

  Serial.begin(115200, SERIAL_8N1);
  Serial.begin(kBaudRate, SERIAL_8N1);

  ConnectToNetwork(); // starts Wifi connection
  while (!networkConnected)
  {
    delay(200);
  }

  while (!Serial) // Wait for the serial connection to be establised.
    delay(50);

  setupIR();

  Server.begin();
}

void sendIRSignal()
{
  Serial.println("Sending IR Signal");
  irsend.sendRaw(onOffCommand, 71, 38); // Send a raw data capture at 38kHz.
  delay(2000);
  Serial.println("Signal Sent!");
}

void handleReceiving()
{
  // Check if the IR code has been received.
  if (irrecv.decode(&results))
  {
    // Display a crude timestamp.
    uint32_t now = millis();
    Serial.printf(D_STR_TIMESTAMP " : %06u.%03u\n", now / 1000, now % 1000);
    // Check if we got an IR message that was to big for our capture buffer.
    if (results.overflow)
      Serial.printf(D_WARN_BUFFERFULL "\n", kCaptureBufferSize);
    // Display the library version the message was captured with.
    Serial.println(D_STR_LIBRARY "   : v" _IRREMOTEESP8266_VERSION_STR "\n");
    // Display the tolerance percentage if it has been change from the default.
    if (kTolerancePercentage != kTolerance)
      Serial.printf(D_STR_TOLERANCE " : %d%%\n", kTolerancePercentage);
    // Display the basic output of what we found.
    Serial.print(resultToHumanReadableBasic(&results));
    // Display any extra A/C info if we have it.
    String description = IRAcUtils::resultAcToString(&results);
    if (description.length())
      Serial.println(D_STR_MESGDESC ": " + description);
    yield(); // Feed the WDT as the text output can take a while to print.
    // Output the results as source code
    Serial.println(resultToSourceCode(&results));
    Serial.println(); // Blank line between entries
    yield();          // Feed the WDT (again)
  }
}

void CheckForConnections()
{
  if (Server.hasClient())
  {
    // If we are already connected to another computer,
    // then reject the new connection. Otherwise accept
    // the connection.
    if (RemoteClient.connected())
    {
      Serial.println("Connection rejected");
      Server.available().stop();
    }
    else
    {
      Serial.println("Connection accepted");
      RemoteClient = Server.available();
    }
  }
}

String CheckMessage(String message)
{
  String response = "Invalid command.";

  Serial.println("Checking command: " + message);
  if (message == "on")
  {
    sendIRSignal();
  }
  else
  {
    Serial.println("Invalid command. Valid commands: on");
    response = "Invalid command";
  }

  return message;
}

void ReceiveData()
{
  while (RemoteClient.connected() && RemoteClient.available())
  {
    String message = RemoteClient.readStringUntil('\n');
    message.trim();
    String response = CheckMessage(message);
    Serial.println(response);
    char responseCharArray[response.length() + 1];
    strcpy(responseCharArray, response.c_str());
    RemoteClient.write(responseCharArray);
  }
}

void loop()
{
  CheckForConnections();
  ReceiveData();
  // handleEmitting();
  // handleReceiving();
}
