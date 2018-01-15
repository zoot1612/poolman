#include <DHT.h>
#include <SPI.h>
#include <Ethernet2.h>
#include <LiquidCrystal_I2C.h>
#include <Time.h>
#include <TimeAlarms.h>
#include <DS1307RTC.h>

void updateChar();
void calibrationUdate();
void serveConnectedClient(EthernetClient client);
void printout(EthernetClient client);

#define DHTPIN 5
#define PRESSUREPIN A2

#define DHTTYPE DHT21
DHT dht(DHTPIN, DHTTYPE);

#define READINTERVAL 30                                           //Readings averaged over seconds interval.
#define INTERVAL 5000                                             //Milliseconds between reads.

tmElements_t tm;

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
const byte LCDLINES = 4;
const byte LCDCHARS = 20;
byte degees[8] = {
  B00110,
  B01001,
  B01001,
  B00110,
  B00000,
  B00000,
  B00000,
};

byte humid[8] = {
  B00100,
  B00100,
  B01010,
  B01010,
  B10001,
  B10001,
  B10001,
  B01110,
};

char lineBuffer[LCDLINES][LCDCHARS + 1];
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
byte ip[] = {192, 168, 0, 233};
byte gateway[] = {192, 168, 0, 1};
byte subnet[] = {255, 255, 255, 0};
EthernetServer server(80);

int sensorValue = 0;
int sensorMin = 1023;
int sensorMax = 0;
int offset = 0;

unsigned long currentMsecs = 0;
unsigned long previousMsecs = 0;

const int numReadings = (READINTERVAL * 1000) / INTERVAL;
int tempArray[numReadings];
int humdArray[numReadings];
int presArray[numReadings];
int readIndex = 0;
boolean bufferReady = false;
long tempTotal = 0;
long humdTotal = 0;
long presTotal = 0;

float tempAverage = 0;
float humdAverage = 0;
int presAverage = 0;

void reset() {
  readIndex = 0;
  bufferReady = false;
  tempTotal = 0;
  humdTotal = 0;
  presTotal = 0;
  tempAverage = 0;
  humdAverage = 0;
  presAverage = 0;
  memset(tempArray, 0, numReadings);
  memset(humdArray, 0, numReadings);
  memset(presArray, 0, numReadings);
}

void setup() {
  Serial.begin(9600);
  calibrationUdate(); //Needs to be changed so it only does a calibration if pump is off.
  dht.begin();
  lcdSetup();
  initRTC();

  if (Ethernet.begin(mac) == 0) {
    Serial.println(F("Failed to configure Ethernet using DHCP using Static IP address"));
    Ethernet.begin(mac, ip, gateway, subnet);
  }
  server.begin();
  Serial.print(F("server is at "));
  Serial.println(Ethernet.localIP());
}

void loop() {
  currentMsecs = millis();
  EthernetClient client = server.available();
  if (client) {
    serveConnectedClient(client);
  }

  if ((currentMsecs - previousMsecs) > INTERVAL) {
    updateChar();
    previousMsecs = currentMsecs;
  }
  displayTime(now());
}

void lcdSetup() {
  lcd.begin(LCDCHARS, LCDLINES);
  lcd.createChar(0, degees);
  lcd.createChar(2, humid);
  for (int i = 0; i < LCDLINES; i++) {
    lineBuffer[i][LCDCHARS + 1] = '\0';
    memset(lineBuffer[i], ' ', LCDCHARS);
  }
  lcd.setCursor(0, 0);
  Serial.println(F("LCD initialised"));
}

void updateChar() {
  float sensorVoltage = analogRead(PRESSUREPIN);

  float h = dht.readHumidity();
  float t = dht.readTemperature();
  float p = sensorVoltage + offset;

  if (isnan(h) || isnan(t)) {
    Serial.println(F("DHT sensor failed"));
    for (int i = 0; i < LCDLINES; i++) {
      memset  (lineBuffer[i], ' ', LCDCHARS);
      lineBuffer[i][LCDCHARS + 1] = '\0';
    }
    lcd.setCursor(0, 1);
    lcd.print("Failed to read DHT sensor!");

  } else {
    tempTotal = tempTotal - tempArray[readIndex];
    humdTotal = humdTotal - humdArray[readIndex];
    presTotal = presTotal - presArray[readIndex];
    tempArray[readIndex] = t * 100;
    humdArray[readIndex] = h * 100;
    presArray[readIndex] = p * 100;
    tempTotal = tempTotal + tempArray[readIndex];
    humdTotal = humdTotal + humdArray[readIndex];
    presTotal = presTotal + presArray[readIndex];
    if (bufferReady) {
      tempAverage = (round(((tempTotal / numReadings) / 100.0f) * 2.0f)) / 2.0f;
      humdAverage = (round(((humdTotal / numReadings) / 100.0f) * 2.0f)) / 2.0f;
      presAverage = (((presTotal / numReadings) / 100) - 102) * 87 / 410;
      Serial.print(F("Temperature    : "));
      Serial.println(tempAverage);
      Serial.print(F("Humidity       : "));
      Serial.println(humdAverage);
      Serial.print(F("Filter Pressure: "));
      Serial.println(presAverage);
      Serial.print(F("Time: "));
      Serial.println(lineBuffer[0]);
      Serial.println("");
      static char outstr[5];
      dtostrf(tempAverage, 4, 1, outstr);
      sprintf(lineBuffer[1], "%sC", outstr);
      dtostrf(humdAverage, 4, 1, outstr);
      strncat(lineBuffer[1], "   ", 3);
      strncat(lineBuffer[1], outstr, 5);
      strncat(lineBuffer[1], "%", 1);
      sprintf(lineBuffer[2],   "Filt Pressure:%02d PSI", presAverage);
      lcd.setCursor(0, 1);
      lcd.print(lineBuffer[1]);
      lcd.setCursor(5, 1);
      lcd.write(byte(0));
      lcd.setCursor(7, 1);
      lcd.write(byte(2));
      lcd.setCursor(0, 2);
      lcd.print(lineBuffer[2]);
    } else {
      if (readIndex == 0) {
        for (int i = 0; i < numReadings; i++) {
          lcd.setCursor(i, 2);
          lcd.write("O");
        }
      }
      lcd.setCursor(0, 1);
      lcd.print(READINTERVAL);
      lcd.print("s ");
      lcd.print("moving average");
      lcd.setCursor(readIndex, 2);
      lcd.write("X");
    }
    readIndex = readIndex + 1;
    if (readIndex >= numReadings) {
      readIndex = 0;
      bufferReady = true;
      lcd.clear();
    }
  }
}

void calibrationUdate() {
  //Check time.
  //Check pump status.
  //If pump is off
  Serial.println(F("Calculating Sensor Offset"));
  while (readIndex < numReadings) {
    float sensorVoltage = analogRead(PRESSUREPIN);
    presTotal = presTotal - presArray[readIndex];
    presArray[readIndex] = sensorVoltage;
    presTotal = presTotal + presArray[readIndex];
    readIndex = readIndex + 1;
  }
  presAverage = (presTotal / numReadings);
  offset = 102 - presAverage;
  Serial.print(F("Sensor Offset is:"));
  Serial.println(offset);
  reset();
}

void serveConnectedClient(EthernetClient client) {
  Serial.println(F("New client connected"));
  boolean currentLineIsBlank = true;
  while (client.connected()) {
    if (client.available()) {
      char c = client.read();
      Serial.write(c);
      if (c == '\n' && currentLineIsBlank) {
        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: text/html");
        client.println("Connection: close");
        client.println("Refresh: 5");
        client.println();
        client.println("<!DOCTYPE HTML>");
        client.println("<html>");
        printout(client);

        client.println("</html>");
        break;
      }
      if (c == '\n') {
        currentLineIsBlank = true;
      } else if (c != '\r') {
        currentLineIsBlank = false;
      }
    }
  }
  delay(1);
  client.stop();
  Serial.println(F("client disconnected"));
}

void printout(EthernetClient client) {
  server.write(lineBuffer[0], 21);
  client.println("<br />");
  server.write(lineBuffer[1], 21);
  client.println("<br />");
  //client.println(" \xB0 <br />");
  server.write(lineBuffer[2], 21);
  client.println("<br />");
  //client.println(" \xB0 <br />");
  server.write(lineBuffer[3], 21);
  client.println("<br />");
}

void displayTime(time_t aTime) {
  tmElements_t tmElem;
  breakTime(aTime, tmElem);
  sprintf(lineBuffer[0], "%02d/%02d/%04d  %02d:%02d:%02d", tmElem.Day, tmElem.Month, tmElem.Year + 1970,
          tmElem.Hour, tmElem.Minute, tmElem.Second);
  lcd.setCursor(0, 0);
  lcd.print(lineBuffer[0]);
}

void initRTC() {
  if (RTC.read(tm)) {
    Serial.println(F("Setting real time clock as sync provider"));
    setSyncProvider(RTC.get);
    if (timeStatus() != timeSet) {
      Serial.println("Unable to sync with the RTC");
    }
    else {
      Serial.println("RTC has set the system time");
    }
  }
  setSyncInterval(60);
}

byte ip[] = {192, 168, 0, 233};
byte gateway[] = {192, 168, 0, 1};
byte subnet[] = {255, 255, 255, 0};
EthernetServer server(80);

int sensorValue = 0;
int sensorMin = 1023;
int sensorMax = 0;
long pressureOffset = 0;

unsigned long currentMsecs = 0;
unsigned long previousMsecs = 0;

const int numReadings = (READINTERVAL * 1000) / INTERVAL;
int tempArray[numReadings];
int humdArray[numReadings];
int presArray[numReadings];
int readIndex = 0;
boolean bufferReady = false;
long tempTotal = 0;
long humdTotal = 0;
long presTotal = 0;

float tempAverage = 0;
float humdAverage = 0;
int presAverage = 0;

void setup() {
  Serial.begin(9600);
  dht.begin();
  lcdSetup();
  initRTC();
  memset(tempArray, 0, numReadings);
  memset(humdArray, 0, numReadings);
  memset(presArray, 0, numReadings);
  if (Ethernet.begin(mac) == 0) {
    Serial.println(F("Failed to configure Ethernet using DHCP using Static IP address"));
    Ethernet.begin(mac, ip, gateway, subnet);
  }
  server.begin();
  Serial.print(F("server is at "));
  Serial.println(Ethernet.localIP());
}

void loop() {
  currentMsecs = millis();
  EthernetClient client = server.available();
  if (client) {
    serveConnectedClient(client);
  }

  if ((currentMsecs - previousMsecs) > INTERVAL) {
    updateChar();
    previousMsecs = currentMsecs;
  }
  displayTime(now());
}

void lcdSetup() {
  lcd.begin(LCDCHARS, LCDLINES);
  lcd.createChar(0, degees);
  lcd.createChar(2, humid);
  for (int i = 0; i < LCDLINES; i++) {
    lineBuffer[i][LCDCHARS + 1] = '\0';
    memset(lineBuffer[i], ' ', LCDCHARS);
  }
  lcd.setCursor(0, 0);
  Serial.println(F("LCD initialised"));
}

void updateChar() {
  float sensorVoltage = analogRead(PRESSUREPIN);
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  int psi = ((sensorVoltage - 102 + pressureOffset) * 87 / 410);

  if (isnan(h) || isnan(t)) {
    Serial.println(F("DHT sensor failed"));
    for (int i = 0; i < LCDLINES; i++) {
      memset  (lineBuffer[i], ' ', LCDCHARS);
      lineBuffer[i][LCDCHARS + 1] = '\0';
    }
    lcd.setCursor(0, 1);
    lcd.print("Failed to read DHT sensor!");

  } else {
    tempTotal = tempTotal - tempArray[readIndex];
    humdTotal = humdTotal - humdArray[readIndex];
    presTotal = presTotal - presArray[readIndex];
    tempArray[readIndex] = t * 100;
    humdArray[readIndex] = h * 100;
    presArray[readIndex] = psi;
    tempTotal = tempTotal + tempArray[readIndex];
    humdTotal = humdTotal + humdArray[readIndex];
    presTotal = presTotal + presArray[readIndex];

    if (bufferReady) {
      tempAverage = (round(((tempTotal / numReadings) / 100.0f) * 2.0f)) / 2.0f;
      humdAverage = (round(((humdTotal / numReadings) / 100.0f) * 2.0f)) / 2.0f;
      presAverage = (presTotal / numReadings);

      Serial.print(F("Temperature: "));
      Serial.println(tempAverage);
      Serial.print(F("Humidity   : "));
      Serial.println(humdAverage);
      //Serial.print(F("Heat Index : "));
      //Serial.println(hIndAverage);
      Serial.print(F("Pressure   : "));
      Serial.println(presAverage);
      Serial.print(F("Time      : "));
      Serial.println(lineBuffer[0]);
      Serial.println("");
      static char outstr[5];
      dtostrf(tempAverage, 4, 1, outstr);
      sprintf(lineBuffer[1], "%sC", outstr);
      dtostrf(humdAverage, 4, 1, outstr);
      strncat(lineBuffer[1], "   ", 3);
      strncat(lineBuffer[1], outstr, 5);
      strncat(lineBuffer[1], "%", 1);
      sprintf(lineBuffer[2],   "Pres:%02d PSI", presAverage);

      lcd.setCursor(0, 1);
      lcd.print(lineBuffer[1]);
      lcd.setCursor(5, 1);
      lcd.write(byte(0));
      lcd.setCursor(7, 1);
      lcd.write(byte(2));
      //lcd.print(lineBuffer[1]);
      //lcd.write(byte(0));
      lcd.setCursor(0, 2);
      lcd.print(lineBuffer[2]);
      //lcd.write(byte(0));
      //lcd.setCursor(0, 3);
      //lcd.print(lineBuffer[3]);
    } else {
      if (readIndex == 0) {
        for (int i = 0; i < numReadings; i++) {
          lcd.setCursor(i, 2);
          lcd.write("O");
        }
      }
      lcd.setCursor(0, 1);
      lcd.print(READINTERVAL);
      lcd.print("s ");
      lcd.print("moving average");
      lcd.setCursor(readIndex, 2);
      lcd.write("X");
    }
    readIndex = readIndex + 1;
    if (readIndex >= numReadings) {
      readIndex = 0;
      bufferReady = true;
      lcd.clear();
    }
  }
}


void calibrationUdate() {
  //Check time.
  //Check pump status.
  //If pump is off
  float sensorVoltage = analogRead(PRESSUREPIN);
  pressureOffset = sensorVoltage - 102;
}

void serveConnectedClient(EthernetClient client) {
  Serial.println(F("New client connected"));
  boolean currentLineIsBlank = true;
  while (client.connected()) {
    if (client.available()) {
      char c = client.read();
      Serial.write(c);
      if (c == '\n' && currentLineIsBlank) {
        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: text/html");
        client.println("Connection: close");
        client.println("Refresh: 5");
        client.println();
        client.println("<!DOCTYPE HTML>");
        client.println("<html>");
        printout(client);

        client.println("</html>");
        break;
      }
      if (c == '\n') {
        currentLineIsBlank = true;
      } else if (c != '\r') {
        currentLineIsBlank = false;
      }
    }
  }
  delay(1);
  client.stop();
  Serial.println(F("client disconnected"));
}

void printout(EthernetClient client) {
  server.write(lineBuffer[0], 21);
  client.println("<br />");
  server.write(lineBuffer[1], 21);
  client.println("<br />");
  //client.println(" \xB0 <br />");
  server.write(lineBuffer[2], 21);
  client.println("<br />");
  //client.println(" \xB0 <br />");
  server.write(lineBuffer[3], 21);
  client.println("<br />");
}

void displayTime(time_t aTime) {
  tmElements_t tmElem;
  breakTime(aTime, tmElem);
  sprintf(lineBuffer[0], "%02d/%02d/%04d  %02d:%02d:%02d", tmElem.Day, tmElem.Month, tmElem.Year + 1970,
          tmElem.Hour, tmElem.Minute, tmElem.Second);
  lcd.setCursor(0, 0);
  lcd.print(lineBuffer[0]);
}

void initRTC() {
  if (RTC.read(tm)) {
    Serial.println(F("Setting real time clock as sync provider"));
    setSyncProvider(RTC.get);
    if (timeStatus() != timeSet) {
      Serial.println("Unable to sync with the RTC");
    }
    else {
      Serial.println("RTC has set the system time");
    }
  }
  setSyncInterval(60);
}
