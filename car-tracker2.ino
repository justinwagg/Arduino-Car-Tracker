#include "Adafruit_FONA.h"
#include <SoftwareSerial.h>
#include "LowPower.h"

#define FONA_RX 3
#define FONA_TX 4
#define FONA_RST 5
#define FONA_PWR 6
#define FONA_PWR_STAT 9

// led
int redPin = 12;
int greenPin = 11;
int bluePin = 10;
int ledState = LOW;
unsigned long onMillis;
unsigned long offMillis;
const long led_interval = 2500;

int error_level = 0;

const int voltage_pin = A1;
float last_voltage;

bool startShutdown = false;
unsigned long shutdown_start_millis;
unsigned long last_check;


boolean have_valid_location = false;
bool have_message = false;
uint16_t vbat;

char sendto[21] = "";
char message[80];

SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

void setup() {

  Serial.begin(9600);

  // led
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  pinMode(FONA_PWR_STAT, INPUT);
  digitalWrite(FONA_PWR_STAT, HIGH);

  pinMode(FONA_PWR, OUTPUT);

  // If the FONA is on, turn it off.
  if (digitalRead(FONA_PWR_STAT)) {
    Serial.println(F("BOOT STATUS: FONA is ON, Turning FONA OFF."));
    fona_toggle_pwr(false);
  }
  else {
    Serial.println(F("BOOT STATUS: FONA is already OFF."));
  }


  last_voltage = (float)(analogRead(voltage_pin) / 1024.0) * 3.3;
  last_check = millis();
}

void loop() {

  if (digitalRead(FONA_PWR_STAT) && startShutdown == false) {
    Serial.println(F("LOOP STATUS: FONA is ON, Turning FONA OFF."));
    fona_toggle_pwr(false);
  }

  get_voltage();
  status_led(error_level);

  if (startShutdown) {
    create_message();
    if (have_message) {
      send_message();
      go_to_sleep();
    }
  }


}

bool send_message() {

  bool message_sent = false;
  int attempts = 3;
  int x = 0;
  do {
    if (!fona.sendSMS(sendto, message)) {
      Serial.println(F("SMS Failed"));
      message_sent = false;
      error_level = 2;
      ++x;
    } else {
      Serial.println(F("SMS Sent!"));
      message_sent = true;
      error_level = 0;
      break;
    }
    delay(100);
  }
  while (x <= attempts);
  return message_sent;
}

void create_message() {

  if (!digitalRead(FONA_PWR_STAT)) {
    Serial.println(F("FONA BOOT STATUS: FONA is off."));
    fona_toggle_pwr(true);
  }
  else {
    //    Serial.println(F("FONA BOOT STATUS: FONA is on."));
  }

  if (millis() - last_check > 1000) {

    float latitude, longitude, speed_kph, heading, speed_mph, altitude;
    have_valid_location = fona.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude);
    char LAT[10];
    char LONG[10];

    if (have_valid_location) {
      dtostrf(latitude, 8, 6, LAT);
      dtostrf(longitude, 8, 6, LONG);
    } else {
      Serial.println(F("Waiting for FONA GPS 3D fix..."));
    }

    // Get battery %
    bool have_battery_pct = fona.getBattPercent(&vbat);
    if (have_battery_pct && have_valid_location) {
      sprintf(message, "Location:\nmaps.google.com/?q=""%s,%s""\nBatt: ""%u""%%", LAT, LONG, vbat);
      Serial.println(message);
      have_message = true;
    } else {
      have_message = false;
    }
    last_check = millis();
  }

  if (have_message && have_valid_location) {
    error_level = 0;
  }
  else {
    error_level = 1;
  }

}

void fona_toggle_pwr(bool booting) {

  Serial.println(F("Toggling FONA power."));

  digitalWrite(FONA_PWR, HIGH);
  delay(2000);
  digitalWrite(FONA_PWR, LOW);
  delay(2000);
  digitalWrite(FONA_PWR, HIGH);


  if (booting) {
    fonaSerial->begin(4800);
    if (! fona.begin(*fonaSerial)) {
      Serial.println(F("Couldn't find FONA"));
      error_level = 2;
      while (1);
    }
    Serial.println(F("FONA is OK"));
    Serial.println(F("Boot starting GPS"));
    fona.enableGPS(true);
  }


}


static void get_voltage() {

  float current_voltage = analogRead(voltage_pin);
  current_voltage = (float)(current_voltage / 1024.0) * 3.3;
  //  Serial.println(current_voltage);
  if ( current_voltage < .5 && last_voltage >= .5 ) {
    startShutdown = true;
    shutdown_start_millis = millis();
    //    last_voltage = current_voltage;
    //    Serial.println(F("Hit shutdown"));
  }
  else {
    last_voltage = current_voltage;
    startShutdown = false;
  }

}

void go_to_sleep() {

  setColor(0, 0, 0);
  Serial.println(F("go_to_sleep function:"));
  attachInterrupt(0, wakeUp, RISING);
  fona_toggle_pwr(false);
  Serial.println(F("Shutdown FONA - Delaying Arduino Sleep for 5 Seconds"));
  counter_delay(5000);
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
  detachInterrupt(0);
  Serial.println(F("Interrupt Hit, Good Morning!"));
  //  fona_toggle_pwr(true);
  startShutdown = false;
  error_level = 0;

}



void wakeUp() {
  // Just a handler for the pin interrupt.
}


void counter_delay(long delay_millis) {

  unsigned long started = millis();
  unsigned long last_print = 0;
  do {

    unsigned long time_till_continue = delay_millis - (millis() - started);
    if (millis() - last_print > 1000) {
      Serial.println(time_till_continue / 1000);
      last_print = millis();
    }
  }
  while (millis() - started < delay_millis);

}

void status_led(int error_status) {

  unsigned long currentMillis = millis();
  if (ledState == HIGH && ((currentMillis - onMillis) > 50)) {
    setColor(0, 0, 0);
    ledState = LOW;
    offMillis = currentMillis;
  }
  else if (ledState == LOW && ((currentMillis - offMillis) >= led_interval)) {
    onMillis = currentMillis;

    if (error_status == 0) {
      setColor(0, 100, 0);
    }
    else if (error_status == 1) {
      setColor(200, 200, 0);
    }
    else if (error_status == 2) {
      setColor(200, 0, 0);
    }
    else {
      setColor(80, 0, 80);
    }
    ledState = HIGH;
  }

}

void setColor(int red, int green, int blue) {
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);
}

