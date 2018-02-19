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
bool have_valid_location = false;
bool have_battery_pct = false;
bool message_sent = false;



char sendto[21] = "";
char message[80];

uint16_t vbat;

SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

void setup() {
  Serial.begin(9600);

  // Status LED
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  // Pin reads the PSTAT pin on the FONA, and will be HIGH when the FONA is on
  pinMode(FONA_PWR_STAT, INPUT);

  // Pin controls the KEY pin on the FONA, driving HIGH for 2 seconds, then LOW for 2 seconds will toggle the power
  pinMode(FONA_PWR, OUTPUT);

  Serial.println(F("BOOT STATUS: Delaying 5 Seconds to Start"));
  counter_delay(5000);

  // Determine whether the FONA is ON, if it is, we need to turn it OFF
  if (digitalRead(FONA_PWR_STAT)) {
    Serial.println(F("VOID SETUP: FONA is ON, Turning FONA OFF.\n"));
    fona_toggle_pwr(false);
  }
  else {
    Serial.println(F("VOID SETUP: FONA is already OFF.\n"));
  }

  // Determine whether the 5v jack is supplying power. If it is, we should go into void_loop as normal
  // If not, we should go to sleep and wait for the next plugin.
  last_voltage = (float)(analogRead(voltage_pin) / 1024.0) * 3.3;

  if (last_voltage >= .5) {
    Serial.println(F("VOID SETUP: 5v jack is supplying power. Continue as normal.\n"));
  }
  else {
    Serial.println(F("VOID SETUP: 5v jack is not supply power. Going into low power mode.\n"));
    go_to_sleep();
  }


}

void loop() {

  if (digitalRead(FONA_PWR_STAT) && startShutdown == false) {
    Serial.println(F("LOOP STATUS: FONA is ON, Turning FONA OFF."));
    fona_toggle_pwr(false);
  }

  status_led(error_level);

  // get_voltage() sets the global variable startShutdown
  get_voltage();

  if (startShutdown) {
    // Turn the FONA On
    fona_toggle_pwr(true);
    // get_gps_location() sets global variable have_valid_location
    get_battery(); // Will return a battery % even if 30x attempts fail.
    get_gps_location();
    if (have_valid_location) {
      send_sms();
      fona_toggle_pwr(false);
      go_to_sleep();
    }
    else {
      Serial.println(F("VOID LOOP: Unable to Fix GPS Location, Going to Sleep\n"));
      fona_toggle_pwr(false);
      go_to_sleep();
    }
  }

}




void get_battery() {
  // Try to get the battery percentage
  Serial.println(F("Attempting to get battery life from FONA"));
  int tries = 0;
  have_battery_pct = false;
  do {
    have_battery_pct = fona.getBattPercent(&vbat);
    if (have_battery_pct) {
      return have_battery_pct;
    }
    ++tries;
    delay(1000);
  }
  while (tries <= 30);
  if (!have_battery_pct) {
    vbat = 00;
    have_battery_pct = true;
  }
  Serial.print(F("The battery has a value of "));
  Serial.println(vbat);
  Serial.print(" out of 100% \n");
}

void get_gps_location() {
  Serial.println(F("Attempting to grab the most up to date GPS Fix on the location./n"));
  int tries = 0;
  do {
    float latitude, longitude, speed_kph, heading, speed_mph, altitude;
    have_valid_location = fona.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude);
    char LAT[10];
    char LONG[10];

    if (have_valid_location) {
      dtostrf(latitude, 8, 6, LAT);
      dtostrf(longitude, 8, 6, LONG);
      sprintf(message, "Location:\nmaps.google.com/?q=""%s,%s""\nBatt: ""%u""%%", LAT, LONG, vbat);
      Serial.println(F("Sucessfully put together the URL string, including the GPS fix./n"));
      Serial.println(message);
      return have_valid_location;
    } else {
      Serial.println(F("Waiting for FONA GPS 3D fix..."));
    }
    delay(1000);
  } while ( tries < 500 );
}

void send_sms() {


  // Send SMS
  Serial.println(F("Attempting to send the SMS Message: \n"));
  Serial.print(message);
  Serial.print(F("\n"));
  bool message_sent = false;
  int tries = 0;
  do {
    if (!fona.sendSMS(sendto, message)) {
      Serial.println(F("SMS Failed"));
      message_sent = false;
      ++tries;
    } else {
      Serial.println(F("SMS Sent!"));
      message_sent = true;
      break;
    }
    delay(100);
  }
  while (tries <= 4);

}

void fona_toggle_pwr(bool booting) {

  digitalWrite(FONA_PWR, HIGH);
  delay(2000);
  digitalWrite(FONA_PWR, LOW);
  delay(2000);
  digitalWrite(FONA_PWR, HIGH);

  if (booting) {
    Serial.println(F("Booting - starting Serial FONA comms."));
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

void get_voltage() {

  float current_voltage = analogRead(voltage_pin);
  current_voltage = (float)(current_voltage / 1024.0) * 3.3;
  if ( current_voltage < .5 && last_voltage >= .5 ) {
    startShutdown = true;
  }
  else {
    startShutdown = false;
  }
  last_voltage = current_voltage;

}

void go_to_sleep() {

  setColor(0, 0, 0);
  attachInterrupt(0, wakeUp, RISING);
  counter_delay(5000);
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
  detachInterrupt(0);
  Serial.println(F("Ignition Started - In standby until next shutdown of automobile."));

  startShutdown = false;
  error_level = 0;
  // Car power when turning on goes high-low-high. Delay seeks toavoid  putting FONA to sleep because of this.
  delay(5000);

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
