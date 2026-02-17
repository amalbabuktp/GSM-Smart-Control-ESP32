/* 
  Compact GSM-only motor controller (Version B)
  Pins (final):
    L298N IN1  -> GPIO26
    L298N IN2  -> GPIO27
    L298N ENA  -> GPIO25 (PWM)
    DS3231 SDA -> GPIO21
    DS3231 SCL -> GPIO22
    DHT11 DATA -> GPIO4
    ACS712 OUT -> GPIO34 (ADC1)
    SW-420 DO  -> GPIO33
    Buzzer Sig -> GPIO14
    SIM800L RX -> GPIO16 (module TX)
    SIM800L TX -> GPIO17 (module RX)
  SMS commands:
    ON, OFF, SPEED <0-100>, TIMER <minutes> OR SET <H>:<M>, CANCEL, STATUS, HI
*/

#include <Wire.h>
#include "RTClib.h"
#include "DHT.h"

// ---------- Pins ----------
#define IN1_PIN 26
#define IN2_PIN 27
#define PWM_PIN 25
#define PWM_CH 0

#define SDA_PIN 21
#define SCL_PIN 22

#define DHT_PIN 4
#define DHT_TYPE DHT11
DHT dht(DHT_PIN, DHT_TYPE);

#define ACS_PIN 34     // ADC1
#define VIB_PIN 33     // SW-420 digital out
#define BUZZER_PIN 14  // active buzzer signal

// SIM800L UART (HardwareSerial 1)
HardwareSerial SIM800(1);
const int SIM_RX_PIN = 16;
const int SIM_TX_PIN = 17;

// ---------- RTC ----------
RTC_DS3231 rtc;

// ---------- ACS parameters ----------
const float ACS_SENSITIVITY_V_PER_A = 0.185f; // 185 mV/A
const float OVERLOAD_CUTOFF_A = 4.0f;
const int ACS_CAL_SAMPLES = 100;
const int ACS_READ_SAMPLES = 50;
int acsZeroADC = 0;

// ---------- State ----------
bool motorRunning = false;
bool dirForward = true;
int motorSpeed = 75; // 0..100
unsigned long motorStartMillis = 0;
unsigned long timerDuration = 0; // ms

unsigned long buzzerEndMillis = 0; // for timed beeps
String lastSmsSender = "+918086745838";         // last SMS sender — used for alerts
bool vibrationAlertSent = false;   // one-shot per vibration start

// SIM timing
const unsigned long SIM_READ_TIMEOUT = 1200;
const unsigned long SIM_POLL_INTERVAL = 500;
unsigned long lastSimPoll = 0;

// ---------- Helpers ----------
String trimStr(String s) {
  s.trim();
  s.replace("\r", "");
  s.replace("\n", "");
  return s;
}

// ---------- Motor helpers ----------
void setDirectionPins() {
  if (dirForward) {
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
  } else {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
  }
}

void applySpeedToMotor() {
  int pwmVal = map(motorSpeed, 0, 100, 0, 255);
  if (motorRunning) ledcWrite(PWM_CH, pwmVal);
  else ledcWrite(PWM_CH, 0);
}

// ---------- ACS helpers ----------
int readAcsAdcAverage(int samples) {
  long tot = 0;
  for (int i = 0; i < samples; ++i) {
    tot += analogRead(ACS_PIN);
    delay(2);
  }
  return (int)(tot / samples);
}

float adcToVoltage(int adc) {
  return (adc * 3.3f) / 4095.0f;
}

float readCurrentA() {
  int avg = readAcsAdcAverage(ACS_READ_SAMPLES);
  float v = adcToVoltage(avg);
  float v0 = adcToVoltage(acsZeroADC);
  float diff = v - v0;
  float a = diff / ACS_SENSITIVITY_V_PER_A;
  if (a < 0) a = -a;
  return a;
}

void acsAutoCalibrate() {
  Serial.println("ACS calibration — keep motor OFF and no load.");
  delay(150);
  acsZeroADC = readAcsAdcAverage(ACS_CAL_SAMPLES);
  Serial.printf("ACS zero ADC=%d (%.4f V)\n", acsZeroADC, adcToVoltage(acsZeroADC));
}

// ---------- Buzzer & vibration ----------
void updateBuzzer() {
  bool vib = digitalRead(VIB_PIN) == HIGH;
  bool timed = (millis() < buzzerEndMillis);
  if (vib || timed) digitalWrite(BUZZER_PIN, HIGH);
  else digitalWrite(BUZZER_PIN, LOW);
}

void handleVibration() {
  bool vib = digitalRead(VIB_PIN) == HIGH;
  if (vib) {
    if (!vibrationAlertSent) {
      // first detection — send SMS alert
      vibrationAlertSent = true;
      String to = lastSmsSender.length() ? lastSmsSender : "";
      if (to.length()) {
        send_SMS: ; // forward reference removed by compiler; we call sendSms below
      }
    }
  } else {
    vibrationAlertSent = false;
  }
}

// We'll declare sendSms after this function because C++ requires functions declared first.
// To avoid forward reference issues, move sendSms declaration up. (We'll declare prototype:)
bool sendSms(const String &to, const String &msg); // prototype

void handleVibrationFixed() {
  bool vib = digitalRead(VIB_PIN) == HIGH;
  if (vib) {
    if (!vibrationAlertSent) {
      vibrationAlertSent = true;
      String to = lastSmsSender.length() ? lastSmsSender : "";
      if (to.length() > 0) {
        sendSms(to, "Vibration detected!");
      }
    }
  } else {
    vibrationAlertSent = false;
  }
}

// ---------- SIM helpers ----------
void simSendCmd(const char *cmd) {
  SIM800.println(cmd);
}

String simReadResponse(unsigned long timeout = SIM_READ_TIMEOUT) {
  unsigned long start = millis();
  String resp;
  while (millis() - start < timeout) {
    while (SIM800.available()) {
      resp += (char)SIM800.read();
    }
  }
  return resp;
}

bool sendSms(const String &to, const String &msg) {
  if (to.length() == 0) return false;
  SIM800.print("AT+CMGF=1\r");
  delay(150);
  simReadResponse(200);
  SIM800.print("AT+CMGS=\"");
  SIM800.print(to);
  SIM800.print("\"\r");
  delay(200);
  simReadResponse(200);
  SIM800.print(msg);
  SIM800.write(26); // Ctrl+Z
  unsigned long start = millis();
  String resp;
  while (millis() - start < 8000) {
    while (SIM800.available()) resp += (char)SIM800.read();
    if (resp.indexOf("OK") != -1 || resp.indexOf("+CMGS:") != -1) {
      Serial.println("SMS sent (resp): " + resp);
      return true;
    }
  }
  Serial.println("SMS send timeout: " + resp);
  return false;
}

// Poll unread SMS and extract first unread (+CMGL). Returns true if found and sets lastSmsSender/text
bool pollUnreadSMS(String &outSender, String &outText) {
  outSender = "";
  outText = "";

  SIM800.print("AT+CMGF=1\r");
  delay(150);
  simReadResponse(120);

  SIM800.print("AT+CMGL=\"REC UNREAD\"\r");
  delay(350);
  String resp = simReadResponse(700);
  if (resp.length() == 0) return false;
  int pos = resp.indexOf("+CMGL:");
  if (pos == -1) return false;

  int lineEnd = resp.indexOf("\n", pos);
  if (lineEnd == -1) return false;
  String meta = resp.substring(pos, lineEnd);
  meta.trim();

  // parse index
  int colon = meta.indexOf(':');
  int commaAfter = meta.indexOf(',', colon + 1);
  if (colon == -1 || commaAfter == -1) return false;
  String idxStr = meta.substring(colon + 1, commaAfter);
  idxStr.trim();
  int idx = idxStr.toInt();

  // get phone number (attempt robust extraction)
  String number = "";
  int q1 = meta.indexOf('"');
  if (q1 != -1) {
    int q2 = meta.indexOf('"', q1 + 1);
    int q3 = meta.indexOf('"', q2 + 1);
    int q4 = meta.indexOf('"', q3 + 1);
    if (q3 != -1 && q4 != -1) number = meta.substring(q3 + 1, q4);
    else if (q1 != -1 && q2 != -1) number = meta.substring(q1 + 1, q2);
  }
  number.trim();

  // message body
  int msgStart = lineEnd + 1;
  int msgEnd = resp.indexOf("\r\n", msgStart);
  if (msgEnd == -1) msgEnd = resp.indexOf("\n", msgStart);
  if (msgEnd == -1) msgEnd = resp.length();
  String text = resp.substring(msgStart, msgEnd);
  text.trim();

  if (text.length() == 0) {
    // delete empty, avoid loop
    char del[32];
    snprintf(del, sizeof(del), "AT+CMGD=%d\r", idx);
    SIM800.print(del);
    delay(200);
    simReadResponse(200);
    return false;
  }

  // delete SMS
  char del[32];
  snprintf(del, sizeof(del), "AT+CMGD=%d\r", idx);
  SIM800.print(del);
  delay(200);
  simReadResponse(300);

  outSender = number;
  outText = text;
  return true;
}

// ---------- Command parsing ----------
void processSMSCommand(const String &text, const String &sender) {
  String s = trimStr(text);
  String up = s;
  up.toUpperCase();

  // record last sender for alerts
  lastSmsSender = sender;

  // simple commands
  if (up == "HI") {
    sendSms(sender, "Cmds: ON, OFF, SPEED <0-100>, TIMER <min> OR SET H:M, CANCEL, STATUS");
    return;
  }
  if (up == "ON") {
    motorRunning = true;
    motorStartMillis = millis();
    setDirectionPins();
    applySpeedToMotor();
    sendSms(sender, "Motor STARTED.");
    return;
  }
  if (up == "OFF") {
    motorRunning = false;
    applySpeedToMotor();
    sendSms(sender, "Motor STOPPED.");
    return;
  }
  if (up.startsWith("SPEED")) {
    // allow "SPEED 80" or "SPEED:80"
    int sp = -1;
    int space = up.indexOf(' ');
    if (space != -1) sp = up.substring(space + 1).toInt();
    else {
      int colon = up.indexOf(':');
      if (colon != -1) sp = up.substring(colon + 1).toInt();
    }
    if (sp >= 0 && sp <= 100) {
      motorSpeed = sp;
      applySpeedToMotor();
      sendSms(sender, "Speed set to " + String(sp));
    } else {
      sendSms(sender, "Invalid speed. Use SPEED 0-100");
    }
    return;
  }
  if (up.startsWith("TIMER")) {
    // TIMER <minutes>
    int space = up.indexOf(' ');
    if (space == -1) { sendSms(sender, "TIMER <minutes>"); return; }
    int mins = up.substring(space + 1).toInt();
    if (mins <= 0) { sendSms(sender, "Invalid minutes"); return; }
    timerDuration = (unsigned long)mins * 60000UL;
    motorStartMillis = millis();
    motorRunning = true;
    applySpeedToMotor();
    sendSms(sender, "Timer set for " + String(mins) + " min. Motor started.");
    return;
  }
  if (up.startsWith("SET ")) {
    // support SET H:M (duration)
    String payload = s.substring(4);
    payload.trim();
    int colon = payload.indexOf(':');
    if (colon == -1) { sendSms(sender, "Use SET H:M (e.g. SET 0:30)"); return; }
    int hh = payload.substring(0, colon).toInt();
    int mm = payload.substring(colon + 1).toInt();
    if (hh < 0 || mm < 0) { sendSms(sender, "Invalid"); return; }
    unsigned long mins = (unsigned long)hh * 60UL + (unsigned long)mm;
    if (mins == 0) { sendSms(sender, "Duration zero"); return; }
    timerDuration = mins * 60000UL;
    motorStartMillis = millis();
    motorRunning = true;
    applySpeedToMotor();
    sendSms(sender, "Timer set for " + String(hh) + "h " + String(mm) + "m. Motor started.");
    return;
  }
  if (up == "CANCEL") {
    timerDuration = 0;
    sendSms(sender, "Timer canceled.");
    return;
  }
  if (up == "STATUS") {
    float cur = readCurrentA();
    float hum = dht.readHumidity();
    float temp = dht.readTemperature();
    DateTime now = rtc.now();
    String out = "";
    out += "Motor: "; out += motorRunning ? "ON\n" : "OFF\n";
    out += "Speed: " + String(motorSpeed) + "%\n";
    out += "Current: " + String(cur, 2) + " A\n";
    if (!isnan(temp)) out += "Temp: " + String(temp, 1) + " C\n";
    if (!isnan(hum)) out += "Hum: " + String(hum, 1) + " %\n";
    out += "Time: " + String(now.timestamp(DateTime::TIMESTAMP_TIME)) + "\n";
    if (motorRunning && timerDuration > 0) {
      unsigned long elapsed = millis() - motorStartMillis;
      unsigned long rem = (elapsed >= timerDuration) ? 0 : (timerDuration - elapsed);
      out += "Remaining: " + String((rem/60000)) + " min " + String(((rem%60000)/1000)) + " sec\n";
    }
    sendSms(sender, out);
    return;
  }

  // unknown
  sendSms(sender, "Unknown. Type HI for commands.");
}

// ---------- Overload and vibration protection ----------
void overloadCheck() {
  if (!motorRunning) return;
  float ifa = readCurrentA();
  if (ifa >= OVERLOAD_CUTOFF_A) {
    // immediate shutdown
    motorRunning = false;
    applySpeedToMotor();
    buzzerEndMillis = millis() + 3000UL;
    String to = lastSmsSender.length() ? lastSmsSender : "";
    if (to.length()) sendSms(to, "OVERLOAD! Motor stopped. Measured: " + String(ifa, 2) + " A");
    Serial.println("OVERLOAD! Motor stopped.");
  }
}

void vibrationCheck() {
  bool vib = digitalRead(VIB_PIN) == HIGH;
  if (vib) {
    // keep buzzer ON while vibration
    // stop motor (immediate)
    if (motorRunning) {
      motorRunning = false;
      applySpeedToMotor();
    }
    // send one SMS when vibration starts
    if (!vibrationAlertSent) {
      vibrationAlertSent = true;
      String to = lastSmsSender.length() ? lastSmsSender : "";
      if (to.length()) sendSms(to, "Vibration detected! Motor stopped.");
    }
  } else {
    vibrationAlertSent = false;
  }
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  delay(50);

  // motor pins
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  ledcSetup(PWM_CH, 5000, 8);
  ledcAttachPin(PWM_PIN, PWM_CH);
  ledcWrite(PWM_CH, 0);

  // sensors/buzzer
  pinMode(VIB_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  Wire.begin(SDA_PIN, SCL_PIN);
  dht.begin();

  if (!rtc.begin()) Serial.println("RTC not found");
  DateTime now = rtc.now();
  if (now.year() < 2000) rtc.adjust(DateTime(__DATE__, __TIME__));

#ifdef ARDUINO_ARCH_ESP32
  analogSetPinAttenuation(ACS_PIN, ADC_11db);
#endif
  acsAutoCalibrate();

  // SIM800 init on UART1 (9600)
  SIM800.begin(9600, SERIAL_8N1, SIM_RX_PIN, SIM_TX_PIN);
  delay(300);
  simSendCmd("AT");
  delay(100);
  simReadResponse(200);
  simSendCmd("ATE0");
  delay(150);
  simReadResponse(200);
  simSendCmd("AT+CMGF=1");
  delay(150);
  simReadResponse(200);
  simSendCmd("AT+CNMI=2,1,0,0,0"); // request push if supported
  delay(150);
  simReadResponse(200);

  Serial.println("System ready. Commands via SMS: ON,OFF,SPEED,TIMER/SET,CANCEL,STATUS,HI");
}

// ---------- Loop ----------
void loop() {
  // update buzzer state (vibration or timed beeps)
  updateBuzzer();

  // check vibration and send SMS once when starting
  vibrationCheck();

  // overload protection
  overloadCheck();

  // timer expiry
  if (motorRunning && timerDuration > 0) {
    unsigned long elapsed = millis() - motorStartMillis;
    if (elapsed >= timerDuration) {
      motorRunning = false;
      applySpeedToMotor();
      timerDuration = 0;
      buzzerEndMillis = millis() + 3000UL;
      String to = lastSmsSender.length() ? lastSmsSender : "";
      if (to.length()) sendSms(to, "TIMER FINISHED - Motor OFF");
      Serial.println("Timer finished - motor off");
    }
  }

  // poll SMS periodically
  if (millis() - lastSimPoll >= SIM_POLL_INTERVAL) {
    lastSimPoll = millis();
    String sender, text;
    if (pollUnreadSMS(sender, text)) {
      // we got an SMS
      Serial.printf("SMS from %s : %s\n", sender.c_str(), text.c_str());
      processSMSCommand(text, sender);
    }
  }

  // small non-blocking wait
  delay(20);
}