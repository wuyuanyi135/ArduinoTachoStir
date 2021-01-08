#include <Arduino.h>
#include <ESP8266Init.h>
#include <PropertyNode.h>
#include <PubSubClientInterface.h>
#include <SimpleCLIInterface.h>
#include <Ticker.h>
#include <WiFiClient.h>

#define PIN_1 D0
#define PIN_2 D2
#define PIN_EN D1
#define PIN_TACH D5
#define PWM_FREQ 100
#define PWM_RANGE 1023
#define TACH_DEBOUNCE_MS 40
#define TACH_REPORT_MS 5000
#define MINIMUM_START_UP 23
#define BOOST_UP_DELAY 1000
ESP8266Init esp8266Init{"DCHost", "dchost000000", "192.168.43.1", 1883,
                        "stir motor controller"};

SimpleCLI cli;
PubSubClientInterface mqttInterface(esp8266Init.client);
SimpleCLIInterface simpleCliInterface(cli, Serial);
PropertyNode<float> level("level", 0.0, false, true);
PropertyNode<float> rpm("rpm", 0.0, true, true);
String buf;
volatile uint32_t tach_counter = 0;
volatile uint32_t tach_last_enter = 0;
bool is_stuck = true;
float boost_to = 0;

void tachometer_report() {
  float rpm_value = tach_counter * 60 / ((float)TACH_REPORT_MS / 1000);
  //  Serial.println(rpm_value);
  tach_counter = 0;
  rpm.set_value(rpm_value);
  rpm.notify_get_request_completed();
}

Ticker tachometer_report_ticker(tachometer_report, TACH_REPORT_MS, 0, MILLIS);

void set_pwm_level(float val);
void boost_up() { set_pwm_level(boost_to); }
Ticker boost_up_ticker(boost_up, BOOST_UP_DELAY, 1, MILLIS);

void set_pwm_level(float val) {
  if (val == 0) {
    digitalWrite(PIN_1, LOW);
    digitalWrite(PIN_2, LOW);
    digitalWrite(PIN_EN, LOW);
    is_stuck = true;
    return;
  }

  if (is_stuck && std::abs(val) < MINIMUM_START_UP) {
    boost_to = val;
    val = MINIMUM_START_UP * (val > 0 ? 1 : -1);
    is_stuck = false;
    boost_up_ticker.start();
  }

  if (val > 0) {
    int pwm_level = (int)(val / 100. * PWM_RANGE);
    analogWrite(PIN_EN, pwm_level);
    digitalWrite(PIN_1, HIGH);
    digitalWrite(PIN_2, LOW);
    return;
  }

  if (val < 0) {
    int pwm_level = (int)(-val / 100. * PWM_RANGE);

    analogWrite(PIN_EN, pwm_level);
    digitalWrite(PIN_1, LOW);
    digitalWrite(PIN_2, HIGH);
    return;
  }
}

ICACHE_RAM_ATTR void tach_isr() {
  auto current_time = millis();
  if (current_time - tach_last_enter <= TACH_DEBOUNCE_MS) {
    return;
  }

  tach_last_enter = current_time;
  tach_counter++;
}

void setup() {
  analogWriteFreq(PWM_FREQ);
  analogWriteRange(PWM_RANGE);
  pinMode(PIN_1, OUTPUT);
  pinMode(PIN_2, OUTPUT);
  pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_TACH, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_TACH), tach_isr, FALLING);
  Serial.begin(115200);
  // write your initialization code here
  if (esp8266Init.blocking_init() != ESP8266Init::FINISHED) {
    delay(1000);
    ESP.restart();
  }

  level.register_interface(mqttInterface);
  level.register_interface(simpleCliInterface);
  level.set_validator(
      [](double value) { return (value >= -100) && (value <= 100); });
  level.set_update_callback(
      [](double oldVal, double newVal) { set_pwm_level(newVal); });

  rpm.register_interface(mqttInterface);
  rpm.register_interface(simpleCliInterface);

  tachometer_report_ticker.start();
}

void loop() {
  // write your code here
  esp8266Init.client.loop();
  tachometer_report_ticker.update();
  boost_up_ticker.update();
  while (Serial.available()) {
    // Read out string from the serial monitor
    char ch = (char)Serial.read();
    buf += ch;
    // Parse the user input into the CLI
    if (ch == '\n') {
      cli.parse(buf);
      buf.clear();
    }
  }
}