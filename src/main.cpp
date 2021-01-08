#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ESP8266Init.h>
#include <PropertyNode.h>
#include <PubSubClientInterface.h>
#include <Ticker.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>

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
ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

ESP8266Init esp8266Init{"DCHost", "dchost000000", "192.168.43.1", 1883,
                        "stir motor controller"};

PubSubClientInterface mqttInterface(esp8266Init.client);
PropertyNode<float> level("level", 0.0, false, true);
PropertyNode<float> rpm("rpm", 0.0, true, true);
String buf;

bool is_stuck = true;
float boost_to = 0;

class Tachometer {
private:
  uint32_t counter;
  int pin;
  int debounce_ms;
  int loop_interval_ms; // 0 for no wait

  uint32_t last_query = 0;

  uint8_t current_state = 0;

  uint32_t last_state_change_begin = 0;

  uint32_t last_counter_retrieval = millis();

public:
  Tachometer(int pin, int debounce_ms = 5, int loopIntervalMs = 0)
      : pin(pin), debounce_ms(debounce_ms), loop_interval_ms(loopIntervalMs) {}

public:
  float average_rpm() {
    auto current = millis();
    auto sec = (current - last_counter_retrieval) / 1000.;
    auto rpm_value = counter / sec * 60.;
    counter = 0;
    last_counter_retrieval = current;
    return rpm_value;
  }
  void feed(uint8 pin_state) {
    if (current_state != pin_state) {
      if (last_state_change_begin != 0) {
        // there is a previous state change stored.
        auto delta = millis() - last_state_change_begin;
        if (delta <= debounce_ms) {
          // this state change is not valid (noise)
        } else {
          current_state = !current_state;
          if (current_state == 0) counter++; // There will be two state changes. Only count one.
        }
        last_state_change_begin = 0;
      }
      last_state_change_begin = millis();
    }
  }

  void loop() {
    if ((loop_interval_ms != 0) && (millis() - last_query < loop_interval_ms)) {
      // Should wait and should wait longer
      return;
    }

    feed(digitalRead(pin));
  }
} tachometer{PIN_TACH};

void tachometer_report() {
  float rpm_value = tachometer.average_rpm();
  //  Serial.println(rpm_value);
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

void configure_ota() {
  std::stringstream ss;
  ss << ESP.getChipId();

  MDNS.begin((("update_" + ss.str()).c_str()));
  httpUpdater.setup(&httpServer);
  httpServer.begin();
  MDNS.addService("http", "tcp", 80);

//  ArduinoOTA.setPort(8266);
//  ArduinoOTA.setRebootOnSuccess(true);
//  ArduinoOTA.begin();
//  httpUpdater.setup(&httpServer);
//  httpServer.begin();
}
void setup() {
  analogWriteFreq(PWM_FREQ);
  analogWriteRange(PWM_RANGE);
  pinMode(PIN_1, OUTPUT);
  pinMode(PIN_2, OUTPUT);
  pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_TACH, INPUT);
  Serial.begin(115200);
  // write your initialization code here
  if (esp8266Init.blocking_init() != ESP8266Init::FINISHED) {
    delay(1000);
    ESP.restart();
  }
  configure_ota();

  level.register_interface(mqttInterface);
  level.set_validator(
      [](double value) { return (value >= -100) && (value <= 100); });
  level.set_update_callback(
      [](double oldVal, double newVal) { set_pwm_level(newVal); });

  rpm.register_interface(mqttInterface);

  tachometer_report_ticker.start();

}

void loop() {
  // write your code here
  esp8266Init.client.loop();
  tachometer_report_ticker.update();
//  ArduinoOTA.handle();
  httpServer.handleClient();
  MDNS.update();
  boost_up_ticker.update();
  tachometer.loop();
}