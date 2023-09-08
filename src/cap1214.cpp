/*
 * Author: Matthias Akstaller
 */

#if defined(ENABLE_CAP1214)

#if defined(ENABLE_DEBUG) && !defined(ENABLE_DEBUG_NFCREADER)
#undef ENABLE_DEBUG
#endif

#include "cap1214.h"
#include <Wire.h>
#include "app_config.h"
#include "debug.h"
#include "lcd.h"

#ifndef I2C_SDA
#define I2C_SDA 21
#endif

#ifndef I2C_SCL
#define I2C_SCL 22
#endif

#define  SCAN_DELAY            100
#define  INIT_DELAY             20

#define SLEEP_COUNT (10000 / SCAN_DELAY)

#define MAX_PIN_LENGTH 8

#define MAXIMUM_UNRESPONSIVE_TIME  60000UL //after this period the pn532 is considered offline
#define AUTO_REFRESH_CONNECTION         30 //after this number of polls, the connection to the PN532 will be refreshed


/*
 * Documentation of the CAP1214:
 * https://ww1.microchip.com/downloads/en/DeviceDoc/1214.pdf
 * (accessed: 2023-09-08)
 */
#include "cap1214-reg.h"

CAP1214::CAP1214() : MicroTasks::Task() {
    
}

void CAP1214::begin() {
    Wire.begin(I2C_SDA, I2C_SCL);
    MicroTask.startTask(this);
}

unsigned long CAP1214::loop(MicroTasks::WakeReason reason){
  switch (status) {

  case DeviceStatus::BOOTUP:
    // Pause before we initialize.
    status = DeviceStatus::NOT_ACTIVE;
    return INIT_DELAY;

  case DeviceStatus::NOT_ACTIVE:
    if (config_rfid_enabled()) {
      // This will set the device status as a side effect
      initialize();
      DBUGLN(F("[rfid] CAP1214 initialized"));
    }
    return SCAN_DELAY;

  default:
  case DeviceStatus::FAILED:
    // do nothing, no keypad detected
    return SCAN_DELAY;

  case DeviceStatus::ACTIVE:
    if (!config_rfid_enabled()) {
      status = DeviceStatus::NOT_ACTIVE;
      write(CAP1214_REG_MAIN_STATUS, 0x40); // disable scanning
      return SCAN_DELAY;
    }
    static char last_key = 'Q';
    char key;
    bool prox;
    bool detected = false;
    read_key(&key, &prox);
    if (key != last_key) {
      last_key = key;
      if (key != '\0') {
        if (key == '\b') {
          if (pin.length() > 0) {
            pin.remove(pin.length()-1);
          }
        } else if (key == '\r') {
          detected = (pin.length() > 0);
        } else {
          if (pin.length() >= MAX_PIN_LENGTH) {
            // limit length of pin
            pin.remove(pin.length()-1);
          }
          pin += key;
        }
        String msg = "  PIN: " + pin;
        lcd.display(msg, 0, 1, 20*1000, LCD_CLEAR_LINE | LCD_DISPLAY_NOW);
      }
    }
    if (prox || key != '\0') {
      sleepCount = SLEEP_COUNT;
    }
    if (sleepCount > 0) {
      // should be awake
      sleepCount--;
      if (!is_awake) {
        is_awake = true;
        // wake up: flip polarity to normally-on
        write(CAP1214_REG_LED_POLARITY_1, 0xFF);
        write(CAP1214_REG_LED_POLARITY_2, 0xFF);
      }
    } else {
      // should be asleep
      if (is_awake) {
        is_awake = false;
        // go to sleep: flip polarity to normally-off
        write(CAP1214_REG_LED_POLARITY_1, 0x00);
        write(CAP1214_REG_LED_POLARITY_2, 0x00);
      }
    }
    if (detected) {
      onCardDetected(pin);
    }
    return SCAN_DELAY;
  }
}

bool CAP1214::readerFailure() {
    return config_rfid_enabled() && status == DeviceStatus::FAILED;
}

bool CAP1214::write(uint8_t regno, uint8_t val) {
  Wire.beginTransmission(CAP1214_I2C_ADDR);
  Wire.write(regno);
  Wire.write(val);
  uint8_t status = Wire.endTransmission();
  if (status != 0) {
    return false;
  }
  return true;
}

bool CAP1214::read(uint8_t regno, uint8_t *val) {
    Wire.beginTransmission(CAP1214_I2C_ADDR);
    Wire.write(regno);
    uint8_t status = Wire.endTransmission();
    if (status != 0) {
        return false;
    }
    uint8_t nBytes = Wire.requestFrom(CAP1214_I2C_ADDR, 1);
    if (nBytes == 0) {
        return false;
    }
    *val = Wire.read();
    return true;
}

void CAP1214::initialize(void) {
  uint8_t vendor_id, product_id, revision, build_rev;
  // this should be called no sooner than 20ms from reset
  if (!read(CAP1214_REG_VENDOR_ID, &vendor_id)) { goto failed; }
  if (vendor_id != 0x5D) { goto failed; }
  if (!read(CAP1214_REG_PRODUCT_ID, &product_id)) { goto failed; }
  if (product_id != 0x5A) { goto failed; }
  if (!read(CAP1214_REG_REVISION, &revision)) { goto failed; }
  if (revision != 0x80) { goto failed; }
  if (!read(CAP1214_REG_BUILD_REVISION, &build_rev)) { goto failed; }
  if (build_rev != 0x10) { goto failed; }
  // Activate LEDs and scanning
  write(CAP1214_REG_MAIN_STATUS, 0x00);
  write(CAP1214_REG_CONFIGURATION, 0x2A);// enable recal on grouped sens, disable repeat rate
  write(CAP1214_REG_CONFIGURATION_2, 0x02); // *not* VOL_UP_DOWN
  write(CAP1214_REG_GROUP_CONFIGURATION_1, 0x40); // zero out M_PRESS (CS8)
  write(CAP1214_REG_GROUPED_CHANNEL_SENSOR_ENABLE, 0x3F); // disable CS14
  // Increase general sensitivity
  write(CAP1214_REG_DATA_SENSITIVITY, 0x1F); // 64x sensitivity
  write(CAP1214_REG_PROXIMITY_CONTROL, 0x80); // CS1 is prox, max sens
  // allow independent configuration of CS1 threshold
  write(CAP1214_REG_RECALIBRATION_CONFIGURATION, 0x13); // !BUT_LD_TH
  // 256ms, 2kHz 110 01000
  write(CAP1214_REG_FEEDBACK_CONFIGURATION, 0xC8); // 256ms, 2kHz
  // Everything triggers feedback, but CS1, CS8, and CS14
  // (CS8 has a weird press-and-hold misfeatures that triggers feedback
  // before the actual press is registered in the queue.)
  write(CAP1214_REG_FEEDBACK_CHANNEL_CONFIGURATION_1, 0x7E);
  write(CAP1214_REG_FEEDBACK_CHANNEL_CONFIGURATION_2, 0x3E);
  // Link the sensor LEDs CS2-CS3,CS5-CS7
  // - excluding CS1 because it is used for proximity
  // - excluding CS4 for which we need manual control because it is paired
  // - excluding UP_DOWN_LINK
  write(CAP1214_REG_SENSOR_LED_LINKING, 0x76);
  // LED/GPIO pins are outputs!
  write(CAP1214_REG_LED_GPIO_DIRECTION, 0xFF); // LED1-LED8 = output
  // flip polarity of all LEDs (normally on!)
  write(CAP1214_REG_LED_POLARITY_1, 0xFF);
  write(CAP1214_REG_LED_POLARITY_2, 0xFF);
  // disable multiple touch suppression (since PROX aka CS1 sets it off)
  write(CAP1214_REG_MULTIPLE_PRESS_CONFIGURATION, 0x00);
  // Improve responsiveness: by default we need 3 readings over threshold;
  // change to 1 to make things a bit snappier.
  write(CAP1214_REG_QUEUE_CONTROL, 0x01);

  status = DeviceStatus::ACTIVE;
  return;

failed:
  status = DeviceStatus::FAILED;
  return;
}

uint16_t CAP1214::read_buttons(void) {
  uint8_t st1 = 0, st2 = 0, st3 = 0;
  read(CAP1214_REG_BUTTON_STATUS_1, &st1);
  read(CAP1214_REG_BUTTON_STATUS_2, &st2);
  read(CAP1214_REG_GROUP_STATUS, &st3); // clears TAP
  // clear the latched button status by clearing INT!
  write(CAP1214_REG_MAIN_STATUS, 0x00);

  // merge
  uint16_t merged = (st1 & 0x3F); // zero out UP & DOWN
  merged |= (((uint16_t)st2) << 6); // Add in high order bits at the right place
  // account for tap and hold logic on CS8 (CS14 is unused)
  if (st3 & 0xF) { merged |= 0x80; }

  // manually set feedback pins for CS8-CS13
  // CS8/9/10 -> LED8/9/10
  uint8_t led1 = 0, led2 = 0;
  static bool saw_cs8 = false;
  if (merged & (((uint16_t)1)<<7)) { // CS8 (touch 11, key 0)
    led1 |= 0x80;
    // Need to manually trigger feedback, because if we link this automatically
    // feedback will be triggered for touches too short to actually register
    // in the button queue.
    if (!saw_cs8) {
      write(CAP1214_REG_FEEDBACK_ONESHOT, 0xFF);
      saw_cs8 = true;
    }
  } else {
    saw_cs8 = false;
  }
  if (merged & (((uint16_t)1)<<8)) { // CS9 (touch 2)
    led2 |= 0x01;
  }
  if (merged & (((uint16_t)1)<<9)) { // CS10 (touch 3)
    led2 |= 0x02;
  }
  // CS11 -> LED4 (as well as CS4)
  if (merged & (((uint16_t)1)<<3)) { // CS4 (touch 10, key backspace)
    led1 |= 0x08;
  }
  if (merged & (((uint16_t)1)<<10)) { // CS11 (touch 12, key enter)
    led1 |= 0x08;
  }
  // CS12 -> LED1
  if (merged & (((uint16_t)1)<<11)) { // CS12 (touch 9)
    led1 |= 0x01;
  }
  // CS13 -> LED11
  if (merged & (((uint16_t)1)<<12)) { // CS13 (touch 6)
    led2 |= 0x04;
  }
  // Update LED OUTPUT CONTROL if necessary
  // last_* initialized to bogus values to force an initial update
  static uint8_t last_led1 = 0xFF, last_led2 = 0xFF;
  if (led1 != last_led1) {
    write(CAP1214_REG_LED_OUTPUT_CONTROL_1, led1);
    last_led1 = led1;
  }
  if (led2 != last_led2) {
    write(CAP1214_REG_LED_OUTPUT_CONTROL_2, led2);
    last_led2 = led2;
  }
  return merged;
}

void CAP1214::read_key(char *key, bool *prox) {
  uint16_t st = read_buttons();
  *prox = (st & 1);
  *key = 0;
  uint16_t mask;
  const char *cp = "47\b158023\r96";
  for (mask=2; *cp; cp++, mask<<=1) {
    if (st&mask) {
      *key = *cp;
      return;
    }
  }
}

CAP1214 cap1214;

#endif
