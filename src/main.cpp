#include <Arduino.h>
#include "driver/twai.h"
#include "xiaomi_cybergear_driver.h"

// Pins used to connect to CAN bus transceiver:
#define RX_PIN 4
#define TX_PIN 5

// Intervall:
#define TRANSMIT_RATE_MS 1000
#define POLLING_RATE_MS 1000

static bool driver_installed = false;
static bool rotate_clockwise = false;
//static String inputString = "";
unsigned long previousMillis = 0;  // will store last time a message was send

uint8_t CYBERGEAR_CAN_ID1 = 0x65;
uint8_t CYBERGEAR_CAN_ID2 = 0x66;
uint8_t CYBERGEAR_CAN_ID3 = 0x67;
uint8_t MASTER_CAN_ID = 0x00;
XiaomiCyberGearDriver cybergear1 = XiaomiCyberGearDriver(CYBERGEAR_CAN_ID1, MASTER_CAN_ID);
XiaomiCyberGearDriver cybergear2 = XiaomiCyberGearDriver(CYBERGEAR_CAN_ID2, MASTER_CAN_ID);
XiaomiCyberGearDriver cybergear3 = XiaomiCyberGearDriver(CYBERGEAR_CAN_ID3, MASTER_CAN_ID);

float input_pos = 0.0f;

// function declarations
static void initialize_all_motors();
static void all_motor_pos_to_zero();
static void handle_rx_message(twai_message_t& message);
static void check_alerts();

void setup() {
  initialize_all_motors(); // All initializing for all motors!
  all_motor_pos_to_zero(); // Power all motors and set positions to zero

  cybergear1.set_position_ref(0.0f);
  cybergear2.set_position_ref(0.0f);
  cybergear3.set_position_ref(0.0f);
}

void loop() {
  if (!driver_installed) {
    delay(1000);
    return;
  }

  delay(30);
  check_alerts();

  XiaomiCyberGearStatus cybergear_status = cybergear1.get_status();
  Serial.printf("POS:%f V:%f T:%f temp:%d\n", cybergear_status.position, cybergear_status.speed, cybergear_status.torque, cybergear_status.temperature);

  // if (cybergear_status.torque > 0.9 || cybergear_status.torque < -0.9) {
  //   cybergear1.stop_motor();
  //   Serial.println("Torque limit reached - stopping motor");
  //   return; // Exit the loop
  // }

  char command = Serial.read();   // Read the incoming byte

  if (command == '=') {
//    input_pos += 0.1f;
      cybergear1.reset_twai(RX_PIN, TX_PIN, /*serial_debug=*/true); // once for any cybergear
  }

//  if (command == '-') {
//    input_pos += -0.1f;
//  }

  if (command == '-') {
    cybergear1.init_twai(RX_PIN, TX_PIN, /*serial_debug=*/true); // once for any cybergear
  }

  cybergear3.set_position_ref(input_pos);

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= TRANSMIT_RATE_MS) {
    previousMillis = currentMillis;
    cybergear1.request_status();
  }
}

static void handle_rx_message(twai_message_t& message) {
  if (((message.identifier & 0xFF00) >> 8) == CYBERGEAR_CAN_ID1){
    cybergear1.process_message(message);
  }
}

static void all_motor_pos_to_zero()
{

//  cybergear1.motor_pos_to_zero();
  cybergear1.set_position_ref(0.0f);
  cybergear2.set_position_ref(0.0f);
  cybergear3.set_position_ref(0.0f);
}

static void initialize_all_motors()
{
  cybergear1.init_twai(RX_PIN, TX_PIN, /*serial_debug=*/true); // once for any cybergear

  cybergear1.init_motor(MODE_POSITION); 
  cybergear1.set_limit_speed(1.0f); /* set the maximum speed of the motor */ // was set to 10.0f!
  cybergear1.set_limit_current(4.0); /* current limit allows faster operation */
  cybergear1.set_limit_torque(1.5f); // lowered from 1.5
  cybergear1.enable_motor(); /* turn on the motor */

  cybergear2.init_motor(MODE_POSITION); 
  cybergear2.set_limit_speed(1.0f); /* set the maximum speed of the motor */ // was set to 10.0f!
  cybergear2.set_limit_current(4.0); /* current limit allows faster operation */
  cybergear2.set_limit_torque(1.5f); // lowered from 1.5
  cybergear2.enable_motor(); /* turn on the motor */

  cybergear3.init_motor(MODE_POSITION); 
  cybergear3.set_limit_speed(1.0f); /* set the maximum speed of the motor */ // was set to 10.0f!
  cybergear3.set_limit_current(4.0); /* current limit allows faster operation */
  cybergear3.set_limit_torque(1.5f); // lowered from 1.5
  cybergear3.enable_motor(); /* turn on the motor */

  driver_installed = true;
}

static void check_alerts(){
  // Check if alert happened
  uint32_t alerts_triggered;
  twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(POLLING_RATE_MS));
  twai_status_info_t twai_status;
  twai_get_status_info(&twai_status);

  // Handle alerts
  if (alerts_triggered & TWAI_ALERT_ERR_PASS) {
    Serial.println("Alert: TWAI controller has become error passive.");
  }
  if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
    Serial.println("Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.");
    Serial.printf("Bus error count: %d\n", twai_status.bus_error_count);
  }
  if (alerts_triggered & TWAI_ALERT_TX_FAILED) {
    Serial.println("Alert: The Transmission failed.");
    Serial.printf("TX buffered: %d\t", twai_status.msgs_to_tx);
    Serial.printf("TX error: %d\t", twai_status.tx_error_counter);
    Serial.printf("TX failed: %d\n", twai_status.tx_failed_count);
  }

  // Check if message is received
  if (alerts_triggered & TWAI_ALERT_RX_DATA) {
    twai_message_t message;
    while (twai_receive(&message, 0) == ESP_OK) {
      handle_rx_message(message);
    }
  }
}