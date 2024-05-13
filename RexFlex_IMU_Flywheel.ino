// CAN ===========================================================================================================
#include <Arduino.h>
#include "ODriveCAN.h"
#include <Arduino_CAN.h>
#include <ODriveHardwareCAN.hpp>
// Documentation for this example can be found here:
// https://docs.odriverobotics.com/v/latest/guides/arduino-can-guide.html
// See https://github.com/arduino/ArduinoCore-API/blob/master/api/HardwareCAN.h
// and https://github.com/arduino/ArduinoCore-renesas/tree/main/libraries/Arduino_CAN

// IMU ===========================================================================================================
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55);
// PD controller initializing
float phieq = 166.19;
float betaeq = 0.81;
float phi_current = 0;
int counter = 0;

// float signal = 0.0;
float sign = 0;

float diff_x = 0;
float diff_y = 0;

float error = 0;
float d_term = 0;
float p_term = 0;
float full_k = 0;
float xval=0;
float yval=0;
const float dt = 100; //100 milliseconds

// Gains
const float Kp = 0.3;
const float Kd = 1;

float vel = 0;
float max_vel = 0;

// CAN ===========================================================================================================
// CAN bus baudrate. Make sure this matches for every device on the bus
#define CAN_BAUDRATE 250000
// ODrive node_id for odrv0
#define ODRV0_NODE_ID 0
#define IS_ARDUINO_BUILTIN // Arduino boards with built-in CAN interface (e.g. Arduino Uno R4 Minima)
HardwareCAN& can_intf = CAN;
bool setupCan() {
  return can_intf.begin((CanBitRate)CAN_BAUDRATE);
}
//IMU ===========================================================================================================
float PID()
{
// positive torque brings to equilibrium 
//   -x              ^ 
// -----------------(RW)     
//   +x
// error = phi_current-phieq;
error = diff_x;
d_term = Kd* (error / dt);
p_term = Kp* (error);
full_k = d_term + p_term;
return full_k;
}
void ValPoint()
{
  // Getting equilibrium value point
  sensors_event_t event; 
  bno.getEvent(&event);

  Serial.println("Getting Setpoints...");
  delay(10000);
  betaeq=event.orientation.y;
  phieq=event.orientation.x; 
  Serial.println("Setpoints Achieved");
  delay (5000);

}
void IMU_Data()
{
  sensors_event_t event; 
  bno.getEvent(&event);

  xval= event.orientation.x;
  // Serial.print("Xval: ");
  // Serial.print(xval);

  yval= event.orientation.y;
  // Serial.print("Yval: ");
  // Serial.print(yval);
  // Serial.println(""); 
}

//CAN ===========================================================================================================
// Instantiate ODrive objects
ODriveCAN odrv0(wrap_can_intf(can_intf), ODRV0_NODE_ID); // Standard CAN message ID
ODriveCAN* odrives[] = {&odrv0}; // Make sure all ODriveCAN instances are accounted for here

struct ODriveUserData {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
};

// Keep some application-specific user data for every ODrive.
ODriveUserData odrv0_user_data;

// Called every time a Heartbeat message arrives from the ODrive
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_heartbeat = msg;
  odrv_user_data->received_heartbeat = true;
}

// Called every time a feedback message arrives from the ODrive
void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_feedback = msg;
  odrv_user_data->received_feedback = true;
}

// Called for every message that arrives on the CAN bus
void onCanMessage(const CanMsg& msg) {
  for (auto odrive: odrives) {
    onReceive(msg, *odrive);
  }
}
// BOTH ===================================
float getSign(){
  if (full_k > 0) {
    sign = 1.0;
  }
  else{
    sign = -1.0;
  }

  if (abs(full_k) < 0.1){
    sign = sign*0.1;
  }
  return sign;
}

float calcVel(float full_k){
  getSign();
  max_vel = 40; // was 10
  vel = 15*sign*full_k*full_k*full_k*full_k;
  if (abs(vel) > max_vel){
    vel = max_vel*sign;
  }
  // added for zero set point nullification
  // vel = 20-vel;
  // Serial.print("Vel: ");
  // Serial.print(vel, 4);
  return vel;
}

void setup() {
  Serial.begin(9600);

  // CAN SETUP ===========================================================================================================
  // Wait for up to 3 seconds for the serial port to be opened on the PC side.
  // If no PC connects, continue anyway.
  for (int i = 0; i < 30 && !Serial; ++i) {
    delay(100);
  }
  delay(200);

  Serial.println("Starting ODriveCAN demo");

  // Register callbacks for the heartbeat and encoder feedback messages
  odrv0.onFeedback(onFeedback, &odrv0_user_data);
  odrv0.onStatus(onHeartbeat, &odrv0_user_data);

  // Configure and initialize the CAN bus interface. This function depends on
  // your hardware and the CAN stack that you're using.
  if (!setupCan()) {
    Serial.println("CAN failed to initialize: reset required");
    while (true); // spin indefinitely
  }

  Serial.println("Waiting for ODrive...");
  while (!odrv0_user_data.received_heartbeat) {
    pumpEvents(can_intf);
    delay(100);
  }

  Serial.println("found ODrive");

  // request bus voltage and current (1sec timeout)
  Serial.println("attempting to read bus voltage and current");
  Get_Bus_Voltage_Current_msg_t vbus;
  if (!odrv0.request(vbus, 1)) {
    Serial.println("vbus request failed!");
    while (true); // spin indefinitely
  }

  Serial.print("DC voltage [V]: ");
  Serial.println(vbus.Bus_Voltage);
  Serial.print("DC current [A]: ");
  Serial.println(vbus.Bus_Current);

  Serial.println("Enabling closed loop control...");
  while (odrv0_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
    odrv0.clearErrors();
    delay(1);
    odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);

    // Pump events for 150ms. This delay is needed for two reasons;
    // 1. If there is an error condition, such as missing DC power, the ODrive might
    //    briefly attempt to enter CLOSED_LOOP_CONTROL state, so we can't rely
    //    on the first heartbeat response, so we want to receive at least two
    //    heartbeats (100ms default interval).
    // 2. If the bus is congested, the setState command won't get through
    //    immediately but can be delayed.
    for (int i = 0; i < 15; ++i) {
      delay(10);
      pumpEvents(can_intf);
    }
  }

  Serial.println("ODrive running!");

  // IMU SETUP ===========================================================================================================
  Serial.println("Orientation Sensor Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
    
  bno.setExtCrystalUse(true);
}


void loop() {
// IMU ===========================================================================================================
  sensors_event_t event; 
  bno.getEvent(&event);
  // phi_current = event.orientation.x;

  if (counter ==400) {
  // ValPoint();
  phieq = xval;
  betaeq = yval;
  counter ++; 
  }
  if (counter <400){
  counter ++;
  }

  full_k = PID();

  IMU_Data();

  diff_y = yval - betaeq; 
  diff_x = xval - phieq;
  // Serial.print("\tBeta: ");
  // Serial.print(betaeq, 4);

  // Serial.print("\tPhi: ");
  // Serial.print(phieq, 4);

  // Serial.print("Diff_X, ");
  // Serial.print(diff_x, 4);
  // Serial.print(", ");

  // Serial.print("\tDiff_y: ");
  // Serial.print(diff_y, 4);
  // Serial.println(""); 
  
  // Serial.print("\tFull K, ");
  // Serial.print(full_k, 4);
  // Serial.print(", ");

  // Serial.print("Vel: ");
  // Serial.print(vel, 4);
  // Serial.print(", ");



  // // Serial.print(phi_current, 4);
  // Serial.print("\tY: ");
  // Serial.print(event.orientation.y, 4);
  // Serial.println("");
  // delay(100);

    // CAN ===========================================================================================================
  pumpEvents(can_intf); // This is required on some platforms to handle incoming feedback CAN messages

  // THIS IS NEGATIVE TO TAKE ENERGY *OUT* OF THE SYSTEM!
  vel = 1*calcVel(full_k);
  // signal = 10.0*sign;
  // odrv0.setVelocity(vel);
  // // set torque max is 6.62

  // vel = 20;
  // signal = 10.0*sign;

  // odrv0.setVelocity(vel);

  // set torque max is 6.62

  // print position and velocity for Serial Plotter
  if (odrv0_user_data.received_feedback) {
    Get_Encoder_Estimates_msg_t feedback = odrv0_user_data.last_feedback;
    odrv0_user_data.received_feedback = false;
    // Serial.print("odrv0-pos:");
    // Serial.print(feedback.Pos_Estimate);
    // Serial.print(",");
    // Serial.print("odrv0-vel:");
    Serial.print(diff_x, 4);
    Serial.print(", ");
    // Serial.print(full_k, 4);
    // Serial.print(", ");
    Serial.print(feedback.Vel_Estimate);
    Serial.println(" ");
  }
}