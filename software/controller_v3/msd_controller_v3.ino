#include "ArduinoJson.h"
#include "DCServo.h"
#include "delta_kinematics.h"

// Scheduling
long last_servo_update = 0;
long servo_update_period = 1;
long last_heartbeat = 0;
long heartbeat_period = 1000;
long last_graphing = 0;
long graphing_period = 50;

// Servo configuration
const int num_servos = 3;
const int servo_feeback_pins[num_servos] = {A8, A10, A9};
const int servo_motornum[num_servos] = {1,3,2};
DCServo *servos[num_servos];
float kp = 8, ki = 0.005, kd = 60;
int out_max = 255, out_min = -255;
int debug_mode = 1;
int deadband = 120;
double count_per_degree = 5.6889; // servo units per degree
double center_position[] = {540,550,505}; // center in servo units
double limit_max = 120; // degrees
double limit_min = -120; // degrees

void updateServoParams () {
  for (int i=0; i<num_servos; i++) {
    servos[i]->pid.kp = kp;
    servos[i]->pid.ki = ki;
    servos[i]->pid.kd = kd;
    servos[i]->pid.out_max = out_max;
    servos[i]->pid.out_min = out_min;
    servos[i]->pid.max_ramp_rate = (out_max - out_min); // full range
    servos[i]->limit_max = limit_max;
    servos[i]->limit_min = limit_min;
    servos[i]->motor.deadband = deadband;
    servos[i]->zero_offset = center_position[i] / count_per_degree;
    servos[i]->scaling = count_per_degree;
    servos[i]->motor_update_interval = servo_update_period;
    
  }
}

// JSON Buffer
StaticJsonBuffer<400> jsonBuffer;

// Positioning
// All units in mm and degrees
double cart_offset[] = {0,0,-225};
double target_cart[3] = {0,0,0};
const float init_end_effector_radius = 50; // radius from the center of the end effector to the effective link point
const float init_base_radius = 50; // radius from the center of the base to the point of rotation of horns
const float init_link_length = 173; // linkage arm length
const float init_horn_length = 100; // servo horn length
DeltaKinematics kine(init_end_effector_radius, init_base_radius, init_link_length, init_horn_length);

void updateJointTargets() {
  double theta1, theta2, theta3;
  Serial.print("Updating joint positions for cart: ");Serial.print(target_cart[0]);Serial.print(", ");Serial.print(target_cart[1]);Serial.print(", ");Serial.println(target_cart[2]);
  if (kine.cartToJoint(target_cart[0], target_cart[1], target_cart[2], &theta1, &theta2, &theta3) < 0) {
    Serial.println("invalid position");
  } else {
    Serial.print("Updated joint positions: ");
    Serial.print(theta1);
    Serial.print(", ");
    Serial.print(theta2);
    Serial.print(", ");
    Serial.println(theta3);
    
    servos[0]->target = -theta1;
    servos[1]->target = -theta2;
    servos[2]->target = -theta3;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting");

  for (int i=0; i<num_servos; i++) {
    servos[i] = new DCServo(servo_motornum[i], servo_feeback_pins[i], limit_min, limit_max);
    servos[i]->enable();
  }
  updateServoParams();

  // pot power
  pinMode(53, OUTPUT);
  digitalWrite(53, HIGH);
  pinMode(52, OUTPUT);
  digitalWrite(52, HIGH);
  pinMode(51, OUTPUT);
  digitalWrite(51, HIGH);
  pinMode(49, OUTPUT);
  digitalWrite(49, LOW);
}

void loop() {
  // Process commands
  if (Serial.available()) {
    if (Serial.peek() == '{') {
      // Probably a message in there somewhere
      
      JsonObject& message = jsonBuffer.parse(Serial);
  
      if (message != JsonObject::invalid()) {
        if (message.containsKey("x")) {
          Serial.print("Cartesian Targets: ");
          for (int i=0; i<3; i++) {
            target_cart[i] = double(message["x"][i]) + cart_offset[i];
            Serial.print(target_cart[i]);Serial.print(", ");
          }
          
          Serial.println();
          updateJointTargets();
    
          Serial.print("Joint Targets: ");
          for (int i=0; i<num_servos; i++) {
            Serial.print(servos[i]->target);Serial.print(", ");
          }
          Serial.println();
        }
    
        if (message.containsKey("j")) {
          Serial.print("Joint Targets: ");
          for (int i=0; i<num_servos; i++) {
            servos[i]->target = message["j"][i];
            Serial.print(servos[i]->target);Serial.print(", ");
          }
          Serial.println();
        }
        
        if (message.containsKey("debug_mode")) debug_mode = message["d"];
        if (message.containsKey("kp")) kp = message["kp"];
        if (message.containsKey("ki")) ki = message["ki"];
        if (message.containsKey("kd")) kd = message["kd"];
        if (message.containsKey("out_max")) out_max = message["out_max"];
        if (message.containsKey("out_min")) out_max = message["out_min"];
        if (message.containsKey("deadband")) deadband = message["deadband"];
        if (message.containsKey("limit_max")) limit_max = message["limit_max"];
        if (message.containsKey("limit_min")) limit_min = message["limit_min"];
        if (message.containsKey("count_per_degree")) count_per_degree = message["count_per_degree"];
        if (message.containsKey("center_position")) {
          for (int i=0; i<3; i++) servos[i]->zero_offset = message["center_position"][i];
        }
        
        updateServoParams();
        jsonBuffer.clear();
      }
    } else {
      Serial.println("Serial data doesn't seem good:");
      // Clear data that's not the start or end of object
      char c = Serial.read();
      while (c != '{' && c != -1 && c != 255){
        Serial.write(c);
        c = Serial.read();
      }
      Serial.println();
    }
  }
  
  // Scheduled tasks
  long t = millis();

  for (int i=0;i<num_servos;i++) {
    servos[i]->update(t);
  }

  if (last_heartbeat + heartbeat_period <= t) {
    Serial.println("boo-doom");
    last_heartbeat = t;
  }

  if (debug_mode == 1) {
    if (last_graphing + graphing_period <= t) {
      for (int i = 0; i < num_servos; i++) {
        // Some multiplied by 10 to improve visibility given scaling
        Serial.print(servos[i]->target * 10);Serial.print(",");
        Serial.print(servos[i]->feedback * 10);Serial.print(",");
        Serial.print(servos[i]->output);Serial.print(",");
      }
      Serial.println();

      last_graphing = t;
    }
  }
}
