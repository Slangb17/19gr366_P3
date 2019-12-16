#include "protocol2.h"
#include <math.h>

class crustcrawler {
  public:
    uint8_t id[5]; // joint ids
    bool torque_enabled = false; // initial torque state
    float d1 = 250, L1 = 220, L2 = 160; // length of links
    int32_t base_offset = 1937, shoulder_offset = 2048, elbow_offset = 2048; // offset to get desired zero position

    int32_t present_position[5]; // encoder ticks for each actuator
    float actual_angle[3], target_angle[3]; // the angle of the actuator, angle required to reach Cartesian target
    float actual_xyz[3], target_xyz[3]; // forward kinematics coordinates, and end-effector target
    float spherical_coordinates[3]; // spherical coordinates of target_xyz (radius r, inclination θ, azimuth φ)
    
    // Controller:
    float error_angle[3]; // calculated error angle from actual angle to target angle
    int32_t error_position[3]; // calculated error position from error angle

    // Constructor:
    crustcrawler(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, bool);

    // Member functions:
    void enable_torque(bool); // enables torque for all ids
    void set_pwm(uint16_t); // set desired pwm of all ids
    void joystick(); // control the end-effector with a joystick
    void gripper(int); // open and close the gripper
    void myo(); // control the end-effector with the Myo armband
    void update_target(float, float, float); // update the spherical coordinates
    bool update_position(); // get current encoder values from the actuators
    void forward_kinematics(); // determine the end-effector position
    void inverse_kinematics(); // determien the joint angles required to reach desired end-effector position
    void controller(); // calculating error and scaling with set P
    void move_to_target(); // instructing the actuators to move to new goal positions
};

crustcrawler::crustcrawler(uint8_t base_id, uint8_t shoulder_id, uint8_t elbow_id, uint8_t gripper1, uint8_t gripper2, bool torque_state) {
  id[0] = base_id;
  id[1] = shoulder_id;
  id[2] = elbow_id;
  id[3] = gripper1;
  id[4] = gripper2;

  enable_torque(torque_state);
  
  set_pwm(500);

  update_position();

  forward_kinematics();

  // First target is the position of the robot:
  target_xyz[0] = actual_xyz[0];
  target_xyz[1] = actual_xyz[1];
  target_xyz[2] = actual_xyz[2];

  // Calculating the spherical coordinates of the target:
  float X = target_xyz[0];
  float Y = target_xyz[1];
  float Z = target_xyz[2] - d1;

  spherical_coordinates[0] = sqrt(X * X + Y * Y + Z * Z);
  spherical_coordinates[1] = acos(Z / spherical_coordinates[0]);
  spherical_coordinates[2] = atan2(Y, X);
}

void crustcrawler::enable_torque(bool state) {

  uint8_t value = 0;
  if (state == true) value = 1;

  for (int i = 0; i < sizeof(id); i++) {
    send_write_instruction(id[i], 64, value);

    if (receive_package(100)) {
      torque_enabled = true;
    }
    else {
      torque_enabled = false;
      break;
    }
  }
}

void crustcrawler::set_pwm(uint16_t pwm) {
  for (int i = 0; i < sizeof(id); i++) {
    send_write_instruction(id[i], (uint16_t)100, (uint16_t)pwm); // setting PWM
    receive_package(100);
  }
}

void crustcrawler::joystick() {
  float i, a, r;

  //Analog pins for joysticks:
  const int I_pin = 0; // analog pin connected to inclination output
  const int A_pin = 1; // analog pin connected to azimuth output
  const int R_pin = 2; // analog pin connected to radius output

  r = analogRead(R_pin);
  i = analogRead(I_pin);
  a = analogRead(A_pin);

  r = map(r, 0, 1024, -3, 4);
  i = map(i, 0, 1024, -3, 4);
  a = map(a, 0, 1024, -3, 4);

  i = i * PI / 180.0;
  a = a * PI / 180.0 * -1;

  update_target(r, i, a);

  if (digitalRead(8) == LOW) {
    gripper(20);
  }

  if (digitalRead(9) == LOW) {
    gripper(-20);
  }
}

void crustcrawler::myo() {
  // Protocol variables:
  String data_buffer = "";
  bool start_found = false;
  bool end_found = false;

  // TRUE, if the amount of characters availble on the serial receive buffer is greater than 0:
  while (Serial.available() > 0) {
    char incomming_char = Serial.read();

    // If the start of the message has been received:
    if (incomming_char == '$' && start_found == false) {
      start_found = true;
      data_buffer = data_buffer + incomming_char;
    }
    else if (incomming_char == '$' && start_found == true) {
      data_buffer = "";
      data_buffer = incomming_char;
    }
    // Else if the end of the message has been received:
    else if (incomming_char == '#' && start_found == true) {
      end_found = true;
      data_buffer = data_buffer + incomming_char;
      break;
    }
    // Triggered if '$' has been received before:
    else if (start_found == true) {
      data_buffer = data_buffer + incomming_char;
    }
  }

  // If a full message has been received:
  if (start_found == true && end_found == true) {
    int8_t delimiter_found = 0;
    int8_t delimiter_index[3];

    // Target spherical coordinates:
    float add_r = -1, add_i = -1, add_a = -1;
    int pose = -1;

    for (int i = 0; i < data_buffer.length(); i++) {
      if (data_buffer[i] == ',') {
        delimiter_index[delimiter_found] = i;
        delimiter_found++;

        if (delimiter_found == 1) add_r = data_buffer.substring(1, delimiter_index[0]).toInt();
        else if (delimiter_found == 2) add_i = data_buffer.substring(delimiter_index[0] + 1, delimiter_index[1]).toInt() * -1;
        else if (delimiter_found == 3) add_a = data_buffer.substring(delimiter_index[1] + 1, delimiter_index[2]).toInt();
      }

      if (data_buffer[i] == '#') pose = data_buffer.substring(delimiter_index[2] + 1, data_buffer.length() - 1).toInt();
    }

    // Scaling r:
    add_r = add_r * 0.6;
    add_i = add_i * 0.3;
    add_a = add_a * 0.3;

    // Degrees to radians:
    add_i = add_i * PI / 180.0;
    add_a = add_a * PI / 180.0;

    // Updating spherical coordiantes:
    update_target(add_r, add_i, add_a);

    // Setting gripper action:
    if (pose == 1) gripper(-20);
    else if (pose == 2) gripper(20);

    start_found = false;
    end_found = false;
    data_buffer = "";
  }
}

void crustcrawler::gripper(int pos) {
  present_position[3] = present_position[3] + pos;
  present_position[4] = present_position[4] - pos;

  for (int i = 3; i < 5; i++) {
    send_write_instruction(id[i], 116, (int32_t)present_position[i]);
    receive_package(100);
  }
}

void crustcrawler::update_target(float add_r, float add_i, float add_a) {

  spherical_coordinates[0] = spherical_coordinates[0] + add_r; // radius
  spherical_coordinates[1] = spherical_coordinates[1] + add_i; // inclination
  spherical_coordinates[2] = spherical_coordinates[2] + add_a; // azimuth

  // Restricing workspace to fit the test scenario:
  if (spherical_coordinates[1] < 0.01) spherical_coordinates[1] = 0.01;
  if (spherical_coordinates[2] > 2.6) spherical_coordinates[2] = 2.6;
  else if (spherical_coordinates[2] < -2.6) spherical_coordinates[2] = -2.6;

  // Restricting acos to be undefined later:
  if (spherical_coordinates[0] > 380) spherical_coordinates[0] = 380;
  else if (spherical_coordinates[0] < 250) spherical_coordinates[0] = 250;

  // Spherical coordinates to cartesian:
  float X = spherical_coordinates[0] * sin(spherical_coordinates[1]) * cos(spherical_coordinates[2]);
  float Y = spherical_coordinates[0] * sin(spherical_coordinates[1]) * sin(spherical_coordinates[2]);
  float Z = spherical_coordinates[0] * cos(spherical_coordinates[1]);

  // Setting the target for inverse kinematics:
  target_xyz[0] = X;
  target_xyz[1] = Y;
  target_xyz[2] = Z + d1;
}

bool crustcrawler::update_position() {
  for (int i = 0; i < sizeof(id); i++) {
    send_read_instruction(id[i], 132, 4);

    if (receive_package(100)) {

      present_position[i] = get_int32t(rxBuffer, 9);

      // Homing offsets and angles of rotation:
      if (i == 0) present_position[0] = present_position[0] - base_offset;
      else if (i == 1) present_position[1] = present_position[1] - shoulder_offset;
      else if (i == 2) present_position[2] = present_position[2] - elbow_offset;

      if (i < 3) actual_angle[i] = present_position[i] * (360.0 / 4096.0);
    }
    else return false;
  }

  return true;
}

void crustcrawler::forward_kinematics() {
  // The equations for X, Y, and Z have been derived in the report:
  actual_xyz[0] = cos(actual_angle[0] / 180 * PI) * (L2 * cos((actual_angle[1] + actual_angle[2]) / 180 * PI) + L1 * cos(actual_angle[1] * PI / 180));
  actual_xyz[1] = sin(actual_angle[0] / 180 * PI) * (L2 * cos((actual_angle[1] + actual_angle[2]) / 180 * PI) + L1 * cos(actual_angle[1] * PI / 180));
  actual_xyz[2] = d1 + L2 * sin((actual_angle[1] + actual_angle[2]) / 180 * PI) + L1 * sin(actual_angle[1] / 180 * PI);
}

void crustcrawler::inverse_kinematics() {

  // Calculating base rotation utilizing frame 0 to T:
  float theta_1 = atan2(target_xyz[1], target_xyz[0]);

  // Calculating target X and Z in frame 1 to T:
  float X = target_xyz[0] / cos(theta_1);
  float Z = target_xyz[2] - d1;

  // Calculating length L3:
  float L3 = sqrt(X * X + Z * Z);

  // Calculating theta 3:
  float theta_3 = (acos(-1 * ((L1 * L1 + L2 * L2 - L3 * L3) / (2 * L1 * L2))));

  // Calculating theta 2:
  float phi1 = atan2(Z, X);
  float phi2 = asin((L2 * sin(PI - theta_3)) / L3);

  // Choosing the theta 3 requiring the smallest movement of the elbow:
  float actual_theta2_rad = actual_angle[2] * PI / 180;
  if (abs(actual_theta2_rad - theta_3) > abs(actual_theta2_rad - (-1 * theta_3))) {
    theta_3 = -1 * theta_3;
    phi2 = -1 * phi2;
  }

  // Theta 2:
  float theta_2 = phi1 - phi2;

  // Converting the found thetas from rad to deg:
  target_angle[0] = theta_1 * 180 / PI;
  target_angle[1] = theta_2 * 180 / PI;
  target_angle[2] = theta_3 * 180 / PI;
}

void crustcrawler::controller() {
  float P = 0.5;
  for (int i = 0; i < 3; i++) {
    // Finding the error:
    error_angle[i] = (actual_angle[i] - target_angle[i]) * P;

    // Converting angles to ticks:
    error_position[i] = (int32_t)(error_angle[i] * (4096.0 / 360.0));
  }
}


void crustcrawler::move_to_target() {
  int32_t new_position = 0;

  // Sending the new goal positions to the actuators:
  for (int i = 0; i < 3; i++) {
    if (i == 0) new_position = present_position[0] - error_position[0] + base_offset;
    if (i == 1) new_position = present_position[1] - error_position[1] + shoulder_offset;
    if (i == 2) new_position = present_position[2] - error_position[2] + elbow_offset;

    send_write_instruction(id[i], 116, (int32_t)new_position);

    // Receiving status:
    receive_package(100);
  }
}

void setup()
{
  // The UART serial port with the computer is opened with a baud rate of 57600 bps.
  Serial.begin(57600);

  // The UART serial port with the RS485 is opened with a baud rate of 57600 bps - (default of the dynamixel actuators):
  Serial1.begin(57600);

  // Setting digital pins as outputs:
  pinMode(2, OUTPUT); // RS485: Request to send (RTS)

  // Controller buttons:
  pinMode(8, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);
}

void loop() {

  // Constructing an object of the class crustcrawler:
  crustcrawler robot(1, 2, 3, 4, 5, true);

  // Setting loop frequency:
  float hertz = 1000 / 20;

  while (true) {

    long old_time = millis();

    // Running member functions:
    robot.update_position();
    robot.joystick();
    robot.myo();
    robot.inverse_kinematics();
    robot.controller();
    robot.move_to_target();

    while (millis() - old_time < hertz);
    //Serial.print("Loop time: "); Serial.println(millis() - old_time);
  }
}
