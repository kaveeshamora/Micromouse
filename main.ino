#include <EEPROM.h>

// LED
const uint8_t LED = A5;
// IR sensor
const uint8_t IRPIN = A1;
int IRsensorValue;
bool arrived_goal = false;
// switch
const uint8_t SWITCHPIN = A2;
// Motor pins
const int RightMotorForward = A0;
const int RightMotorBackward = 2;
const int LeftMotorForward = 4;
const int LeftMotorBackward = 5;

// Using timer2
#define right_pwm_pin 3 // right enable OC2B, PD3
#define left_pwm_pin 11 // left enable OC2A, PB3 = 11

int base_speed = 120;
int reduce_speed = 120; // previouly 100, with 64 prescaler 150, with 32 prescaler 50
int min_speed = 5;
int rightmotor_speed = 0;
int leftmotor_speed = 0;

// Ultrasonic Sensors right
#define trig_pin_right 6 //analog input 1
#define echo_pin_right 7 //analog input 2
// forward
#define trig_pin_forward  8 //analog input 1 fwd
#define echo_pin_forward 9 //analog input 2
// left
#define trig_pin_left  12 //analog input 1 lft
#define echo_pin_left 13 //analog input 2

int duration;
int distance_mm;
int distance_right;
int distance_forward;
int distance_left;

int threshold_right_wall = 77;
int threshold_left_wall = 77;

// PID constants
double Kp_left = 0.8;
double Kd_left = 0.5;

double Kp_right = 0.8;
double Kd_right = 0.5; 

// Variables for PD
double error = 0;
double previous_error = 0;
double derivative = 0;
double dt = 0.05;

// define prev_time and current_time
long prev_time = 0;
long current_time = 0;

// wall availability
bool rightwall_available = 0;
bool leftwall_available = 0;
bool frontwall_available = 0;

// parameters regarding step forward
bool step = 1;
int next_forward_distance = 0;

// error correction
bool no_wall_condition = false;

// 0=> west, 1=> north, 2=> east, 3=> south
uint8_t globdirection_array[4] = {0, 1, 2, 3};

// current row and column
uint8_t current_row = 0;
uint8_t current_col = 0;

// directional stack
uint8_t direction_stack[60][2];
int current_stack_index = -1;
// wall matrix
bool wall_matrix[9][9][4] =  {
                                {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}}, 
                                {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}}, 
                                {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}}, 
                                {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}}, 
                                {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}}, 
                                {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}}, 
                                {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}}, 
                                {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}}, 
                                {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}}
                              };

// bool wall_matrix[9][9][4] =  {
//                                 {{1, 1, 0, 1}, {0, 0, 1, 1}, {1, 1, 0, 1}, {0, 1, 0, 1}, {0, 0, 1, 1}, {1, 0, 0, 1}, {0, 0, 1, 1}, {1, 0, 0, 1}, {0, 1, 1, 1} }, 
//                                 {{1, 0, 0, 1}, {0, 1, 1, 0}, {1, 0, 0, 1}, {0, 0, 1, 1}, {1, 1, 0, 0}, {0, 0, 1, 0}, {1, 0, 1, 0}, {1, 0, 1, 0}, {1, 0, 1, 1} }, 
//                                 {{1, 0, 1, 0}, {1, 0, 0, 1}, {0, 1, 1, 0}, {1, 0, 0, 0}, {0, 0, 0, 1}, {0, 1, 1, 0}, {1, 0, 1, 0}, {1, 1, 0, 0}, {0, 0, 1, 0} }, 
//                                 {{1, 1, 0, 0}, {0, 0, 1, 0}, {1, 1, 0, 1}, {0, 1, 1, 0}, {1, 0, 0, 0}, {0, 1, 1, 1}, {1, 0, 0, 0}, {0, 1, 0, 1}, {0, 1, 1, 0} }, 
//                                 {{1, 0, 1, 1}, {1, 0, 1, 0}, {1, 0, 0, 1}, {0, 1, 0, 1}, {0, 0, 0, 0}, {0, 1, 0, 1}, {0, 0, 1, 0}, {1, 0, 0, 1}, {0, 1, 1, 1} }, 
//                                 {{1, 0, 0, 0}, {0, 0, 1, 0}, {1, 1, 0, 0}, {0, 0, 1, 1}, {1, 1, 1, 0}, {1, 0, 0, 1}, {0, 1, 0, 0}, {0, 1, 1, 0}, {1, 0, 1, 1} }, 
//                                 {{1, 0, 1, 0}, {1, 1, 0, 0}, {0, 0, 1, 1}, {1, 0, 0, 0}, {0, 0, 1, 1}, {1, 0, 1, 0}, {1, 0, 0, 1}, {0, 1, 0, 1}, {0, 1, 1, 0} }, 
//                                 {{1, 1, 1, 0}, {1, 0, 0, 1}, {0, 0, 1, 0}, {1, 0, 1, 0}, {1, 0, 1, 0}, {1, 1, 0, 0}, {0, 0, 0, 0}, {0, 1, 0, 1}, {0, 1, 1, 1} }, 
//                                 {{1, 1, 0, 1}, {0, 1, 1, 0}, {1, 1, 0, 0}, {0, 1, 1, 0}, {1, 1, 0, 0}, {0, 1, 0, 1}, {0, 1, 0, 0}, {0, 1, 0, 1}, {0, 1, 1, 1} } 
//                               };


// tracking matrix
bool maze_tracking_matrix[9][9] = {
                                  {0, 0, 0, 0, 0, 0, 0, 0, 0}, 
                                  {0, 0, 0, 0, 0, 0, 0, 0, 0}, 
                                  {0, 0, 0, 0, 0, 0, 0, 0, 0}, 
                                  {0, 0, 0, 0, 0, 0, 0, 0, 0},
                                  {0, 0, 0, 0, 0, 0, 0, 0, 0},
                                  {0, 0, 0, 0, 0, 0, 0, 0, 0},
                                  {0, 0, 0, 0, 0, 0, 0, 0, 0},
                                  {0, 0, 0, 0, 0, 0, 0, 0, 0},
                                  {0, 0, 0, 0, 0, 0, 0, 0, 0}
                                  };

// traverse end or not
bool end = false;
// goal pos
uint8_t goal_pos[2];
// pre arrived no wall condition
bool next_pos_no_wall = false;

// flood matrix for solving
uint8_t flood_fill_matrix[9][9] = { {127} };
// stack for floodfill solving
uint8_t flood_fill_stack[85][3];

// EEPROM address of goal col and row
int goalpos_col_eepromaddress = 500;
int goalpos_row_eepromaddress = 504;

void setup() {
  Serial.begin(9600); 

  // Changing pin modes
  change_pinmodes();

  // switch pin status. If switch on => search, save. else => load and solve
  bool switch_status = digitalRead(SWITCHPIN); 

  // PWM register level setting
  setup_PWM();

  // at the first run it gives wrong values
  check_wall_availability();

  //-----------------------------------start traversing depth first-----------------------------------
  Serial.println("Start traversing");
  while (true && switch_status){
    // check the walls
    check_wall_availability();
    // current position into the direction stack
    addto_direction_stack();
    // update wall matrix
    update_wall_matrix();
    // decide next command - prioritised array => forward, right, left
    rotate_decision_priority_config_1();

    if (end == true){
      break;
    }
    else{
      // Movestepforward needs latest distance values at first
      Measure_distances();

      Movestepforward();
      update_mouse_pos();

      // check whether current position is the goal => Put the code for checking the IR sensor
      check_goal();
    }
  }
  // Put the wall
  wall_matrix[0][0][3] = 1;

  // End of searching
  for (uint8_t i=0; i<5; i++){
    digitalWrite(LED, HIGH);
    delay(50);
    digitalWrite(LED, LOW);
    delay(50);
  }
  Serial.println("Searching is over");
  delay(1000);

  // save to EEPROM
  if (switch_status){
    for (uint8_t i=0; i<5; i++){
      digitalWrite(LED, HIGH);
      delay(50);
      digitalWrite(LED, LOW);
      delay(50);
    }
    savewallMatrixToEEPROM(); // save the wall matrix
    saveIntegerToEEPROM(goalpos_col_eepromaddress, goal_pos[0]); // save goal column
    saveIntegerToEEPROM(goalpos_row_eepromaddress, goal_pos[1]); // save goal row
    delay(1000);
  }
  else{
    // Load the saved wall matrix into same wall matrix
    for (uint8_t i=0; i<20; i++){
      digitalWrite(LED, HIGH);
      delay(50);
      digitalWrite(LED, LOW);
      delay(50);
    }
    load_saved_wall_to_wall_matrix(); // load the wall matrix
    goal_pos[0] = readIntegerFromEEPROM(goalpos_col_eepromaddress); // load goal column
    goal_pos[1] = readIntegerFromEEPROM(goalpos_row_eepromaddress); // load goal row
    
    // Solving started
    solve_maze();
    // Solving ended

    // Traversing started
    traverse_after_solve();
    // solving completed
    for (uint8_t i=0; i<5; i++){
      digitalWrite(LED, HIGH);
      delay(50);
      digitalWrite(LED, LOW);
      delay(50);
    }
  }
}

void loop() {
  for (uint8_t i=0; i<5; i++){
    digitalWrite(LED, HIGH);
    delay(50);
    digitalWrite(LED, LOW);
    delay(50);
  }
}

void traverse_after_solve(){

  globdirection_array[0] = 0;
  globdirection_array[1] = 1;
  globdirection_array[2] = 2;
  globdirection_array[3] = 3;

  delay(500);

  current_row = 0;
  current_col = 0;

  uint8_t current_val;

  while (true){
    check_wall_availability();
    current_val = flood_fill_matrix[current_row][current_col];
    
    // check for west side square, wnes
    if ((current_col - 1 >= 0) && (flood_fill_matrix[current_row][current_col - 1] == current_val - 1) && (wall_matrix[current_row][current_col][0] != 1)){
      // if head is at south turn right
      if (globdirection_array[1] == 3){
          turnRight();
          update_globdirection_array(1);
      }
      else if(globdirection_array[1] == 0){
          // pass
      }
      else if(globdirection_array[1] == 1){
          turnLeft();
          update_globdirection_array(-1);
      }
    }
    
    // check for north side square
    else if ((current_row + 1 <= 8) && (flood_fill_matrix[current_row + 1][current_col] == current_val - 1) && (wall_matrix[current_row][current_col][1] != 1)){
      if (globdirection_array[1] == 0){
          turnRight();
          update_globdirection_array(1);
      }
      else if(globdirection_array[1] == 1){
          // pass
      }
      else if(globdirection_array[1] == 2){
          turnLeft();
          update_globdirection_array(-1);
      }
    }
    
    // check for east side square    
    else if ((current_col + 1 <= 8) && (flood_fill_matrix[current_row][current_col + 1] == current_val - 1) && (wall_matrix[current_row][current_col][2] != 1)){
      if (globdirection_array[1] == 1){
          turnRight();
          update_globdirection_array(1);
      }
      else if(globdirection_array[1] == 2){
          // pass
      }
      else if(globdirection_array[1] == 3){
          turnLeft();
          update_globdirection_array(-1);
      }
    }
           
    // check for south side square      
    else if ((current_row - 1 >= 0) && (flood_fill_matrix[current_row - 1][current_col] == current_val - 1) && (wall_matrix[current_row][current_col][3] != 1)){
      if (globdirection_array[1] == 2){
          turnRight();
          update_globdirection_array(1);
      }
      else if(globdirection_array[1] == 3){
          // pass
      }
      else if(globdirection_array[1] == 0){
          turnLeft();
          update_globdirection_array(-1);
      }
    }
    
    update_mouse_pos();
    
    Movestepforward();

    if (current_col == goal_pos[0] && current_row == goal_pos[1]){
        break;
    }
  }
}

void initialise_flood_matrix(){
  for (int i = 0; i < 9; i++) {
    for (int j = 0; j < 9; j++) {
      flood_fill_matrix[i][j] = 127;
    }
  }
}

void printMatrix() {
  for (int i = 0; i < 9; i++) {
    for (int j = 0; j < 9; j++) {
      Serial.print(flood_fill_matrix[i][j]);
      Serial.print("\t"); 
    }
    Serial.println(); 
  }
  Serial.println(); 
}

void solve_maze(){
  current_stack_index = -1;
  initialise_flood_matrix();

  current_col = goal_pos[0]; // goal position column
  current_row = goal_pos[1]; // goal position row

  // add pos and value to the flood stack
  addto_floodfill_stack(current_col, current_row, 0);
  
  // fill the goal position with flood value 0
  flood_fill_matrix[current_row][current_col] = 0;

  uint8_t stack_element_col;
  uint8_t stack_element_row;
  uint8_t stack_element_val;

  uint8_t changing_pos_col;
  uint8_t changing_pos_row;
  uint8_t changing_pos_val;

  while (true){
    // load the stack elements
    stack_element_col = flood_fill_stack[0][0];
    stack_element_row = flood_fill_stack[0][1];
    stack_element_val = flood_fill_stack[0][2];

    // decide flood values according to the wall configuration of the current position
    changing_pos_val = stack_element_val + 1; // This can be used inside the conditions. but for optimization I put it outside
    
    // if no walls in west
    if (!wall_matrix[stack_element_row][stack_element_col][0]){
      changing_pos_col = stack_element_col - 1;
      changing_pos_row = stack_element_row;
      // if value doesn't changed previously add that position to the stack and add the value to the flood fill matrix
      if (flood_fill_matrix[changing_pos_row][changing_pos_col] == 127){
        flood_fill_matrix[changing_pos_row][changing_pos_col] = changing_pos_val;
        addto_floodfill_stack(changing_pos_col, changing_pos_row, changing_pos_val);
      }
    }
    // if no walls in north
    if (!wall_matrix[stack_element_row][stack_element_col][1]){
      changing_pos_col = stack_element_col;
      changing_pos_row = stack_element_row + 1;
      // if value doesn't changed previously add that position to the stack and add the value to the flood fill matrix
      if (flood_fill_matrix[changing_pos_row][changing_pos_col] == 127){
        addto_floodfill_stack(changing_pos_col, changing_pos_row, changing_pos_val);
        flood_fill_matrix[changing_pos_row][changing_pos_col] = changing_pos_val;
      }
    }
    // if no walls in east
    if (!wall_matrix[stack_element_row][stack_element_col][2]){
      changing_pos_col = stack_element_col + 1;
      changing_pos_row = stack_element_row;
      // if value doesn't changed previously, add that position to the stack and add the value to the flood fill matrix
      if (flood_fill_matrix[changing_pos_row][changing_pos_col] == 127){
        addto_floodfill_stack(changing_pos_col, changing_pos_row, changing_pos_val);
        flood_fill_matrix[changing_pos_row][changing_pos_col] = changing_pos_val;
      }
    }
    // if no walls in south
    if (!wall_matrix[stack_element_row][stack_element_col][3]){
      changing_pos_col = stack_element_col;
      changing_pos_row = stack_element_row - 1;
      // if value doesn't changed previously, add that position to the stack and add the value to the flood fill matrix
      if (flood_fill_matrix[changing_pos_row][changing_pos_col] == 127){
        addto_floodfill_stack(changing_pos_col, changing_pos_row, changing_pos_val);
        flood_fill_matrix[changing_pos_row][changing_pos_col] = changing_pos_val;
      }
    }

    // remove the first element from floodfill_stack
    removefrom_floodfill_stack();

    // There's no element left in the floodfill_stack
    if (current_stack_index == -1){
      break;
    }

    Serial.println(" ");
  }
}

void addto_floodfill_stack(uint8_t col, uint8_t row, uint8_t val){
  current_stack_index += 1;
  flood_fill_stack[current_stack_index][0] = col;
  flood_fill_stack[current_stack_index][1] = row;
  flood_fill_stack[current_stack_index][2] = val;
}

void removefrom_floodfill_stack(){
  for (int i = 0; i < current_stack_index; i++){
    flood_fill_stack[i][0] = flood_fill_stack[i+1][0];
    flood_fill_stack[i][1] = flood_fill_stack[i+1][1];
    flood_fill_stack[i][2] = flood_fill_stack[i+1][2];
  }
  current_stack_index -= 1;
}

void setup_PWM() {
    // Timer 2 Fast PWM Mode, prescaler = 64 (976 Hz), non inverting
    TCCR2A |= (1 << WGM20) | (1 << WGM21);
    TCCR2B |= (1 << CS21) | (1 << CS20); //prescaler = 32
    // TCCR2B |= (1 << CS22); // prescaler = 64
    TCCR2A |= (1 << COM2A1) | (1 << COM2B1);
    setup_PWM_duty_cycle(min_speed, min_speed);
}

void setup_PWM_duty_cycle(uint8_t right_speed, uint8_t left_speed) {
    OCR2B = right_speed;
    OCR2A = left_speed;
}

void check_goal(){
  IRsensorValue = analogRead(IRPIN);
  if (IRsensorValue <= 500){
    goal_pos[0] = current_col;
    goal_pos[1] = current_row;
    arrived_goal = true;
    saveIntegerToEEPROM(goalpos_col_eepromaddress, goal_pos[0]); // save goal column
    saveIntegerToEEPROM(goalpos_row_eepromaddress, goal_pos[1]); // save goal row
    for (uint8_t i=0; i<5; i++){
      digitalWrite(LED, HIGH);
      delay(50);
      digitalWrite(LED, LOW);
      delay(50);
    }
  }
}

void goto_stack_prev_pos(){
  // stack position previously to the current position
  uint8_t stack_prevpos_col = direction_stack[current_stack_index-1][0];
  uint8_t stack_prevpos_row = direction_stack[current_stack_index-1][1];
  uint8_t mouse_headdir = globdirection_array[1];

  // if previous pos is at south to the current pos
  if (stack_prevpos_row == current_row - 1){
    // if head is at east, west, south => rightturn, leftturn, just pass
    if (mouse_headdir == 2){
      turnRight();
      update_globdirection_array(1);
    }
    else if(mouse_headdir == 0){
      turnLeft();
      update_globdirection_array(-1);
    }
  }
  // if previous pos is at north to the current pos
  else if (stack_prevpos_row == current_row + 1){
    // if head is at west, east, north => rightturn, leftturn, just pass
    if (mouse_headdir == 0){
      turnRight();
      update_globdirection_array(1);
    }
    else if(mouse_headdir == 2){
      turnLeft();
      update_globdirection_array(-1);
    }
  }
  // if previous pos is at east to the current pos
  else if (stack_prevpos_col == current_col + 1){
    // if head is at north, south, east => rightturn, leftturn, just pass
    if (mouse_headdir == 1){
      turnRight();
      update_globdirection_array(1);
    }
    else if(mouse_headdir == 3){
      turnLeft();
      update_globdirection_array(-1);
    }
  }
  // if previous pos is at west to the current pos
  else if (stack_prevpos_col == current_col - 1){
    // if head is at south, north, west => rightturn, leftturn, just pass
    if (mouse_headdir == 3){
      turnRight();
      update_globdirection_array(1);
    }
    else if(mouse_headdir == 1){
      turnLeft();
      update_globdirection_array(-1);
    }
  }
}

void update_mouse_pos(){
  // global direction of front side
  uint8_t glob_dir = globdirection_array[1];

  if (glob_dir == 0){
    current_col -= 1;
  }
  else if (glob_dir == 1){
    current_row += 1;
  }
  else if (glob_dir == 2){
    current_col += 1;
  }
  else if (glob_dir == 3){
    current_row -= 1;
  }
}

void update_globdirection_array(int adder){
  int new_val;
  for(uint8_t i = 0; i < 4; i++){
    new_val = globdirection_array[i] + adder;
    if (new_val == -1){
      new_val = 3;
    }
    else if(new_val == 4){
      new_val = 0;
    }
    // update the array
    globdirection_array[i] = new_val;
  }
}

// left => 0, forward => 1, right => 2
// argument is the direction which should be checked to ensure it is not previously traversed.
// argument is 0 or 1 or 2 => represent left or front or right
bool check_not_traversed_before(uint8_t check_dir){
  uint8_t next_row = current_row;
  uint8_t next_col = current_col;
  uint8_t glob_dir = globdirection_array[check_dir];

  if (glob_dir == 0){
    next_col -= 1;
  }
  else if (glob_dir == 1){
    next_row += 1;
  }
  else if (glob_dir == 2){
    next_col += 1;
  }
  else if (glob_dir == 3){
    next_row -= 1;
  }
  return !maze_tracking_matrix[next_row][next_col];
}

void rotate_decision_priority_config_1(){
  // we are prioritising to front available path, next right available path, next left available path

  if ((!frontwall_available) && (check_not_traversed_before(1))){
    // just pass 
  }
  else if ((!rightwall_available) && (check_not_traversed_before(2))){
    // turn right and update the global direction array
    turnRight();                  
    update_globdirection_array(1);
  }
  else if ((!leftwall_available) && (check_not_traversed_before(0))){
    // turn left and update the global direction array
    turnLeft();                  
    update_globdirection_array(-1);
  }
  else{
    // This means front, left, right all are blocked or all are traversed before. So turn and follow until not traversed block meet
    // Turn 180 degree and update the global direction array
    turn180();
    update_globdirection_array(1);
    update_globdirection_array(1);
    delay(500);

    // Follow tracked path until not traversed block arrived.
    while (true){      
      // Movestepforward needs latest distance values at first
      Measure_distances();

      Movestepforward();
      update_mouse_pos();

      // if it comes back to the starting position => traversed completed
      if (current_col == 0 && current_row == 0){
        end = true;
        break;
      }

      // pop the previous position from direction stack
      popfrom_direction_stack();

      // check the walls
      check_wall_availability();

      // check for not traversed positions => prioritising front, right, left
      if ((!frontwall_available) && (check_not_traversed_before(1))){
        // if front pos is not traversed, break the loop
        break;
      }
      else if ((!rightwall_available) && (check_not_traversed_before(2))){
        // if right pos is not traversed, turn right and break the loop
        turnRight();
        update_globdirection_array(1);
        break;
      }
      else if ((!leftwall_available) && (check_not_traversed_before(0))){
        // if left pos is not traversed, turn left and break the loop
        turnLeft();
        update_globdirection_array(-1);
        break;
      }

      // if stack stack_prevpos is previously traversed
      goto_stack_prev_pos();
    }
  }
}

void savewallMatrixToEEPROM(){
  int address = 0; // EEPROM address starts at 0
  for (int i = 0; i < 9; i++) {
    for (int j = 0; j < 9; j++) {
      for (int k = 0; k < 4; k++) {
        if (address < EEPROM.length()) { // Ensure we don't overflow EEPROM
          EEPROM.write(address, wall_matrix[i][j][k]);
          address++;
        }
      }
    }
  }
}

void load_saved_wall_to_wall_matrix(){
  int address = 0; // EEPROM address starts at 0
  for (int i = 0; i < 9; i++) {
    for (int j = 0; j < 9; j++) {
      for (int k = 0; k < 4; k++) {
        if (address < EEPROM.length()) { // Ensure we don't overflow EEPROM
          wall_matrix[i][j][k] = EEPROM.read(address);
          address++;
        }
      }
    }
  }
}

void saveIntegerToEEPROM(int address, int value) {
  EEPROM.put(address, value);
}

int readIntegerFromEEPROM(int address) {
  return EEPROM.read(address);
}

void update_wall_matrix(){
  uint8_t glob_dir;
  if (leftwall_available){
    glob_dir = globdirection_array[0];
    wall_matrix[current_row][current_col][glob_dir] = 1;
  }
  if (frontwall_available){
    glob_dir = globdirection_array[1];
    wall_matrix[current_row][current_col][glob_dir] = 1;
  }
  if (rightwall_available){
    glob_dir = globdirection_array[2];
    wall_matrix[current_row][current_col][glob_dir] = 1;
  }
  maze_tracking_matrix[current_row][current_col] = 1;
}

void addto_direction_stack(){
  current_stack_index += 1;
  direction_stack[current_stack_index][0] = current_col;
  direction_stack[current_stack_index][1] = current_row;
}

void popfrom_direction_stack(){
  current_stack_index -= 1;
}

bool check_next_pos_is_no_wall(uint8_t check_dir){
  uint8_t next_row = current_row;
  uint8_t next_col = current_col;
  uint8_t glob_dir = globdirection_array[check_dir];

  if (glob_dir == 0){
    next_col -= 1;
  }
  else if (glob_dir == 1){
    next_row += 1;
  }
  else if (glob_dir == 2){
    next_col += 1;
  }
  else if (glob_dir == 3){
    next_row -= 1;
  }

  for (int i=0; i<4; i++){
    if (wall_matrix[next_row][next_col][i] == 1){
      return false;
    }
  }
  return true;
}

void Movestepforward(){
  // Activate the required pins for the forward. This doesn't make the wheel go front since after a break the initial pwm is always reduce to 5.
  startmoveForward();

  // check whether next destination is a previously traversed block and it is a no wall condition. This is useful to eliminate collisions at noisy no wall coniditions.
  bool next_pos_no_wall = false;
  if ((!check_not_traversed_before(1)) && (check_next_pos_is_no_wall(1))){
    next_pos_no_wall = true;
  }

  // noisy point can be occured in a place like continuos blocks are ahead.
  check_wall_availability();
  bool following_is_left_wall = false;
  bool following_is_both_walls = false;

  if (leftwall_available && !rightwall_available){
    following_is_left_wall = true;
  }
  if (leftwall_available && rightwall_available){
    following_is_both_walls = true;
  }

  // if next pos is a no wall condition
  if (next_pos_no_wall){
    // If available passage occurs in left or right, that can be taken to correct the error.
    bool prev_step_left_wall = leftwall_available;
    bool prev_step_right_wall = rightwall_available;
    bool state_change = false;
    long timetocheck_statechangeStop;
    long timetocheck = millis() + 1050;

    while (millis() < timetocheck){
      check_wall_availability();
      // state change action
      if ((prev_step_left_wall != leftwall_available) || (prev_step_right_wall != rightwall_available)){
        state_change = true;
        prev_step_left_wall = leftwall_available;
        prev_step_right_wall = rightwall_available;
        timetocheck_statechangeStop = millis() + 670;
      }
      if ((state_change) && (millis() > timetocheck_statechangeStop)){
        break;
      }
      setup_PWM_duty_cycle(120, 110);
    }
  }

  // good to measure step from forward distance
  else if (distance_forward < 570){
    // decide when to stop
    if (distance_forward < 360){
      next_forward_distance = 70;
    }
    else{
      next_forward_distance = 325;
    }

    long timetocheck_fullStop = millis() + 1100;

    while(distance_forward > next_forward_distance){
      // solution for noisy data call the function and above all walls are covered with left and right walls.
      if (timetocheck_fullStop < millis()){
        break;
      }

      check_wall_availability();

      if (leftwall_available){
        threshold_left_wall = constrain(distance_left, 75, 80);   
        // Left wall detected
        follow_left_wall();
      }

      else if (rightwall_available){
        threshold_right_wall = constrain(distance_right, 75, 80);   
        // right wall detected
        follow_right_wall();
      }

      else {
        if (following_is_left_wall){
          setup_PWM_duty_cycle(100, 125);
        }
        else if(following_is_both_walls){
          setup_PWM_duty_cycle(110, 100);
        }
        else{
          setup_PWM_duty_cycle(100, 100);
        }
        no_wall_condition = true;
      }
    }
  }

  // good to measure step from time steps
  else{
    // If available passage occurs in left or right, that can be taken to correct the error.
    bool prev_step_left_wall = leftwall_available;
    bool prev_step_right_wall = rightwall_available;
    bool state_change = false;
    long timetocheck_statechangeStop;
    long timetocheck = millis() + 1050;

    while (millis() < timetocheck){
      check_wall_availability();
      // state change action
      if ((prev_step_left_wall != leftwall_available) || (prev_step_right_wall != rightwall_available)){
        state_change = true;
        prev_step_left_wall = leftwall_available;
        prev_step_right_wall = rightwall_available;
        timetocheck_statechangeStop = millis() + 670;
      }
      if ((state_change) && (millis() > timetocheck_statechangeStop)){
        break;
      }

      if (leftwall_available){
        threshold_left_wall = constrain(distance_left, 75, 80);   
        // Left wall detected
        follow_left_wall();
      }

      else if (rightwall_available){
        threshold_right_wall = constrain(distance_right, 75, 80);   
        // right wall detected
        follow_right_wall();
      }

      else {
        if (following_is_left_wall){
          setup_PWM_duty_cycle(100, 125);
        }
        else if(following_is_both_walls){
          setup_PWM_duty_cycle(110, 100);
        }
        else{
          setup_PWM_duty_cycle(100, 100);
        }
        no_wall_condition = true;
      }
    }
  }
  Break();
  delay(300);

}

void follow_right_wall(){
  // error
  error = threshold_right_wall - distance_right;

  // Proportional term
  double Pout = Kp_right * error;

  // Derivative term 
  derivative = (error - previous_error) / dt;
  double Dout = Kd_right * derivative;

  double output = Pout + Dout;
  previous_error = error;

  rightmotor_speed = base_speed + output;
  leftmotor_speed = base_speed - output;

  // speed is constraints to 60 and 160
  rightmotor_speed = constrain(rightmotor_speed, 40, 220); 
  leftmotor_speed = constrain(leftmotor_speed, 40, 220); 

  setup_PWM_duty_cycle(rightmotor_speed, leftmotor_speed);
  // startmoveForward();
}

void follow_left_wall(){
  // error
  error = threshold_left_wall - distance_left;

  // Proportional term
  double Pout = Kp_left * error;

  // Derivative term 
  derivative = (error - previous_error) / dt;
  double Dout = Kd_left * derivative;

  double output = Pout + Dout;
  previous_error = error;

  rightmotor_speed = base_speed - output;
  leftmotor_speed = base_speed + output;

  // speed is constraints to 60 and 160
  rightmotor_speed = constrain(rightmotor_speed, 40, 220); 
  leftmotor_speed = constrain(leftmotor_speed, 40, 220);

  setup_PWM_duty_cycle(rightmotor_speed, leftmotor_speed);
  // startmoveForward();
}

void startmoveForward(){
  digitalWrite(RightMotorForward, HIGH);
  digitalWrite(LeftMotorForward, HIGH);
  
  digitalWrite(RightMotorBackward, LOW);
  digitalWrite(LeftMotorBackward, LOW);
}

void turnRight(){
  if (frontwall_available){
    pin_setting_right_turn();
    // turning right with check conditions
    while(ultrasonic_sensor_distance(trig_pin_forward, echo_pin_forward) < 180){}
    delay(155);
    Break();
  }
  else if (leftwall_available){
    pin_setting_right_turn();
    // turning right with check conditions
    while(ultrasonic_sensor_distance(trig_pin_left, echo_pin_left) < 180){}
    delay(210);
    Break();
  }
  else{
    pin_setting_right_turn();
    delay(440);
    Break();
  }
  // Reset PID paramerters
  double previous_error = 0;
}

void pin_setting_right_turn(){
  digitalWrite(RightMotorBackward, HIGH);
  digitalWrite(LeftMotorForward,  HIGH);
  digitalWrite(RightMotorForward,  LOW);
  digitalWrite(LeftMotorBackward, LOW);
  // motor driver config for turn right
  setup_PWM_duty_cycle(reduce_speed, reduce_speed);
}

void turnLeft(){
  if (frontwall_available){
    pin_setting_left_turn();
    // turning right with check conditions
    while(ultrasonic_sensor_distance(trig_pin_forward, echo_pin_forward) < 180){}
    delay(155);
    Break();
  }
  else if (rightwall_available){
    pin_setting_left_turn();
    // turning right with check conditions
    while(ultrasonic_sensor_distance(trig_pin_right, echo_pin_right) < 180){}
    delay(250);
    Break();
  }
  else{
    pin_setting_left_turn();
    delay(550);
    Break();
  }
  // Reset PID paramerters
  double previous_error = 0;
}

void pin_setting_left_turn(){
  digitalWrite(RightMotorForward,  HIGH);
  digitalWrite(LeftMotorBackward,  HIGH);
  digitalWrite(RightMotorBackward, LOW);
  digitalWrite(LeftMotorForward, LOW);

  // motor driver config for turn left
  setup_PWM_duty_cycle(100, 100);
}

void turn180(){
  check_wall_availability(); 

  // if left, right, front all are blocked with walls or front and left or front and right walls only
  if (frontwall_available){
    // if left, right, front walls are available
    bool right_rotate = false; // To make sure it rotated 180 degree correctly.
    if (leftwall_available && rightwall_available){
      if (distance_right > distance_left){
        right_rotate = true;
        pin_setting_180_turn(1);
      }
      else{
        pin_setting_180_turn(0);
      }
      while(ultrasonic_sensor_distance(trig_pin_forward, echo_pin_forward) < 180){}
      delay(130);
      Break();
      // verify the rotation
      if (right_rotate){
        check_wall_availability();
        if (frontwall_available){
          turnRight();
        }
      }
      else{
        check_wall_availability();
        if (frontwall_available){
          turnLeft();
        }
      } 
    }
    // if only front and right walls available
    else if(rightwall_available){
      pin_setting_180_turn(1); 
      while(ultrasonic_sensor_distance(trig_pin_forward, echo_pin_forward) < 180){}
      delay(120);
      Break();
      // verify the 180 degree rotation
      check_wall_availability();
      if (frontwall_available){
        turnRight();
      }
    }
    // if only front and left walls available
    else if(leftwall_available){
      pin_setting_180_turn(0); 
      while(ultrasonic_sensor_distance(trig_pin_forward, echo_pin_forward) < 180){}
      delay(120);
      Break();
      // verify the 180 degree rotation
      check_wall_availability();
      if (frontwall_available){
        turnLeft();
      }
    }
    // if front only (warning conditions of rotate using other sensor feedbacks and delays)
    else{
      pin_setting_180_turn(1); 
      while(ultrasonic_sensor_distance(trig_pin_forward, echo_pin_forward) < 180){}
      delay(120);
      while(ultrasonic_sensor_distance(trig_pin_left, echo_pin_left) < 180){}
      delay(150);
      Break();
    } 
  }
  // no front wall. right wall available. left may be there or not. (warning conditions of rotate using other sensor feedbacks and delays)
  else if (rightwall_available){
    pin_setting_180_turn(1); 
    while(ultrasonic_sensor_distance(trig_pin_right, echo_pin_right) < 180){}
    delay(160);
    while(ultrasonic_sensor_distance(trig_pin_forward, echo_pin_forward) < 180){}
    delay(150);
    Break();
  }
  // no front or right. Only leftwall available. (warning conditions of rotate using other sensor feedbacks and delays)
  else if (leftwall_available){
    pin_setting_180_turn(0); 
    while(ultrasonic_sensor_distance(trig_pin_left, echo_pin_left) < 180){}
    delay(160);
    while(ultrasonic_sensor_distance(trig_pin_forward, echo_pin_forward) < 180){}
    delay(150);
    Break();
  }
  // no left, right, front wall avalable. (warning conditions of rotate using other sensor feedbacks and delays)
  else{
    pin_setting_180_turn(1);
    delay(800);
    Break();
  }
  
  // Reset PID paramerters
  double previous_error = 0;
}

void pin_setting_180_turn(bool priority){
  if (priority){
    digitalWrite(RightMotorBackward, HIGH);
    digitalWrite(LeftMotorForward,  HIGH);
    digitalWrite(RightMotorForward,  LOW);
    digitalWrite(LeftMotorBackward, LOW);
    // motor driver config for turn right
    setup_PWM_duty_cycle(reduce_speed, reduce_speed);
  }
  else{
    digitalWrite(RightMotorForward,  HIGH);
    digitalWrite(LeftMotorBackward,  HIGH);
    digitalWrite(RightMotorBackward, LOW);
    digitalWrite(LeftMotorForward, LOW);
    // motor driver config for turn left
    setup_PWM_duty_cycle(reduce_speed, reduce_speed);
  }
}

void Stop(){
  digitalWrite(RightMotorForward,  LOW);
  digitalWrite(RightMotorBackward, LOW);
  
  digitalWrite(LeftMotorForward,  LOW);
  digitalWrite(LeftMotorBackward, LOW);
}

void Break(){
  digitalWrite(RightMotorBackward, HIGH);
  digitalWrite(LeftMotorBackward, HIGH);

  digitalWrite(RightMotorForward,  LOW);
  digitalWrite(LeftMotorForward,  LOW);

  delay(30);
  Stop();
  setup_PWM_duty_cycle(min_speed, min_speed);
}

void change_pinmodes(){
  // Motor connected pins
  pinMode(RightMotorForward, OUTPUT);
  pinMode(RightMotorBackward, OUTPUT);
  pinMode(LeftMotorForward, OUTPUT);
  pinMode(LeftMotorBackward, OUTPUT);
  // Motor control (PWM) pins
  pinMode(left_pwm_pin, OUTPUT);
  pinMode(right_pwm_pin, OUTPUT);
  // Ultrasonic pins
  pinMode(trig_pin_right, OUTPUT);
  pinMode(echo_pin_right, INPUT);
  pinMode(trig_pin_forward, OUTPUT);
  pinMode(echo_pin_forward, INPUT);
  pinMode(trig_pin_left, OUTPUT);
  pinMode(echo_pin_left, INPUT);
  // LED
  pinMode(LED, OUTPUT);
  // SWITCH
  pinMode(SWITCHPIN, INPUT);
}

void check_wall_availability(){
  Measure_distances();
  frontwall_available = (distance_forward < 160) ? true : false;
  rightwall_available = (distance_right < 200) ? true : false;
  leftwall_available = (distance_left < 200) ? true : false;
}

void Measure_distances(){
  distance_right = ultrasonic_sensor_distance(trig_pin_right, echo_pin_right);
  distance_forward = ultrasonic_sensor_distance(trig_pin_forward, echo_pin_forward);
  distance_left = ultrasonic_sensor_distance(trig_pin_left, echo_pin_left);
}

int ultrasonic_sensor_distance (int trigPin, int echoPin){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  duration = pulseIn(echoPin, HIGH);
  distance_mm = duration / 2 / 3;
  return distance_mm;
}