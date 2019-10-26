#include <MCP3008.h>


// define pin connections
#define CS_PIN 12
#define CLOCK_PIN 9
#define MOSI_PIN 11
#define MISO_PIN 10

// put pins inside MCP3008 constructor
MCP3008 adc(CLOCK_PIN, MOSI_PIN, MISO_PIN, CS_PIN);




//*** Variables ***:
int ir[9];
int run_tracker = 0;       // determines if Dry Run or Actual Run
char node, turn;
char memory[50];          // For saving the maze path
int index1 = 0, index2 = 0;            // For indexing the memory array
float left_base_speed=180, right_base_speed=180, left_speed, right_speed;
float error, previous_error, total_error;         // LINE_TRACK() variables
int sum;
int L1=7, L2=8, L3=5, R1=4, R2=6, R3=3;                                       // Motor Pins
float kp=1.4, kd=3.35;                                                               // Value of kp and kd
int threshold = 350;

//*** Functions ***:
void DRY_RUN();                                      // Main Function - 01
void Actual_RUN();                                   // Main Function - 02
void SENSOR_READ();                                  // 1
void LINE_TRACK();                                   // 2
char NODE_CHECK();                                   // 3
void TURN(char node, char turn);                     // 4
void MOTOR(int left_speed, int right_speed);         // 5
void CALCULATE_THE_SHORTEST_PATH();                  // 6
void SENSOR_PRINT();


void setup() {
  // put your setup code here, to run once:
  pinMode(2,OUTPUT);
  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(13, LOW);
  delay(500);
  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(13, LOW);
  Serial.begin(9600);
//  digitalWrite(2,HIGH);
//  delay(500);
//  digitalWrite(2,LOW);
//  delay(500);
}

void loop() {

  SENSOR_READ();
//  LINE_TRACK();
//    MOTOR(100, -100);
//  DRY_RUN();
//    SENSOR_READ();
//    delay(100);
//    SENSOR_PRINT();
//    NODE_CHECK();
//MOTOR(150, 150);
}


void DRY_RUN(){                                          // Main Function - 01
  
  while(run_tracker == 0)
  {
  SENSOR_READ();                                             // white line on black surface ---> black e 0 and white e 1
  if((!ir[0] && !ir[8]) && (ir[3] || ir[4] || ir[5]))                                       // pichoner duitai line er baire thakle
  {
    SENSOR_READ();                                          // 1 on white(line).. karon white e < threshold and 0 on black(surface)
    
    while((!ir[0] && !ir[8]) && (ir[3] || ir[4] || ir[5]))                                  // jotokkhon pichoner duitai line er baire thakbe
    {
      LINE_TRACK();
      SENSOR_READ();                                         // 1 on white.. karon white e < threshold
    }
  }
  else                                                       // pichoner jkono ekta line paile
  {
    MOTOR(100, 100);
    delay(2);
    MOTOR(-100, -100);
    delay(1);
    MOTOR(0, 0);
    delay(2);
    
    node = NODE_CHECK(); //l, r, L, R, T, P, d, b, Y, U
//    MOTOR(0,0);
//    delay(100);
    
    if(node == 'T' || node == 'P'){
      turn = 'L';
    }
    else if(node == 'R'){
      turn = 'S';
    }
    else if(node == 'E'){                                     //***************** Reaches the White Block and Ends the dry run !! *********************
      run_tracker++;                                          // Sets the value of run_tracker to 1
      digitalWrite(13, HIGH);
      MOTOR(0, 0);
      CALCULATE_THE_SHORTEST_PATH();
      delay(5000);
      for(int i=0; memory[i]; i++){
        Serial.print(memory[i]);
      }
      
      delay(5000);
      return;                                                 // Terminates the DRY_RUN Function
    }
    else{
      turn = node;
    }
    if(!(node == 'l' || node == 'r')){
      memory[index1++] = turn;
    }
    
    TURN(node, turn);
  }
  }
}

void ACTUAL_RUN(){                                         //---------   Main Function - 02  ---------------------
  SENSOR_READ();                                             // white line on black surface ---> black e 0 and white e 1
  digitalWrite(13, LOW);
  if(!ir[0] && !ir[8])                                       // pichoner duitai line er baire thakle
  {
    SENSOR_READ();                                          // 1 on white(line).. karon white e < threshold and 0 on black(surface)
    
    while(!ir[0] && !ir[8])                                  // jotokkhon pichoner duitai line er baire thakbe
    {
      LINE_TRACK();
      SENSOR_READ();                                         // 1 on white.. karon white e < threshold
    }
  }
  else{
    node = NODE_CHECK();
    if(node == 'E'){                                         //********** Completes the actual run **************
      digitalWrite(13, HIGH);
      MOTOR(0, 0);
      for(int i=0; i<24; i++){
        delay(5000);
      }
    }
    if(node == 'l' || node == 'r'){
      TURN(node, node);
    }
    else{
      turn = memory[index2++];
      TURN(node, turn);
    }
  } 
}

void SENSOR_READ() // black line on white surface
{
  Serial.println("ok");
  for(int i=1; i<8; i++)
  {
    ir[i-1] = adc.readADC(i)>threshold;
      Serial.print(adc.readADC(i));
      Serial.print(" ");
  }
  ir[7] = analogRead(A0)>threshold;
  ir[8] = analogRead(A1)>threshold;
    Serial.print(analogRead(A0));
    Serial.print(" ");
    Serial.println(analogRead(A1));
    
       
}

void SENSOR_PRINT()
{
  for(int i=0; i<9; i++)
  {
    Serial.print(ir[i]);
//    Serial.print('\t');
  }
  Serial.println(' ');
}

void LINE_TRACK(){
  sum = ir[1] + ir[2] + ir[3] + ir[4] + ir[5] + ir[6] + ir[7];
  if(sum > 3){
    return;
  }
  error = (ir[1] * -45 + ir[2] * -35 + ir[3] * -10 + ir[4] * 0 + ir[5] *10 + ir[6] * 35 + ir[7] * 45);
  total_error = kp * error + kd * (error - previous_error);
//  Serial.println(total_error);
  left_speed = left_base_speed + total_error;
  right_speed = right_base_speed - total_error;

  if(left_speed > 255){
    left_speed = 255;
  }
  if (right_speed > 255){
    right_speed = 255;
  }

  MOTOR(left_speed, right_speed);
//  Serial.print(left_speed);
//  Serial.print("\t");
//  Serial.println(right_speed);
  previous_error = error;
}

char NODE_CHECK(){                          //l, r, L, R, T, P, d, b, Y, U
  char ch;
  if(ir[0] && !ir[8]){                                                                      // l or L
    if(ir[4] && !ir[7]){
      ch = 'L';
    }
    else{
      if(ir[7]){
        ch = 'T';
      }
      else{
        ch = 'l';
      }
    }
  }
  else if(!ir[0] && ir[8]){                                                                // r or R
    if(ir[4] && !ir[1]){
      ch = 'R';
    }
    else{
      if(ir[1]){
        ch = 'T';
      }
      else{
        ch = 'r';
      }
    }
  }
  else if(ir[0] && ir[8]){                                                                 // T or P or End Full White block (E)nd
//    if(ir[3] || ir[4] || ir[5]){
//      ch = 'P';
//    }
     if(!ir[4] && !ir[6]){                                   // ir[6] baad
      ch = 'T';
    }
    //***************** Special_Case(White_Block) *************************
    else if(ir[1] && ir[2] && ir[3] && ir[4] && ir[5] && ir[6] && ir[7]){                  // White Block
      ch = 'E';
    }
  }
//  Serial.print("YES");
    if(!(ir[0] || ir[1] || ir[2] || ir[3] || ir[4] || ir[5] || ir[6] || ir[7] || ir[8])){
    ch = 'U';
  }
  Serial.println(ch);
  SENSOR_READ();
  SENSOR_PRINT();
  MOTOR(0, 0);
  delay(20);
  MOTOR(-left_speed,-right_speed);
  delay(15);
  MOTOR(0, 0);
  delay(100);
  return ch;
}


void TURN(char node, char turn)                                    // node = l, r, L, R, P, T, E     &&     turn = l, r, L, R, S
{
  SENSOR_READ();
  // left ----------------------------------------------------------------------------------------------------- case - 01
  if(node == 'l'){
    while(!ir[4]){
      MOTOR(-100, 100);
      SENSOR_READ();
    }
    MOTOR(100, -100);
    delay(2);
    MOTOR(0, 0);
    delay(2);
//    MOTOR(100, -100);          // reverse torque
//    MOTOR(0, 0);
    MOTOR(100,100);
    delay(100);
    LINE_TRACK();
  }

  // right ---------------------------------------------------------------------------------------------------- case - 02
  if(node == 'r'){
    SENSOR_READ();
    while(!ir[4]){
      MOTOR(100, -100);
      SENSOR_READ();
    }
//    MOTOR(-100, right_base_speed);           // reverse torque
//    MOTOR(0, 0);
    MOTOR(-100, 100);
    delay(2);
    MOTOR(0, 0);
    MOTOR(100, 100);
    delay(100);
    LINE_TRACK();
  }

  // Left_T__left --------------------------------------------------------------------------------------------- case - 03
  if(node == 'L' && turn == 'L'){
    SENSOR_READ();
    while(ir[4]){
      MOTOR(-100, 100);
      SENSOR_READ();
    }
    while(!ir[4]){
      MOTOR(-100, 100);
      SENSOR_READ();
    }
//    MOTOR(0, 0);
    MOTOR(100, -100);
    delay(2);
    MOTOR(0, 0);
//    delay(2);
    MOTOR(100, 100);
    delay(300);
//    Serial.println("Yes");
    LINE_TRACK();
  }
  

  // Left_T__straight------------------------------------------------------------------------------------------ case - 04
  if(node == 'L' && turn == 'S'){
    SENSOR_READ();
    while(ir[0]){
      MOTOR(100, 100);
      SENSOR_READ();
    }
    MOTOR(100, 100);
    delay(100);
    LINE_TRACK();
    
  }

  // Right_T__right ------------------------------------------------------------------------------------------- case - 05
  if(node == 'R' && turn == 'R'){
    SENSOR_READ();
    while(ir[4]){
      MOTOR(100, -100);
      SENSOR_READ();
    }
    while(!ir[4]){
      MOTOR(100, -100);
      SENSOR_READ();
    }
    MOTOR(-100, 100);
    delay(2);
    MOTOR(0, 0);
    MOTOR(100, 100);
    delay(100);
    LINE_TRACK();
  }

  // Right_T__straight ---------------------------------------------------------------------------------------- case - 06
  if(node == 'R' && turn == 'S'){
    SENSOR_READ();
    while(ir[8]){
      MOTOR(100, 100);
      SENSOR_READ();
    }
    MOTOR(100, 100);
    delay(100);
    LINE_TRACK();
  }
  // T__left -------------------------------------------------------------------------------------------------- case - 07
  if(node == 'T' && turn == 'L'){
    SENSOR_READ();
    while(!ir[4]){
      MOTOR(-100, 100);
      SENSOR_READ();
    }
    MOTOR(100, -100);
    delay(2);
    MOTOR(0, 0);
    MOTOR(100, 100);
    delay(100);
    LINE_TRACK();
  }

  // T__right ------------------------------------------------------------------------------------------------- case - 08
  if(node == 'T' && turn == 'R'){
    SENSOR_READ();
    while(!ir[4]){
      MOTOR(100, -100);
      SENSOR_READ();
    }
    MOTOR(100, 100);
    delay(100);
    LINE_TRACK();
  }

  if(node == 'U'){
    SENSOR_READ();
    while(!ir[4]){
      MOTOR(-100, 100);
      SENSOR_READ();
    }
    MOTOR(100, 100);
    delay(100);
    LINE_TRACK();
  }
}


void MOTOR(int left_speed, int right_speed){
  if(left_speed >= 0){
    digitalWrite(L1, HIGH);
    digitalWrite(L2, LOW);
    analogWrite(L3, left_speed);
  }
  else{
    digitalWrite(L1, LOW);
    digitalWrite(L2, HIGH);
    analogWrite(L3,abs(left_speed));
  }
  if(right_speed >= 0){
    digitalWrite(R1, HIGH);
    digitalWrite(R2, LOW);
    analogWrite(R3, right_speed);
  }
  else{
    digitalWrite(R1, LOW);
    digitalWrite(R2, HIGH);
    analogWrite(R3, abs(right_speed));
  }
}


void CALCULATE_THE_SHORTEST_PATH()
{
  int i, j, k;
  //char memory[] = "SLULLULSULSLLUSSLLLUL";
  for(i=0; memory[i]; i++){
    if(memory[i+1] == 'U'){
      if(memory[i] == 'S' && memory[i+2] == 'S'){
        memory[i] = 'U';
                  }
      else if(memory[i] == 'S' && memory[i+2] == 'R'){
        memory[i] = 'L';
      }
      else if(memory[i] == 'S' && memory[i+2] == 'L'){
        memory[i] = 'R';
      }
      else if(memory[i] == 'R' && memory[i+2] == 'R'){
        memory[i] = 'S';
      }
      else if(memory[i] == 'R' && memory[i+2] == 'S'){
        memory[i] = 'L';
      }
      else if(memory[i] == 'R' && memory[i+2] == 'L'){
        memory[i] = 'U';
      }
      else if(memory[i] == 'L' && memory[i+2] == 'L'){
        memory[i] = 'S';
      }
      else if(memory[i] == 'L' && memory[i+2] == 'S'){
        memory[i] = 'R';
      }
      else if(memory[i] == 'L' && memory[i+2] == 'R'){
        memory[i] = 'U';
      }
      for(j=i+1, k=i+3; memory[k]; j++, k++){
        memory[j] = memory[k];
      }
      memory[j] = '\0';
      //printf("\n\n%s", memory);
      //break;
    }
  }
}
