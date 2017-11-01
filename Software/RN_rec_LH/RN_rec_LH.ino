#include <Boards.h>
#include <Firmata.h>

//This script incorporates bluetooth connection to Handi Hand via data glove with
//LED indicators for filter stabilization and GUI output of sensors to computer.
//25/05/2017

//#include <SoftwareSerial.h>
#include <Servo.h>

//servos
Servo D0_servo;  // create servo objects to control servos
Servo D1_servo;
Servo D2_servo;
Servo D3_servo;
Servo D4_servo;
Servo D5_servo;

//LEDs
int yel_led_1 = 40;
int yel_led_2 = 42;
int yel_led_3 = 44;
int yel_led_4 = 46;
int grn_led = 48;

//Sensor Values
int D0_Angle = 0;
int D1P_Angle = 0;
int D1D_Angle = 0;
int D2P_Angle = 0;
int D2I_Angle = 0;
int D3P_Angle = 0;
int D3I_Angle = 0;
int D4P_Angle = 0;
int D5P_Angle = 0;
  
int D4I_Angle = -10; //dummy variable for spacing in GUI
int D5I_Angle = -10; //dummy variable for spacing in GUI
  
int D1_Voltage = 0;
int D2_Voltage = 0;
int D3_Voltage = 0;
int D4_Voltage = 0;
int D5_Voltage = 0;

//Potentiometer Max and Min readouts for mapping to degrees of flexion:
int D0_max = 380;
int D0_min = 640;
int D1P_max = 1023-301;
int D1P_min = 1023-487;
int D1D_max = 1023-336;
int D1D_min = 1023-515;
int D2P_max = 1023-289;
int D2P_min = 1023-512;
int D2I_max = 1023-264;
int D2I_min = 1023-518;
int D3P_max = 1023-759;
int D3P_min = 1023-530;
int D3I_max = 1023-270;
int D3I_min = 1023-500;
int D4P_max = 1023-753;
int D4P_min = 1023-500;
int D5P_max = 1023-755;
int D5P_min = 1023-568;
  
//setup bluetooth serial connection
int const BLUETOOTH_SETTLE_TIME = 100;

// Recieve buffer
byte const FLAG = 0x2E; //"Ascii period": Flag used to identify start of stream
byte recieveBuffer[12];               // Receive buffer

////variables
int D0_pos = -1;
int D1_pos = -1;
int D2_pos = -1;
int D3_pos = -1;
int D4_pos = -1;
int D5_pos = -1;

int D0_servo_pos = 0;
int D1_servo_pos = 0;
int D2_servo_pos = 0;
int D3_servo_pos = 0;
int D4_servo_pos = 0;
int D5_servo_pos = 0;

int currentAndPastInput[7][6] = {{0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}}; //4th order - 6th order : length 4-6
//currentAndPastInput = [D0_pos,D1_pos,D2_pos,D3_pos,D4_pos,D5_pos] => {{current},{n-1} -> {n-6}}
float currentAndPastOutput[7][6] = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}; //4th order - 6th order : length 4-6
//currentAndPastOutput = [D0_pos,D1_pos,D2_pos,D3_pos,D4_pos,D5_pos] => {{current},(n-1) -> {n-6}}
//samples to reach steady state for step impluse => matlab
int const SAMPLES_FOR_SS_2thOrder = BLUETOOTH_SETTLE_TIME + 100; // 92 exactly

int programCounter = 0;
int numberOfSamplesRecieved = 0;

void setup()
{
  //Set up serial And BT
  Serial.begin(38400);  // Begin the serial monitor at 38400bps
  Serial1.begin(38400);  // Begin Bluetooth
  gBegin(38400);// Set up GUI for output

  // set servo objects to appropriate pins
  D0_servo.attach(3);
  D1_servo.attach(5);
  D2_servo.attach(6);
  D3_servo.attach(9);
  D4_servo.attach(10);
  D5_servo.attach(11);
  
  //Initialize LEDs
  pinMode(yel_led_1, OUTPUT);
  pinMode(yel_led_2, OUTPUT);
  pinMode(yel_led_3, OUTPUT);
  pinMode(yel_led_4, OUTPUT);
  pinMode(grn_led, OUTPUT);

}

void loop()
{
  //**** Main update call for the guino:
  guino_update();
  
  // Bluetooth receive
  if (Serial1.available() >= sizeof(recieveBuffer)) // If there's anything to read
  {
    if ( Serial1.read() * 256 + Serial1.read() == FLAG) {

      if (Serial1.read() * 256 + Serial1.read() == FLAG) {

        Serial1.readBytes(recieveBuffer, sizeof(recieveBuffer));
        numberOfSamplesRecieved =  numberOfSamplesRecieved + 1;
        //variables
        D0_pos = (int)(recieveBuffer[0] * 256 + recieveBuffer[1]);
        D1_pos = (int)(recieveBuffer[2] * 256 + recieveBuffer[3]);
        D2_pos = (int)(recieveBuffer[4] * 256 + recieveBuffer[5]);
        D3_pos = (int)(recieveBuffer[6] * 256 + recieveBuffer[7]);
        D4_pos = (int)(recieveBuffer[8] * 256 + recieveBuffer[9]);
        D5_pos = (int)(recieveBuffer[10] * 256 + recieveBuffer[11]);

        D0_pos = map(D0_pos , 500, 650, 0, 180); //scale it to use it with the servo (value between 0 and 180)
        D1_pos = map(D1_pos , 415, 850, 180, 0);
        D2_pos = map(D2_pos , 485, 950, 0, 180);
        D3_pos = map(D3_pos , 340, 800, 0, 180);
        D4_pos = map(D4_pos , 380, 840, 0, 180);
        D5_pos = map(D5_pos , 340, 760, 180, 0);

        D0_pos = min(max(D0_pos, 0), 180); //scale it to use it with the servo (value between 0 and 180)
        D1_pos = min(max(D1_pos, 0), 180);
        D2_pos = min(max(D2_pos, 0), 180);
        D3_pos = min(max(D3_pos, 0), 180);
        D4_pos = min(max(D4_pos, 0), 180);
        D5_pos = min(max(D5_pos, 0), 180);


        // shift old output to n-1 position so filters do not overwrite it with the new output position
        //update inputs array load new input to current position and push everything else down by one.
        shiftInputAndOutPutArrays();

        if (numberOfSamplesRecieved > BLUETOOTH_SETTLE_TIME) { //inital bluetooth transmission is not always stable, wait for a few samples before engaging filter
          //filter current output 2th order
          secondOrderButterLPF_cutoff_20Hz_difference_equation();



          if (numberOfSamplesRecieved > SAMPLES_FOR_SS_2thOrder) //filter reached steady state
          {
//            Serial.println(min(max(currentAndPastOutput[0][2], 0), 180));;

            //recall
            //currentAndPastOutput = [D0_pos,D1_pos,D2_pos,D3_pos,D4_pos,D5_pos] => {{current},(n-1) -> {n-6}}
             D0_servo_pos = (min(max(currentAndPastOutput[0][0], 0), 180)); //sets the servo position according to the scaled value
             D1_servo_pos = (min(max(currentAndPastOutput[0][1], 0), 180));
             D2_servo_pos = (min(max(currentAndPastOutput[0][2], 0), 180));
             D3_servo_pos = (min(max(currentAndPastOutput[0][3], 0), 180));
             D4_servo_pos = (min(max(currentAndPastOutput[0][4], 0), 180));
             D5_servo_pos = (min(max(currentAndPastOutput[0][5], 0), 180));
             
             D0_servo.write(D0_servo_pos); //sets the servo position according to the scaled value
             D1_servo.write(D1_servo_pos);
             D2_servo.write(D2_servo_pos);
             D3_servo.write(D3_servo_pos);
             D4_servo.write(D4_servo_pos);
             D5_servo.write(D5_servo_pos);
                         
             digitalWrite(yel_led_1,LOW);
             digitalWrite(yel_led_2,LOW);
             digitalWrite(yel_led_3,LOW);
             digitalWrite(yel_led_4,LOW);
             digitalWrite(grn_led,HIGH);
          } 
        else
        {
//          Serial.println(numberOfSamplesRecieved);
          if (numberOfSamplesRecieved <= BLUETOOTH_SETTLE_TIME + 33)
          {
          digitalWrite(yel_led_1,LOW);
          digitalWrite(yel_led_2,HIGH);
          digitalWrite(yel_led_3,LOW);
          digitalWrite(yel_led_4,LOW);
          digitalWrite(grn_led,LOW);
          }
          else
          {
          if (numberOfSamplesRecieved < BLUETOOTH_SETTLE_TIME + 66)
          {
          digitalWrite(yel_led_1,LOW);
          digitalWrite(yel_led_2,LOW);
          digitalWrite(yel_led_3,HIGH);
          digitalWrite(yel_led_4,LOW);
          digitalWrite(grn_led,LOW);
          }
          else{          
          digitalWrite(yel_led_1,LOW);
          digitalWrite(yel_led_2,LOW);
          digitalWrite(yel_led_3,LOW);
          digitalWrite(yel_led_4,HIGH);
          digitalWrite(grn_led,LOW);          
          }
          }
        }
        }
        else
        {
          digitalWrite(yel_led_1,HIGH);
          digitalWrite(yel_led_2,LOW);
          digitalWrite(yel_led_3,LOW);
          digitalWrite(yel_led_4,LOW);
          digitalWrite(grn_led,LOW);           
        }
      }

    }

  }
  programCounter = programCounter + 1;
  
  // Read in the values from each of the potentiometers and FSRs:
  int D0_Voltage = analogRead(A2);
  int D1P_Voltage = analogRead(A3);
  int D1D_Voltage = analogRead(A4);
  int D2P_Voltage = analogRead(A5);
  int D2I_Voltage = analogRead(A6);
  int D3P_Voltage = analogRead(A7);
  int D3I_Voltage = analogRead(A8);
  int D4P_Voltage = analogRead(A9);
  int D5P_Voltage = analogRead(A10);
  D1_Voltage = analogRead(A11);
  D2_Voltage = analogRead(A12);
  D3_Voltage = analogRead(A13);
  D4_Voltage = analogRead(A14);
  D5_Voltage = analogRead(A15);
  
//  Serial.print(String("hello"));
//  Serial.print(String(", "));
//   Serial.print(D1P_Voltage);
//  Serial.print(String(", "));
//   Serial.print(String(D1D_Voltage));
//  Serial.print(String(", "));
//   Serial.print(String(D2P_Voltage));
//  Serial.print(String(", "));
//   Serial.print(String(D2I_Voltage));
//  Serial.print(String(", "));
//   Serial.print(String(D3P_Voltage));
//  Serial.print(String(", "));
//   Serial.print(String(D3I_Voltage));
//  Serial.print(String(", "));
//   Serial.print(String(D4P_Voltage));
//  Serial.print(String(", "));
//   Serial.println(String(D5P_Voltage));

  
    // Map the Potentiometer voltages to % flexion:
  D0_Angle = map(D0_Voltage,D0_min,D0_max,0,100);
  D1P_Angle = map(D1P_Voltage,D1P_min,D1P_max,0,100);
  D1D_Angle = map(D1D_Voltage,D1D_min,D1D_max,0,100);
  D2P_Angle = map(D2P_Voltage,D2P_min,D2P_max,0,100);
  D2I_Angle = map(D2I_Voltage,D2I_min,D2I_max,0,100);
  D3P_Angle = map(D3P_Voltage,D3P_min,D3P_max,0,100);
  D3I_Angle = map(D3I_Voltage,D3I_min,D3I_max,0,100);
  D4P_Angle = map(D4P_Voltage,D4P_min,D4P_max,0,100);
  D5P_Angle = map(D5P_Voltage,D5P_min,D5P_max,0,100);
  
  // Send the values to the GUI:
  gUpdateValue(&D0_Angle);
  gUpdateValue(&D1P_Angle);
  gUpdateValue(&D1D_Angle);
  gUpdateValue(&D2P_Angle);
  gUpdateValue(&D2I_Angle);
  gUpdateValue(&D3P_Angle);
  gUpdateValue(&D3I_Angle);
  gUpdateValue(&D4P_Angle);
  gUpdateValue(&D5P_Angle);
  gUpdateValue(&D1_Voltage);
  gUpdateValue(&D2_Voltage);
  gUpdateValue(&D3_Voltage);
  gUpdateValue(&D4_Voltage);
  gUpdateValue(&D5_Voltage);
  
}

void shiftInputAndOutPutArrays()
{
  //shift both arrays downward by one row
  for (int i = 6; i >= 0 ; i--) // process all seven rows
  { 
    for (int j = 0; j <= 5; j++) { //for all the columns
      if (i > 0) // for everything by the first row
      { //recall [D0_pos,D1_pos,D2_pos,D3_pos,D4_pos,D5_pos]
        currentAndPastOutput[i][j] = currentAndPastOutput[i - 1][j]; //assign the current row the values of the previous row
        currentAndPastInput[i][j] = currentAndPastInput[i - 1][j];
      }
      else
      {
        //set current output as zeros (will be replaced will filtered afterwards)
        currentAndPastOutput[0][0] = 0;
        currentAndPastOutput[0][1] = 0;
        currentAndPastOutput[0][2] = 0;
        currentAndPastOutput[0][3] = 0;
        currentAndPastOutput[0][4] = 0;
        currentAndPastOutput[0][5] = 0;
        //set current inputs as the mapped inputs
        currentAndPastInput[0][0] = D0_pos;
        currentAndPastInput[0][1] = D1_pos;
        currentAndPastInput[0][2] = D2_pos;
        currentAndPastInput[0][3] = D3_pos;
        currentAndPastInput[0][4] = D4_pos;
        currentAndPastInput[0][5] = D5_pos;
      }
    }
  }
}

void firstOrderButterLPF_cutoff_20Hz_difference_equation()
{ //needs length(currentAndPastInput) = 5, length(pAstOutPut) = 5
  //cutoff 160
  const float A[] = {1,-0.725343};
  const float B[] = {0.137328,0.137328};

  float K =1.25;
  for (int i = 0; i <= 5 ; i++) { //6 - columns
    //updating current outputs
    currentAndPastOutput[0][i] = (-A[1] * (float)currentAndPastOutput[1][i]+ K*(B[0] * (float)currentAndPastInput[0][i] + B[1] * (float)currentAndPastInput[1][i]));
  }
}
void secondOrderButterLPF_cutoff_20Hz_difference_equation()
{ 
  //160 Hz
  const float A[] = {1,-1.558868,0.639930};
  const float B[] = {0.020265,0.040531,0.020265};
float K = 1.25;
  for (int i = 0; i <= 5 ; i++) { //6 - colums
    currentAndPastOutput[0][i] = (-A[1] * (float)currentAndPastOutput[1][i] - A[2] * (float)currentAndPastOutput[2][i] + K*( B[0] * (float)currentAndPastInput[0][i] + B[1] * (float)currentAndPastInput[1][i] + B[2] * (float)currentAndPastInput[2][i]));  
  }

}

//Set up the guino interface:
void gInit()
{gAddLabel("THUMB ROTATION",1);
  gAddSpacer(1);
  gAddMovingGraph("DO", 0, 100, &D0_Angle, 10);
//  gAddSlider(0,180,"Servo Pos:",&D0_servo_pos);
 
  gAddColumn();
  gAddLabel("THUMB FLEXION",1);
  gAddSpacer(1);
  gAddMovingGraph("Proximal Pot - D1",0,100,&D1P_Angle,10);
  gAddMovingGraph("Distal Pot - D1",0,100,&D1D_Angle,10);
  gAddMovingGraph("FSR - D1",0,1000,&D1_Voltage,10);
//  gAddSlider(0,180,"Servo Pos:",&D1_servo_pos);  
  
  gAddColumn();
  gAddLabel("INDEX",1);
  gAddSpacer(1);
  gAddMovingGraph("Proximal Pot - D2",0,100,&D2P_Angle,10);
  gAddMovingGraph("Intermediate Pot - D2",0,100,&D2I_Angle,10);
  gAddMovingGraph("FSR - D2",0,1000,&D2_Voltage,10);
//  gAddSlider(0,180,"Servo Pos:",&D2_servo_pos);
 
  gAddColumn();
  gAddLabel("MIDDLE",1);
  gAddSpacer(1);
  gAddMovingGraph("Proximal Pot - D3",0,100,&D3P_Angle,10);
  gAddMovingGraph("Intermediate Pot - D3",0,100,&D3I_Angle,10);
  gAddMovingGraph("FSR - D3",0,1000,&D3_Voltage,10); 
//  gAddSlider(0,180,"Servo Pos:",D3_servo_pos);
  
  gAddColumn();
  gAddLabel("RING",1);
  gAddSpacer(1);
  gAddMovingGraph("Proximal Pot - D4",0,100,&D4P_Angle,10);
  gAddMovingGraph("Intermediate Pot - D4",0,100,&D4I_Angle,10);
  gAddMovingGraph("FSR - D4",0,1000,&D4_Voltage,10);
//  gAddSlider(0,180,"Servo Pos:",D4_servo_pos);
  
  gAddColumn();
  gAddLabel("PINKY",1);
  gAddSpacer(1);
  gAddMovingGraph("Proximal Pot - D5",0,100,&D5P_Angle,10);
  gAddMovingGraph("Intermediate Pot - D5",0,100,&D5I_Angle,10);
  gAddMovingGraph("FSR - D5",0,1000,&D5_Voltage,10);
//  gAddSlider(0,180,"Servo Pos:",D5_servo_pos);
  
  gSetColor(274,173,5);
  
}

// Method called everytime a button has been pressed in the interface.
void gButtonPressed(int id)
{
 
}

void gItemUpdated(int id)
{

}
 

