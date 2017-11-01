
#include <SoftwareSerial.h>  

//grab six analog signals of the hand from data glove, (thumb flexion/extension [F/E] & Add/Adbuction [Ad/Ab])
//pins layout
const int D0 = A0; // Thumb Ad/Ab 
const int D1 = A1; // Thumb F/E
const int D2 = A2; // Index
const int D3 = A3; // Middle
const int D4 = A4; // Ring finger 
const int D5 = A5; // Pinky
//variables 
int D0_pos = 0;           
int D1_pos = 0;
int D2_pos = 0;
int D3_pos = 0;
int D4_pos = 0;
int D5_pos = 0;



//setup blue tooth serial connection
const int BLUETOOTH_TX = 2;  // TX-O pin of RN-42, Arduino D2
const int BLUETOOTH_RX = 4;  // RX-I pin of RN-42, Arduino D4
SoftwareSerial bluetooth(BLUETOOTH_TX, BLUETOOTH_RX);

// Send buffer
byte const FLAG = 0x2E; //"ascii period": Flag used to identify start of stream
byte sendBuffer[16];                 // Sendbuffer

void setup()
{
  Serial.begin(38400);  // Begin the serial monitor at 9600bps

  bluetooth.begin(38400);  // The Bluetooth Mate defaults to 115200bps
}

void loop()
{
  //grab current hand positions
  D0_pos = analogRead(D0);           
  D1_pos = analogRead(D1);
  D2_pos = analogRead(D2);
  D3_pos = analogRead(D3);
  D4_pos = analogRead(D4);
  D5_pos = analogRead(D5);

  sendInfo();
  delay(50);
//   Serial.println("^^^^^^^^^^^^^^^^^^^^^^^^");
        Serial.print(String(D0_pos));
        Serial.print(", ");
        Serial.print(String(D1_pos));
        Serial.print(", ");
        Serial.print(String(D2_pos));
        Serial.print(", ");
        Serial.print(String(D3_pos));
        Serial.print(", ");
        Serial.print(String(D4_pos));
        Serial.print(", ");
        Serial.println(String(D5_pos));
}

// Send information to the computer via Bluetooth
void sendInfo()
{
  // Fill byte buffer with sensor data
  // The first two are flags to know when to start looking at the streaming data
  // Each sensor data stores the upper byte and lower byte, since the byte buffer only takes one byte at a time
  // This is more light weight than sending strings, int, double etc since the information from the sensor 
  // Will most likely not exceed 2^16 (two bytes) as the ADC maps values to 10bit -> 2^10.

  // Load flags, 4 periods "...."
  sendBuffer[0]= FLAG >> 8;
  sendBuffer[1]= FLAG & 0xFF;
  sendBuffer[2]= FLAG >> 8;
  sendBuffer[3]= FLAG & 0xFF;
//  sendBuffer[4]= FLAG >> 8;
//  sendBuffer[5]= FLAG & 0xFF;
//  sendBuffer[6]= FLAG >> 8;
//  sendBuffer[7]= FLAG & 0xFF;
  
  int i = 4; //8 buffer_offset
  // Load data glove values
  sendBuffer[i] = D0_pos >>8;     // thumbAx UB: UpperByte 
  sendBuffer[i+1] = D0_pos & 0xFF;  // thumbAx LB: LowerByte 
  sendBuffer[i+2] = D1_pos >> 8;    // thumbFE UB
  sendBuffer[i+3] = D1_pos & 0xFF;  // thumbFE LB
  sendBuffer[i+4] = D2_pos >> 8;    // indexF UB
  sendBuffer[i+5] = D2_pos & 0xFF;  // indexF LB
  sendBuffer[i+6]= D3_pos >> 8;    // middleF UB
  sendBuffer[i+7]= D3_pos & 0xFF;  // middleF LB
  sendBuffer[i+8]= D4_pos >> 8;    // ringF UB
  sendBuffer[i+9]= D4_pos & 0xFF;  // ringF LB
  sendBuffer[i+10]= D5_pos >> 8;    // pinkyF UB
  sendBuffer[i+11]= D5_pos & 0xFF;  // pinkyF LB

  // checksum
//  int Check_1 = D0_pos * 2 + D1_pos * -3 + D2_pos * 3;
//  int Check_2 = D3_pos * -3 + D4_pos * 3 + D5_pos * -7;
//  
//  i = 12;
//  sendBuffer[i]= Check_1 >> 8;
//  sendBuffer[i+1]= Check_1 & 0xFF;
//  sendBuffer[i+2]= Check_2 >> 8;
//  sendBuffer[i+3]= Check_2 & 0xFF;

  // Send data
  bluetooth.write(sendBuffer, sizeof(sendBuffer));

}

