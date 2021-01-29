#include <SPI.h>
#include <mcp_can.h>

const int spiCSPin = 10;

/*********/
float c_motor_pin = A2;      //
int c_motor = 0;     // for MOTOR CURRENT

/*********/

/*********/
float c_bus_pin = A1;      //
int c_bus = 0;     // for BUS CURRENT
     //
/*********/

/*********/
float velocity_pin = A0;      //
int velocity = 0;     // for VELOCITY
      //
/*********/

unsigned char canMsg1[8];
unsigned char canMsg2[8];
unsigned char canMsg3[8];

/*
int yuzdelik = 0;

*/
MCP_CAN CAN(spiCSPin);

void setup() {
  Serial.begin(115200);

  pinMode(c_motor_pin,INPUT);
  pinMode(c_bus_pin,INPUT);
  pinMode(velocity,INPUT);

  while(CAN_OK != CAN.begin(CAN_500KBPS))
  {
    Serial.println("CAN BUS init Failed");
    delay(100);
  }
  Serial.println("CAN BUS Shield Init OK!");

  CAN.sendMsgBuf(0x500,0,8,0x7F1);
  
  }

void loop() {
   
    //Read the value of the pot

  

    
   velocity = analogRead(velocity_pin)/1023;    // pot1 = velocity
   Serial.println(velocity,3);
   canMsg1[4] = (velocity >> 8) & 0xFF;
   canMsg1[5] = (velocity) & 0xFF ;
  

   c_motor = analogRead(c_motor_pin); // pot2 = motor current
   canMsg2[4] = (c_motor << 8) & 0xFF;
   canMsg2[5] = (c_motor) & 0xFF ;                                                                                                              

   c_bus = analogRead(c_bus_pin); // pot3 = bus current
   canMsg3[4] = (c_bus << 8) & 0xFF;
   canMsg3[5] = (c_bus)& 0xFF ;


   if(velocity >0)
   {
    CAN.sendMsgBuf(0x501,0,8,canMsg1[5]);

   // Serial.println(velocity);

   }
   else if(c_motor>0)
   {
    CAN.sendMsgBuf(0x501,0,8,canMsg2[5]);
        //Serial.println(c_motor);

   }
   else if(c_bus>0)
   {
    CAN.sendMsgBuf(0x502,0,8,canMsg3[5]);
        //Serial.println(c_bus);
   
   
   }
  
   delay(200);
}
