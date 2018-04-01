/*
 Robotic arm control and movement using hand co-ordinates

 This is the arduino code required to move the robotic arm according to hand co-ordinates received serially using HC-06.

     AUTHOR  : Nevil Pooniwala
     EMAIL   : nevilpooniwala1997@gmail.com
*/

#include "SoftwareSerial.h"
#include <Servo.h>			    // Include the Servo library 
SoftwareSerial serial_connection(10, 11);   //Create a serial connection with TX and RX on these pins of bluetooth
#define BUFFER_SIZE 64			    //This will prevent buffer overruns.
char inData[BUFFER_SIZE];		    //This is a character buffer where the data sent by the python script will go.
char inChar=-1;			            //Initialie the first character as nothing
int count=0;				    //This is the number of lines sent in from the python script
int i=0;				    //Arduinos are not the most capable chips in the world so I just create the looping variable once
int pre_x=0,pre_y=0,pre_z=0;

// Declare the Servo pin 
int x_pin = 3; 
int y_pin = 5; 
int z_pin = 6; 
int grip_pin = 9; 

// Create a servo object 
Servo ser_x;
Servo ser_y;
Servo ser_z;
Servo ser_grip;

void setup()
{
  Serial.begin(9600);				//Initialize communications to the serial monitor in the Arduino IDE
  serial_connection.begin(9600);		//Initialize communications with the bluetooth module
  serial_connection.println("Ready!!!");	//Send something to just start comms. This will never be seen.
  Serial.println("Started");			//Tell the serial monitor that the sketch has started.
  ser_x.attach(x_pin);
  ser_y.attach(y_pin);
  ser_z.attach(z_pin);
  ser_grip.attach(grip_pin);
  pinMode(8, OUTPUT);
}

void loop()
{
    int x, y, z, grip;
    int iy=0;
    int xyz[4];

  //This will prevent bufferoverrun errors
  byte byte_count=serial_connection.available();	//This gets the number of bytes that were sent by the python script
  if(byte_count)			//If there are any bytes then deal with them
  {
    int first_bytes=byte_count;		//initialize the number of bytes that we might handle. 
    for(i=0;i<first_bytes;i++)		//Handle the number of incoming bytes
    {
      inChar=serial_connection.read();	//Read one byte
      inData[i]=inChar;			//Put it into a character string(array)
      if (inData[i] == ';')
      	break;
    }  
    inData[i]='\0';			//This ends the character array with a null character. This signals the end of a string

  int n = sscanf(inData, "x: %d y: %d z: %d grip: %d", &x, &y, &z, &grip);
  if(x>0 && x<180 && y<180 && y>70 && z>50 && z<115 && (grip==70 || grip==100 ) && ( abs(pre_x-x) <20 )&& ( abs(pre_y-y) <20 )&& ( abs(pre_z-z) <20 ) ){
 	Serial.print("x: ");
	Serial.print(x);
	Serial.print(" y: ");
	Serial.print(y);
	Serial.print(" z: ");
	Serial.print(z);
	Serial.print(" grip: ");
	Serial.println(grip);
	ser_x.write(x); 
	ser_y.write(y); 
	ser_z.write(z); 
	ser_grip.write(grip);

	if ( grip == 100) 
	  digitalWrite(8, HIGH);
	if ( grip == 70)
	  digitalWrite(8, LOW);
  }

  pre_x=x;
  pre_y=y;
  pre_z=z;
  
  count++;
  delay(100);	//Pause for a moment 
  } }
