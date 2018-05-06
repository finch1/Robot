//Include files
#include <Wire.h>
#include <LSM303.h>


//Define compass
LSM303 compass;

//Defines pins numbers
//UltraSonic
const int trigPin = 10;
const int echoPin = 11;
//Motor Driver
const int pha     = 6;
const int ena     = 7;
const int phb     = 9;
const int enb     = 8;
//IR Sensors
const int analog_IR_1 = A1;
const int analog_IR_2 = A2;
//GPIO
const int pushButton3 = 3; //switch
const int pushButton4 = 4; //buzzer

//Define Costants
const unsigned int speed_high = 180;
const unsigned int speed_low = 100;
const unsigned int turn_delay = 17;
unsigned int heading_coo = 160;
unsigned int right_angle = 0;

//Declare Vars
unsigned int  comp = 0;
unsigned int  us = 0;
unsigned int ira = 0;
unsigned int irb = 0;
int count;

//Subscription callback function - motor control
void messageCb( unsigned int val){


 if(val == 0x01)  { //forward
    analogWrite(pha, 130);
    digitalWrite(ena, HIGH);
    analogWrite(phb, 130);
    digitalWrite(enb, HIGH);
 }else if(val == 0x02) { //reverse
    analogWrite(pha, 115);
    digitalWrite(ena, LOW);
    analogWrite(phb, 135);
    digitalWrite(enb, LOW);  
 }else if(val == 0x10) { //right
    analogWrite(pha, 128);
    digitalWrite(ena, HIGH);
    analogWrite(phb, 128);
    digitalWrite(enb, LOW);    
 }else if(val == 0x20) { //left
    analogWrite(pha, 128);
    digitalWrite(ena, LOW);
    analogWrite(phb, 128);
    digitalWrite(enb, HIGH); 
 }else if(val == 0x30) { //right reorient
    analogWrite(pha, 70);
    digitalWrite(ena, HIGH);
    analogWrite(phb, LOW);
    digitalWrite(enb, LOW);    
 }else if(val == 0x40) { //left reorient
    analogWrite(pha, LOW);
    digitalWrite(ena, LOW);
    analogWrite(phb, 70);
    digitalWrite(enb, HIGH); 
 }else if(val == 0xAA) { //brake
    analogWrite(pha, LOW);
    digitalWrite(ena, LOW);
    analogWrite(phb, LOW);
    digitalWrite(enb, LOW);
 }else if(val == 0x06)  { //straight
    analogWrite(pha, 145);
    digitalWrite(ena, HIGH);
    analogWrite(phb, 128);
    digitalWrite(enb, HIGH);
 }
}

//Function gets ultrasonic reading
unsigned int getRange_Ultrasound(){
  unsigned long duration = 0;
  unsigned int cm = 0;

  
  int i, j, a[20] = {0}, n = 20, b[20] = {0}, k = 0, c = 1, max = 0, mode;
  float Pi = 3.14159;
  for (i = 0; i < n; i++)
  {   // The sensor is triggered by a HIGH pulse of 10 or more microseconds.    // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
      digitalWrite(trigPin, LOW);
      delayMicroseconds(5);
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
     
      // Read the signal from the sensor: a HIGH pulse whose    // duration is the time (in microseconds) from the sending    // of the ping to the reception of its echo off of an object.
      pinMode(echoPin, INPUT);
      duration = pulseIn(echoPin, HIGH);
    
      // convert the time into a distance
      a[i] = (duration/2) / 29.1;
  }

  for (i = 0; i < n - 1; i++)
  {   mode = 0;
      for (j = i + 1; j < n; j++)
      {   if (a[i] == a[j]) {
              mode++;
          }
      }

      if ((mode > max) && (mode != 0)) {
          k = 0;

          max = mode;

          b[k] = a[i];

          k++;
      }

      else if (mode == max) {
          b[k] = a[i];
          k++;
      }

  }

  for (i = 0; i < n; i++)
  {
      if (a[i] == b[i]) 
          c++;
  }

  if (c == n){}
  else
  {
        return b[0];
  }
      
}


//Function gets IR reading
int getRange_Infrared(int pin_num){
  return (analogRead(pin_num)/2) / 29.1;
}

//Function gets Compass reading
unsigned int getRange_Compass(){

 int i, j, a[20] = {0}, n = 20, b[20] = {0}, k = 0, c = 1, max = 0, mode;
  float Pi = 3.14159;
  for (i = 0; i < n; i++)
  {   compass.read();
      // Calculate the angle of the vector y,x
      int heading = (int) (atan2(compass.m.y,compass.m.x) * 180) / Pi;

      if (heading < 0)
      {
        heading = 360 + heading;
      }
      a[i] = heading;
  }

  for (i = 0; i < n - 1; i++)
  {   mode = 0;
      for (j = i + 1; j < n; j++)
      {   if (a[i] == a[j]) {
              mode++;
          }
      }

      if ((mode > max) && (mode != 0)) {
          k = 0;

          max = mode;

          b[k] = a[i];

          k++;
      }

      else if (mode == max) {
          b[k] = a[i];
          k++;
      }

  }

  for (i = 0; i < n; i++)
  {
      if (a[i] == b[i]) 
          c++;
  }

  if (c == n){}
  else
  {
        return b[0];
  }
      
}

void obstacle(unsigned int direction, String s){
  Serial.println(s);
  messageCb(direction);
}

void stoptest(){
  while(1){
     obstacle(0xAA, "");
     Serial.printf("ira:[%d] irb:[%d] us:[%d]--", (getRange_Infrared(analog_IR_1)), (getRange_Infrared(analog_IR_2)), getRange_Ultrasound());  
  }
}

void setup()
{
  pinMode(trigPin, OUTPUT);   //Sets the trigPin as an Output
  pinMode(echoPin, INPUT);    //Sets the echoPin as an Input
  
  pinMode(pha, OUTPUT);       //Sets the phaseA as an Output
  pinMode(ena, OUTPUT);       //Sets the enableA as an Output
  pinMode(phb, OUTPUT);       //Sets the phaseB as an Output
  pinMode(enb, OUTPUT);       //Sets the enableB as an Output

  pinMode(pushButton3, INPUT); //input button start
  pinMode(pushButton4, OUTPUT); //buzzer

  digitalWrite(pushButton4, LOW); //buzzer off

  Wire.begin();     //Start the I2C communication
  compass.init();   //Configure compass board parameters
  compass.enableDefault();
  compass.m_min = (LSM303::vector<int16_t>){-32767, -32767, -32767};
  compass.m_max = (LSM303::vector<int16_t>){+32767, +32767, +32767};

  // initialize //Serial communication at 9600 bits per second:
  Serial.begin(9600);
}


void loop()
{
//  //Wait for direction button 
//  
//  if(!digitalRead(pushButton3)){
//    heading_coo = getRange_Compass();
//    //Serial.printf("Aquiring coordinates:[%d]\n", heading_coo);
//    digitalWrite(pushButton4, HIGH);
//  }else{
/*            
*pause to measure    
*/          
    obstacle(0xAA, "aquire sensor input");
    delay(200); //delay to stabilize motion

/*            
*Get the data from the sensors
*/
    comp = getRange_Compass();
    us = getRange_Ultrasound();
    ira = getRange_Infrared(analog_IR_1);
    irb = getRange_Infrared(analog_IR_2);
    Serial.printf("Comp:[%d] US:[%d] IRA:[%d] IRB:[%d]\n", comp, us, ira, irb);
    
/*           
*when obstacle detected, make a turn, then find wall
*/ 
    if(us < 15){ 
      //turn 90
      int temp = 0;
      temp = heading_coo + 90;
      if(temp > 360)
      {
        temp -= 360;
      }
      /*           
      *first make a turn
      */             
        obstacle(0x10, "go to wall, turn right");
        delay(300); //allow turn
        obstacle(0xAA, "check for wall, stop");
        delay(500); //allow pause to get sensors
        comp = getRange_Compass();
        us = getRange_Ultrasound();

      /*           
      *then move straight towards the wall
      */ 
      us = getRange_Ultrasound();            
      while(us > 25){
        if(comp > temp+8){
            while(comp > temp+3){ 
              //Serial.printf("Comp:[%d] RA:[%d]--", comp, temp);           
              obstacle(0x40, "reorient final left to last edge"); 
              delay(2); //allow turn
              obstacle(0xAA, "stop from final r_left to last edge");
              delay(4); //allow pause
              comp = getRange_Compass();
             }       
          }  //correct positioning
          else if(comp < temp-8){
            while(comp < temp-3){ 
              //Serial.printf("Comp:[%d] RA:[%d]--", comp, temp);
              obstacle(0x30, "reorient final right to last edge");  
              delay(2); //allow turn
              obstacle(0xAA, "stop from final r_right to last edge");
              delay(4); //allow pause
              comp = getRange_Compass();
            }
          }
          else{
                messageCb(0x01);
                delay(20); //allow ahead
                obstacle(0xAA, "stop before last edge");
                delay(10); //allow pause
                comp = getRange_Compass();
                us = getRange_Ultrasound();
                //Serial.printf("Wall is %d far away\n", us);              
          }
        }
      /*        
      *help rotation
      */  
      obstacle(0x40, "help rotate left"); 
      delay(250); //allow turn
      obstacle(0xAA, "help rotate left stop");
    }//if us < 15

    /*        
    *check for final target
    */  
    //if(edge detected and heading straight)
    if((ira < 6 || irb < 6) && (heading_coo-15 < comp && comp < heading_coo+15)){ 
      
        /*
        *reverse, not to fall inside hole when turning
        */                         
        obstacle(0x02, "reverse, avoid hole");
        delay(220); //allow reverse
        obstacle(0xAA, "avoid hole reverse stop");
        delay(190); //allow pause
  
        /*
        *turn right, check for wall first
        *///reference angle, rotate by 90 degrees
        right_angle = heading_coo + 90;
        //reset angle if past 360
        if(right_angle > 360)
        {
          right_angle -= 360;
        }
    
        obstacle(0x10, "check for wall, turn right");
        delay(300); //allow turn
        obstacle(0xAA, "check for wall, stop");
        delay(500); //allow pause to get sensors
        comp = getRange_Compass();
        us = getRange_Ultrasound();
        //Serial.printf("Left turn find wall Comp:[%d] US:[%d]\n --", comp, us);

        /*
        * move straight, check for wall and gap
        */
        while(((getRange_Infrared(analog_IR_1) > 6) || (getRange_Infrared(analog_IR_2) > 6)) && (getRange_Ultrasound() > 15)){
          if(comp > right_angle+8){
            while(comp > right_angle+5){ 
              //Serial.printf("Comp:[%d] RA:[%d]--", comp, right_angle);           
              obstacle(0x40, "reorient left adjust"); 
              delay(2); //allow adjust
              obstacle(0xAA, "stop from reorient left adjust");
              delay(4); //allow pause
              comp = getRange_Compass();
              }       
          }  //correct positioning
          else if(comp < right_angle-8){
            while(comp < right_angle-5){ 
              //Serial.printf("Comp:[%d] RA:[%d]--", comp, right_angle);
              obstacle(0x30, "reorient right adjust");  
              delay(2); //allow turn
              obstacle(0xAA, "stop from final reorient right adjust");
              delay(4); //allow pause
              comp = getRange_Compass();
            }
          }
          else{
                messageCb(0x01); // head forward straight
                delay(80); //allow ahead
                obstacle(0xAA, "stop from straight");
                delay(10); //allow pause
                comp = getRange_Compass();
                //Serial.println("final straight a bit more");            
          }
        }//while no edge and no wall
        /*
        *found the first edge or wall
        */
                   
        /*
        * hole and no finish in sight //if wall = turn back and move straight, count steps to check if at the very end
        */          
        if(getRange_Ultrasound() < 15){ 
            //left turn, move straight, turn and check if edge is present or not
            //Serial.println("Just Avoiding the hole. Wall Detected");
            /*
            * left turn
            */               
            obstacle(0x20, "turn and check for a way"); 
            delay(300); //allow turn
            obstacle(0xAA, "stop from check for a way");
            delay(150); //allow pause
            count = 10; //forward counter
  
            while((getRange_Infrared(analog_IR_1) > 5) || (getRange_Infrared(analog_IR_2) > 5) && (count >= 0)){
                if(comp > heading_coo +8){
                  while(comp > heading_coo +5){ 
                    //Serial.printf("Comp:[%d] RA:[%d]--", comp, heading_coo);           
                    obstacle(0x40, "straight check for way reorient left");  
                    delay(2); //allow straight
                    obstacle(0xAA, "stop check for way  reorient left");
                    delay(4); //allow pause 
                    comp = getRange_Compass();
                  }       
                }  //correct positioning
                else if(comp < heading_coo -8){
                  while(comp < heading_coo -5){ 
                    //Serial.printf("Comp:[%d] RA:[%d]--", comp, heading_coo);
                    obstacle(0x30, "straight check for way reorient right");  
                    delay(2); //allow turn
                    obstacle(0xAA, "stop check for way  reorient right");
                    delay(4); //allow pause
                    comp = getRange_Compass();
                  }
                }
                else{
                      messageCb(0x06);
                      delay(80); //allow ahead
                      obstacle(0xAA, "stop from straight");
                      delay(10); //allow pause
                      comp = getRange_Compass();
                }
                count--;
            }//while there is path ahead

            /*
            *if edge and no straight (count not 0) = reverse till the center
            */ 
            if(count != 0){    
              //reverse, not to fall inside hole
              obstacle(0x02, "final reverse, avid hole");
              delay(220); //allow turn
              obstacle(0xAA, "final reverse stop");
              delay(190); //allow pause
           
              /*
              *turn right again and reverse 60
              */
              right_angle = heading_coo + 90;
              if(right_angle > 360)
              {
                right_angle -= 360;
              }
          
              obstacle(0x10, "final, turn right");
              delay(250); //allow turn
              obstacle(0xAA, "final, stop, check for wall");
              delay(500); //allow pause to get sensors
              us = getRange_Ultrasound();            
              /*
              *turn right again and reverse 60
              */
              while(getRange_Ultrasound() < 70){
                obstacle(0x02, "final reverse, avid hole");
                delay(80); //allow turn
                obstacle(0xAA, "final reverse stop");
                delay(10); //allow pause
                }//exit count !=0, go to main loop
              /*        
              *help rotation
              */  
              
              obstacle(0x40, "help rotate left"); 
              delay(100); //allow turn
              obstacle(0xAA, "help rotate left stop");
            }//if count not zero 
                            
        }//if close to wall
        else{//on ending platform
 
          obstacle(0x20, "final, turn left 180");
          delay(500); //allow turn
          obstacle(0xAA, "final, stop, check for wall");
          delay(500); //allow pause to get sensors
          comp = getRange_Compass();

          right_angle = heading_coo - 90;
          if(right_angle < 0)
          {
            right_angle += 360;
          }
          //move straight until other edge
          while((getRange_Infrared(analog_IR_1) > 5) || (getRange_Infrared(analog_IR_2) > 5)){
            if(comp > right_angle+8){
                while(comp > right_angle+5){ 
                  //Serial.printf("Comp:[%d] RA:[%d]--", comp, right_angle);           
                  obstacle(0x40, "reorient left adjust"); 
                  delay(2); //allow adjust
                  obstacle(0xAA, "stop from reorient left adjust");
                  delay(4); //allow pause
                  comp = getRange_Compass();
                }       
              }  //correct positioning
              else if(comp < right_angle-8){
                while(comp < right_angle-5){ 
                  //Serial.printf("Comp:[%d] RA:[%d]--", comp, right_angle);
                  obstacle(0x30, "reorient right adjust");  
                  delay(2); //allow turn
                  obstacle(0xAA, "stop from final reorient right adjust");
                  delay(4); //allow pause
                  comp = getRange_Compass();
                }
              }
              else{
                    messageCb(0x01); // head forward straight
                    delay(25); //allow ahead
                    obstacle(0xAA, "stop from straight");
                    delay(10); //allow pause
                    comp = getRange_Compass();
                    //Serial.println("final straight a bit more");            
              }
          }//while there is path
 
          if(getRange_Infrared(analog_IR_1) < 5 || getRange_Infrared(analog_IR_2) < 5){
            while(1){
              obstacle(0xAA, "finish");
            }
          }//final stop
        }//else if on final platform
    }//if edge detect
                     
/*             
*correct positioning or move straight
*/           
    if(comp < heading_coo-8){
        //right
        while(comp < heading_coo-3){          
          obstacle(0x30, "reorient right");  
          delay(10); //turn a bit 
          obstacle(0xAA, "stop from r_right");
          delay(4); //stop a bit 
          comp = getRange_Compass();
          //Serial.printf("Robot is looking at %d\n", comp);
        }               
    }  //correct positioning
    else if(comp > heading_coo+8){
        //left
        while(comp > heading_coo+3){          
          obstacle(0x40, "reorient left");  
          delay(10); //turn a bit 
          obstacle(0xAA, "stop from r_left");
          delay(4); //stop a bit 
          comp = getRange_Compass();
          //Serial.printf("Robot is looking at %d\n", comp);
        }                
    }
    else{
      //straight 2 sec
      messageCb(0x01);
      //Serial.printf("Robot is moving ahead\n");
      delay(140); 
    }  
  }
//}




