#include <Servo.h>


// Declaration of Motor Controller PINs
#define MFR_A_PIN	11  // FRONT RIGHT MOTOR
#define MFR_B_PIN	12 
#define MFR_PWM_PIN     13 	// PULSE WIDTH MODULATION PIN

#define MFL_A_PIN	5	//FRONT LEFT MOTOR
#define MFL_B_PIN 	6
#define MFL_PWM_PIN     7	// PULSE WIDTH MODULATION PIN

#define MBR_A_PIN	9	//BACK RIGHT MOTOR
#define MBR_B_PIN	10
#define MBR_PWM_PIN	8	// PULSE WIDTH MODULATION PIN

#define MBL_A_PIN	3	//BACK LEFT MOTOR
#define MBL_B_PIN	4
#define MBL_PWM_PIN	2 	// PULSE WIDTH MODULATION PIN 

// Declaration of Parallax IR Line follower PINs
#define IRL_PIN_s0	22
#define IRL_PIN_s1	23
#define IRL_PIN_s2	24
#define IRL_PIN_s3	25
#define IRL_PIN_s4	26
#define IRL_PIN_s5	27
#define IRL_PIN_s6	28
#define IRL_PIN_s7	29
#define IRL_PIN_EN      30

//****************************************//
//      Declaration of the Servo PINs     //
//****************************************//
// Right Arm
#define R_ARM1_PIN 		42
#define R_ARM2_PIN 		44
#define R_ARM3_PIN 		46
#define R_ARM4_PIN 		48
#define R_ARM5_PIN 		50
#define R_ARMC_PIN 		52

// Left Arm
#define L_ARM1_PIN 		43
#define L_ARM2_PIN 		45
#define L_ARM3_PIN 		47
#define L_ARM4_PIN 		49
#define L_ARM5_PIN 		51
#define L_ARMC_PIN 		53

// Servo Object Declaration
// Right Arm
Servo rArm_Servo1;
Servo rArm_Servo2;
Servo rArm_Servo3;
Servo rArm_Servo4;
Servo rArm_Servo5;
Servo rClaw;

// Left Arm
Servo lArm_Servo1;
Servo lArm_Servo2;
Servo lArm_Servo3;
Servo lArm_Servo4;
Servo lArm_Servo5;
Servo lClaw;


int IRL_in;
int Tcount=0;


void setup() {
Serial.begin(9600);
	// Setting up Motor PINs
	pinMode(MFR_A_PIN,OUTPUT);
	pinMode(MFR_B_PIN,OUTPUT);
	pinMode(MFR_PWM_PIN,OUTPUT);// Controls speed- Pulse width modulation
	
	// Motor Front Left Side
	pinMode(MFL_A_PIN,OUTPUT);
	pinMode(MFL_B_PIN,OUTPUT);
	pinMode(MFL_PWM_PIN,OUTPUT);
	 
	// Motor Back Right Side
	pinMode(MBR_A_PIN,OUTPUT);
	pinMode(MBR_B_PIN,OUTPUT);
	pinMode(MBR_PWM_PIN,OUTPUT);
	
	// Motor Back Left Side
	pinMode(MBL_A_PIN,OUTPUT);
	pinMode(MBL_B_PIN,OUTPUT);
	pinMode(MBL_PWM_PIN,OUTPUT);

	pinMode(IRL_PIN_s0, INPUT);
	pinMode(IRL_PIN_s1, INPUT);
	pinMode(IRL_PIN_s3, INPUT);
	pinMode(IRL_PIN_s4, INPUT);
	pinMode(IRL_PIN_s5, INPUT);
	pinMode(IRL_PIN_s6, INPUT);
	pinMode(IRL_PIN_s7, INPUT);
        pinMode(IRL_PIN_EN, OUTPUT);
        digitalWrite(IRL_PIN_EN, HIGH);

	//Initaizing Robotic Right Arm Angles
	rArm_Servo1.write(150);
	rArm_Servo2.write(170);
	rArm_Servo3.write(180);
	rArm_Servo4.write(180);
	rArm_Servo5.write(180);
	
	//Initiaizing Robotic Left Arm Angles
	lArm_Servo1.write(10);
	lArm_Servo2.write(5);
	lArm_Servo3.write(5);
	lArm_Servo4.write(5);
	lArm_Servo5.write(5);
	
	//Initiaizng Heavy Arm Claw to hold position.
	lClaw.writeMicroseconds(1500);
	
	rArm_Servo1.attach(R_ARM1_PIN); // Attaches(inputs the pin out to the Servo object) class) the servos to their respective pin	
	rArm_Servo2.attach(R_ARM2_PIN);	// Attaches(inputs the pin out to the Servo object) class) the servos to their respective pin
	rArm_Servo3.attach(R_ARM3_PIN);	// Attaches(inputs the pin out to the Servo object) class) the servos to their respective pin
	rArm_Servo4.attach(R_ARM4_PIN);	// Attaches(inputs the pin out to the Servo object) class) the servos to their respective pin
	//rArm_Servo5.attach(R_ARM5_PIN);	// Attaches(inputs the pin out to the Servo object) class) the servos to their respective pin
	//rClaw.attach(R_ARMC_PIN);	// Attaches(inputs the pin out to the Servo object) class) the servos to their respective pin
	
	lArm_Servo1.attach(L_ARM1_PIN); // Attaches(inputs the pin out to the Servo object) class) the servos to their respective pin
	lArm_Servo2.attach(L_ARM2_PIN);	// Attaches(inputs the pin out to the Servo object) class) the servos to their respective pin
	lArm_Servo3.attach(L_ARM3_PIN);	// Attaches(inputs the pin out to the Servo object) class) the servos to their respective pin
	lArm_Servo4.attach(L_ARM4_PIN);	// Attaches(inputs the pin out to the Servo object) class) the servos to their respective pin
	//lArm_Servo5.attach(L_ARM5_PIN);	// Attaches(inputs the pin out to the Servo object) class) the servos to their respective pin
	lClaw.attach(L_ARMC_PIN);	// Attaches(inputs the pin out to the Servo object) class) the servos to their respective pin

}

void loop() {

IRL_in=0;
for (int i=0; i<8; i++)
{ IRL_in = (IRL_in << 1) + digitalRead(IRL_PIN_s7 - i);}

/*if (IRL_in == 0b11111111) // stop strider
{nothing();}*/
    
if ( (IRL_in == 0b11000011) || (IRL_in == 0b11100011) || (IRL_in == 0b11000111) || (IRL_in == 0b11000111) ) // strider go forward
{forward();}
      
if ( (digitalRead(IRL_PIN_s6) == 0) || (digitalRead(IRL_PIN_s6) == 0) && (digitalRead(IRL_PIN_s7) ==0 ) ) // curve strider to follow line
{curveleft();} 
    
if ( (digitalRead(IRL_PIN_s1) == 0) || (digitalRead(IRL_PIN_s1) == 0) && (digitalRead(IRL_PIN_s0) ==0 ) ) // curve strider to follow line
{curveright();}
    
/*if ((IRL_in == 0b11110000) || (IRL_in == 0b11100000) || (IRL_in == 0b11000000)) // turn right at angle
{right();}
     
if ((IRL_in == 0b00001111) || (IRL_in == 0b00011111) || (IRL_in == 0b00111111)) // turn left at angle
{left();}  
*/
if (IRL_in == 0b00000000)
{
                      
                       Tcount=Tcount+1;
        	        analogWrite(MFR_PWM_PIN, 0);
			analogWrite(MFL_PWM_PIN, 0);
			analogWrite(MBR_PWM_PIN, 0);
			analogWrite(MBL_PWM_PIN, 0);
                        Serial.println("stopped");
                        delay(2000);
                        //while(~(IRL_in == 0b11000011) || (IRL_in == 0b11100011) || (IRL_in == 0b11000111) || (IRL_in == 0b11000111)||(IRL_in == 0b11101111)||(IRL_in == 0b11110111)){
                        //while(IRL_in<255){
                        ReadIRL();
                        while(IRL_in < 0b11111111)
                        {
                           Serial.println("while < 0b11111111");
                          forward();
                          delay(500);
                          ReadIRL();
                          /*delay(200);
                          nothing();
                          delay(200);*/
                          
                        }
                           Serial.println("stopped after < 111111111");
                          nothing();
                          delay(200);
                        while(IRL_in == 0b11111111)
                        {
                          Serial.println("while == 0b11111111");
                              left();
                              //delay(1000);
                              
                              //nothing();
                              //delay(100);
                              ReadIRL();
                        }
                        Serial.println("stop after == 0b11111111");
                        nothing();
                        delay(2000);
                        
                        if (Tcount == 2)
                        {
                          do
                          {
                          Serial.println("tcount");
                        analogWrite(MFR_PWM_PIN, 0);
			analogWrite(MFL_PWM_PIN, 0);
			analogWrite(MBR_PWM_PIN, 0);
			analogWrite(MBL_PWM_PIN, 0);
                        // do game challenge
                          } while(Tcount!=3);
                      }

                       
}
                        }
                        

void ReadIRL(void)
{
  IRL_in = 0;
  for (int i=0; i<8; i++){
      IRL_in = (IRL_in << 1) + digitalRead(IRL_PIN_s7 - i);
    }
}

void nothing()
{
                         Serial.println("Stop");
                         Serial.println(IRL_in, BIN);
                        
                        digitalWrite(MFR_A_PIN, LOW);
			digitalWrite(MFR_B_PIN, LOW);
			
			digitalWrite(MBR_A_PIN, LOW);
			digitalWrite(MBR_B_PIN, LOW);
		
			digitalWrite(MFL_A_PIN, LOW);
			digitalWrite(MFL_B_PIN, LOW);
		
			digitalWrite(MBL_A_PIN, LOW);
			digitalWrite(MBL_B_PIN, LOW);

      			analogWrite(MFR_PWM_PIN, 0);
			analogWrite(MFL_PWM_PIN, 0);
			
			analogWrite(MBR_PWM_PIN, 0);
			analogWrite(MBL_PWM_PIN, 0);
    
}

void forward()
{
                         Serial.println("forward");
  
                        digitalWrite(MFR_A_PIN, LOW);
			digitalWrite(MFR_B_PIN, HIGH);
			
			digitalWrite(MBR_A_PIN, HIGH);
			digitalWrite(MBR_B_PIN, LOW);
		
			digitalWrite(MFL_A_PIN, LOW);
			digitalWrite(MFL_B_PIN, HIGH);
		
			digitalWrite(MBL_A_PIN, LOW);
			digitalWrite(MBL_B_PIN, HIGH);
		
			analogWrite(MFR_PWM_PIN, 147);
			analogWrite(MFL_PWM_PIN, 157);
			
			analogWrite(MBR_PWM_PIN, 130);
			analogWrite(MBL_PWM_PIN, 130);
}

void curveleft()
{
                        //add more pwm to  to right wheel front
                     Serial.println("left curve");
                     Serial.println(IRL_in, BIN);
                        digitalWrite(MFR_A_PIN, LOW);
			digitalWrite(MFR_B_PIN, HIGH);
			
			digitalWrite(MBR_A_PIN, HIGH);
			digitalWrite(MBR_B_PIN, LOW);
		
			digitalWrite(MFL_A_PIN, LOW);
			digitalWrite(MFL_B_PIN, HIGH);
		
			digitalWrite(MBL_A_PIN, LOW);
			digitalWrite(MBL_B_PIN, HIGH);
		
			analogWrite(MFR_PWM_PIN, 255); //add more when necessay // 147
			analogWrite(MFL_PWM_PIN, 0); // 157
			
			analogWrite(MBR_PWM_PIN, 255); // 130
			analogWrite(MBL_PWM_PIN, 0); // 130
}

void curveright()
{
                      //add more pwm to  to right wheel front
                     Serial.println("right curve");
                     Serial.println(IRL_in, BIN);
                        digitalWrite(MFR_A_PIN, LOW);
			digitalWrite(MFR_B_PIN, HIGH);
			
			digitalWrite(MBR_A_PIN, HIGH);
			digitalWrite(MBR_B_PIN, LOW);
		
			digitalWrite(MFL_A_PIN, LOW);
			digitalWrite(MFL_B_PIN, HIGH);
		
			digitalWrite(MBL_A_PIN, LOW);
			digitalWrite(MBL_B_PIN, HIGH);
		
			analogWrite(MFR_PWM_PIN, 0); //147
			analogWrite(MFL_PWM_PIN, 255); //add more when necessay //157
			
			analogWrite(MBR_PWM_PIN, 0); // 130
			analogWrite(MBL_PWM_PIN, 255); //130
}

void left()
{
                       // strider moves up abit to make centre inline with turn
                        Serial.println("left turn case 2");

                        
                  // delay should be enough to make center of strider inline with turn
                     /*   digitalWrite(MFR_A_PIN, LOW);
			digitalWrite(MFR_B_PIN, HIGH);
			
			digitalWrite(MBR_A_PIN, HIGH);
			digitalWrite(MBR_B_PIN, LOW);
		
			digitalWrite(MFL_A_PIN, LOW);
			digitalWrite(MFL_B_PIN, HIGH);
		
			digitalWrite(MBL_A_PIN, LOW);
			digitalWrite(MBL_B_PIN, HIGH);
		
			analogWrite(MFR_PWM_PIN, 147);
			analogWrite(MFL_PWM_PIN, 157);
			
			analogWrite(MBR_PWM_PIN, 130);
			analogWrite(MBL_PWM_PIN, 130);
                        delay(200);
                        
                         // stop strider for turning
                        analogWrite(MFR_PWM_PIN, 0);
			analogWrite(MFL_PWM_PIN, 0);
			
			analogWrite(MBR_PWM_PIN, 0);
			analogWrite(MBL_PWM_PIN, 0);
                        delay(2500);
                       
                       // loop to spin strider until line follower lines up
                        while ((IRL_in != 0b11000011) || (IRL_in != 0b11000111) || (IRL_in != 0b11100011) )
                        {*/
                          
                        
                        digitalWrite(MFR_A_PIN, LOW);
			digitalWrite(MFR_B_PIN, HIGH);
			
			digitalWrite(MBR_A_PIN, HIGH);
			digitalWrite(MBR_B_PIN, LOW);
		
			digitalWrite(MFL_A_PIN, HIGH);
			digitalWrite(MFL_B_PIN, LOW);
		
			digitalWrite(MBL_A_PIN, HIGH);
			digitalWrite(MBL_B_PIN, LOW);
		
			analogWrite(MFR_PWM_PIN, 255);
			analogWrite(MFL_PWM_PIN, 255);
			
			analogWrite(MBR_PWM_PIN, 255);
			analogWrite(MBL_PWM_PIN, 255);

                      /*  IRL_in=0;
                        for (int i=0; i<8; i++)
                        {  
                         IRL_in = (IRL_in << 1) + digitalRead(IRL_PIN_s7 - i);
                        }
                        
                        } */

}

void right()
{
          // strider moves up abit to make centre inline with turn
                        Serial.println("right turn case 2");
 
                        
                        // delay should be enough to make center of strider inline with turn
                       /* digitalWrite(MFR_A_PIN, LOW);
			digitalWrite(MFR_B_PIN, HIGH);
			
			digitalWrite(MBR_A_PIN, HIGH);
			digitalWrite(MBR_B_PIN, LOW);
		
			digitalWrite(MFL_A_PIN, LOW);
			digitalWrite(MFL_B_PIN, HIGH);
		
			digitalWrite(MBL_A_PIN, LOW);
			digitalWrite(MBL_B_PIN, HIGH);
		
			analogWrite(MFR_PWM_PIN, 147);
			analogWrite(MFL_PWM_PIN, 157);
			
			analogWrite(MBR_PWM_PIN, 130);
			analogWrite(MBL_PWM_PIN, 130);
                        delay(200);
                        
                         // stop strider for turning
                        analogWrite(MFR_PWM_PIN, 0);
			analogWrite(MFL_PWM_PIN, 0);
			
			analogWrite(MBR_PWM_PIN, 0);
			analogWrite(MBL_PWM_PIN, 0);
                        delay(2500);
                       
                       // loop to spin strider until line follower lines up
                        while ((IRL_in != 0b11000011) || (IRL_in != 0b11000111) || (IRL_in != 0b11100011) )
                        {
                          
                        */
                        digitalWrite(MFR_A_PIN, HIGH);
			digitalWrite(MFR_B_PIN, LOW);
			
			digitalWrite(MBR_A_PIN, LOW);
			digitalWrite(MBR_B_PIN, HIGH);
		
			digitalWrite(MFL_A_PIN, LOW);
			digitalWrite(MFL_B_PIN, HIGH);
		
			digitalWrite(MBL_A_PIN, LOW);
			digitalWrite(MBL_B_PIN, HIGH);
		
			analogWrite(MFR_PWM_PIN, 255);
			analogWrite(MFL_PWM_PIN, 255);
			
			analogWrite(MBR_PWM_PIN, 255);
			analogWrite(MBL_PWM_PIN, 255);

                        /*IRL_in=0;
                        for (int i=0; i<8; i++)
                        {  
                         IRL_in = (IRL_in << 1) + digitalRead(IRL_PIN_s7 - i);
                        }
                        
                        } */ 

}

