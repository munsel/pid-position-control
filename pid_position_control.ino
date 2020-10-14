/*******---DEVICES---************************************************************************************/
#define ARDUINO_MEGA
//#define ARDUINO_UNO
#define DEBUGMODE

/*******---INCLUDES---***********************************************************************************/
#include<avr/io.h>
#include<Arduino.h>
#include<avr/interrupt.h>
/******---CONSTANTS---************************************************************************************/
#define n 18              //number of elements in arrays of the lookup function
#define two_pi 6.283185
#define pi 3.1415926
#define INTEGRAL_SATURATE 1000  //Saturation for the integral
//pin definitions
#define MOT1_DIRECTION 4
#define MOT1_ENABLE 5
#define SENS_DIST A0
#define ENCODE_A 2
#define ENCODE_B 3
//define reference voltage for ADC
#ifdef ARDUINO_MEGA
  #define REFERENCE 2.56
#else
  #define REFERENCE 5.0
#endif
//define debug toggle pin for measurements
#ifdef DEBUGMODE
  #define DEBUGPIN 53
#endif
/*******---PROTOTYPES---***********************************************************************************/
float lookup(float x, float xspace[n], float yspace[n]);
int moto_out(float y);
float evaluate_encoder();
void PID_control(float w); 



/*******---GLOBAL VARIABLES---******************************************************************************/
volatile int cnt_encoder=0;
int i=0;
//Voltage values
float U_sensor[n]={0.51, 0.57, 0.61, 0.65, 0.69, 0.73, 0.77, 0.83, 0.9, 0.96, 1.05, 1.15, 1.28, 1.42, 1.61, 1.85, 2.15, 2.54}; //must increase
//corresponding distances
float corr_dist[n]={240, 220, 200, 180, 170, 160, 150, 140,130, 120, 110, 100, 90, 80, 70, 60, 50, 40};
//PID-control variables
float e, e_old=0, e_sum=0, x, y,omega, Kp=0.05, one_over_Tn=2.5, Tv=0.001;
float sensor_voltage;
int raw_adc_value;

/*********---SETUP---***************************************************************************************/


void setup(){
  //attach Interrupt for reading the encoder
  attachInterrupt(0,read_encoder,RISING);//checks pin 2 on Arduino for rising edge
   //set PWM-frequency to 31kHz to make the Motor quiet
  TCCR3B &= 0xF8;
  TCCR3B |= 0x01;
  //set properties for timer 5
  TCNT5=0;			//counter initial state
  TCCR5B &=0x00;		//Timer/Counter Control Register 5B
  TCCR5A &=0x00;
  TCCR5B |= _BV(WGM52)|_BV(CS51); //enable CTC(Clear Timer on Compare Match) and use (system clock)/8
  OCR5A = 19999;//sets Output Compare Register A to 19999 for 100Hz 
  
  //define PIN_MODES
  pinMode(MOT1_DIRECTION, OUTPUT);
  pinMode(MOT1_ENABLE,OUTPUT);
  #ifdef DEBUGMODE
    pinMode(DEBUGPIN,OUTPUT);
  #endif
  //configure Serial Port with baudrate
  Serial.begin(9600);
  //set Voltage Reference for ADC(only on ArduinoMEGA)
  #ifdef ARDUINO_MEGA
    analogReference(INTERNAL2V56);
  #endif
  
}

/*********---LOOP---******************************************************************************************/


void loop(){
   if(TIFR5&(1<<OCF5A)){//is Compare flag set?
     TIFR5|=(1<<OCF5A);//reset flag
    #ifdef DEBUGMODE  //
    digitalWrite(DEBUGPIN,HIGH);
    #endif
    switch(i){
      case 0:
        omega=evaluate_encoder();
        raw_adc_value=analogRead(SENS_DIST);
        sensor_voltage=(REFERENCE/1024.0)*raw_adc_value;
        x=lookup(sensor_voltage, U_sensor, corr_dist);
        PID_control(70);
        Serial.println(moto_out(y));
        i++;
        break;
      case 10:
        
        i=0;
        break;
      default:
        i++;
        break;
    }
    
    
    #ifdef DEBUGMODE
    digitalWrite(DEBUGPIN,LOW);
    #endif
  }
  //Serial.println(cnt_encoder);
 // digitalWrite(DEBUGPIN,OCF1A);
  
  
}

/******---FUNCTIONS---*****************************************************************************************/


/*lookuptable function returns linear interpolated
**value of y by given sets of x- and y cordinates
**and one sample of x, wich must be inside the x-set.
**x-set or xspace must be sorted from small to big
**When the x value is out of range, it returns the border
**element in the yspace array, so no extrapolation!*/
float lookup(float x, float xspace[n], float yspace[n]){
	int i,j;
	float y=0,prestate, dydx, deltax;
	if( x < xspace[0]){
		return yspace[0];
	}
	else if( x > xspace[n-1] ){
		return yspace[n-1];
	}
	else{
		i=0;
		while( !( x>=xspace[i] && x<=xspace[i+1] ) ){ //count as long as x is one or in between two samples
			i++;
		}
		dydx=(yspace[i+1]-yspace[i])/(xspace[i+1]-xspace[i]);
		deltax=x-xspace[i];
		y=yspace[i]+deltax*dydx;
		return y;
	}
}


/*moto_out function gets value from the controller and sets the corresponding 
**output pins for direction and duty cycle of the motor's PWM signal. 
**it also returns the 8 bit PWM value (saturation at x<-255 or x>255 ) */
int moto_out(float degree_of_operation){
  int output;
  if(degree_of_operation<0){
    digitalWrite(MOT1_DIRECTION,LOW);
    degree_of_operation*=-1;
  }
  else digitalWrite(MOT1_DIRECTION,HIGH);
  degree_of_operation += 100;
  if(degree_of_operation>255)degree_of_operation=255;
  output=(int)degree_of_operation;
  analogWrite(MOT1_ENABLE,y);
  return output; 
}


/*Interrupt Service routine for reading the encoder*/
void read_encoder(){
 if( digitalRead(ENCODE_B) == HIGH )cnt_encoder++;
 else cnt_encoder--; 
}


/*evaluate_encoder function  gets the velocity from the counter and
**sets encoder counter to zero*/
float evaluate_encoder(){ 
 float v;
 v=(float)cnt_encoder*two_pi*0.2; //gives Output in rad/s
 cnt_encoder=0;
 return v;
}

/*ISR(TIMER1_COMPA_vect){
  timeflag=1;
}*/

/*PID Algorithm which gets the set point value and
**calculates the controller output*/
void PID_control(float w){
  e=w-x;
  e_sum+=e;
  if(e_sum>INTEGRAL_SATURATE){
    e_sum=INTEGRAL_SATURATE;
  }else if(e_sum <-INTEGRAL_SATURATE){
    e_sum=-INTEGRAL_SATURATE;
  }
  y=Kp*( e + ((e-e_old)*Tv) + e_sum*one_over_Tn );
  e_old=e;
}



