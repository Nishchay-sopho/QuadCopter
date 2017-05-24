//--------------------------------------------ABOUT THE PROJECT-------------------------------------//
/* The following project is an attempt to make a Flight controller for the quadcopter.
 * A project made by NisHchay Agrawal and Siddhant Laddha.
 * Special thanks to Joop Brokking..
 * http://www.brokking.net/ymfc-al_main.html
 * 
 * Watch the following video series so as to get started with it..
 * https://www.youtube.com/watch?v=XFxqFQwRumc&list=PL0K4VDicBzsibZqfa42DVxC8CGCMB7G2G
 * https://www.youtube.com/watch?v=4BoIE8YQwM8
 * 
 */
// -------------------------------------------IMPORTANT--------------------------------------------------------//
// It is strongly reccommended that you(the reader) look up the functions and syntax used on Arduino.cc
// If you want to jump into more details of the program syntax taking a look at C language references might help..
// Please Refer to YMFC -3D tutorial series on Youtube by Joop Brooking..
// -------------------------------------------IMPORTANT--------------------------------------------------------//

/* #define <term> <val>
 is a preprocessor directive. Thus the processor will replace the term wherever it appears in the whole
 program with the 'val' even before compiling begins. 
 This will add to the readability of the program and will not even consume any memory(as opposed to name the 
 variable and then assigning it the value 'val'
 */

// Refer to the SCHEMATIC DIAGRAM for connections and then the following defintions will make sense.
#include <Wire.h>

// FOR THE RECEIVER CHANNELS and corresponding arduino digital I/O pins.
// Note that these are ALSO the  pins which are in Pin Change interrupt vector 0.
#define channel_1 8
#define channel_2 9
#define channel_3 9
#define channel_4 10

// FOR THE OUTPUT(MOTOR) CHANNELS and corresponding arduino digital I/O pins.
#define motor_1 4
#define motor_2 5
#define motor_3 6
#define motor_4 7

// Read the Calibration Report for the Transmitter values and their corresponding pulse widths..
// Defining the zero and max_throttle values in pulse width in microseconds. May vary with the Transmitter.
#define zero_throttle 900 // The control signal when Tx reads zero throttle for arming
#define min_throttle  1000 // The control signal when motor actually moves.
#define max_throttle 1976 // The maximum value of control signal beyond this values the motor behaviour
// is unpredictable..

// the gyro and the accelerometer sensitivity and the corresponding dividend(refer datasheet)

#define a_sense
#define g_sense 131

const int MPU_addr=0x68;  // I2C address of the MPU-6050 when AD0 is at ground.
int16_t  GyX,GyY,GyZ; // Raw values obtained from the gyro are 16 bit signed integers.
int ax,ay,az,gx,gy,gz; // The meaningful values after raw values stage 1 of processing the data.
int gx_m=0,gy_m=0,gz_m=0; // The meaningful memory values after raw values stage 1 of processing the data.
double ax_avg,ay_avg,az_avg,gx_avg,gy_avg,gz_avg;// The average of values of sensor for calibration.
float roll_g=0,yaw_g=0,pitch_g=0;
float roll_a=0,yaw_a=0,pitch_a=0;
float acc_tot;
// Just a time counting variable for motor control
unsigned long timemotor;   

//---------------------THE FOLLOWING ARE THE DEFINITIONS FOR THE TX_CONTROLS_TO_ANGLE_CONVERSIONS-------------//
//The following definitions contain the pulse durations for various transmitter control orientation
// Refer the Calibration report to obtain the average of 1000 values.
// Take a Calibration report everytime Tx is changed.(preferably before every flight)??
// The following variables are set to be volatile so that they can be easily accessed from the 
// Interrupt service routine function without much time delay.
volatile int recval [6];  // The array stores the duration of the HIGH pulse on channel i (in microseconds) in recval[i].  
volatile float throttle; // The above quantity is the throttle value from the reciever.
volatile float r_final[4]; // The human understandable values from the receiver(in degress or degrees
//per second). it is important to note the yaw = r_final[1],pitch = r_final[2] ,roll = r_final[3]

volatile boolean last_channel[6] ;
volatile unsigned long receiver_input [6];
volatile unsigned long timer[6];
volatile unsigned long current_time;
volatile boolean state =0; // For checking the on or off state from the channel 5.
//Obtained from the calibration report and stored in matrix format.
// channel 1 roll zero = 1480   -ve extreme = 918  +ve extreme = 1998 
// channel  2 pitch zero = 1504   -ve extreme = 928  +ve extreme = 2044
// channel 3Throttle zero = 916 full = 2052
// channel 4  yaw zero = 1468   -ve extreme = 2004  +ve extreme = 912
int Tx_base_val [5][4] = {  
  {
    0,0,  0,   0  }
  ,
  {
    0,918,1480,1998  }
  ,
  {
    0,928,1504,2044  }
  ,
  {
    0,916,0,2052  }
  ,
  {
    0,2004,1468,912  }    
};

//--------------THE FOLLOWING ARE THE DEFINITIONS FOR THE PID CALCULATIONS-------------------------------------//
// Values for PID ..
// all the references to the yaw,pitch and roll hereby refer to the respective angular rate of change.
float gyro_ypr[4];
float gyro_ypr_prev[4] = {0,0,0,0};
float pid_output_ypr[4];
// initialising the memory variables to zero.(RESETTING THEM!!)E stands for ERROR.
float E_mem[4] = {
  0,0,0,0}; 
float E_sum[4] = {
  0,0,0,0};
float E_diff[4] = {
  0,0,0,0};
// The pid coefficients for yaw ,pitch and roll separately..
float p_i_d__y_p_r[4][4] = { 
  (1,1,1,1),
  (1,1,1,0),
  (1,10,0.5,0),
  (1,10,0.5,0)};                                    
;
//    | 0 | p | i | d |
//  0 | - | - | - | - |
//    |   |   |   |   |
//  y | - |   |   |   |
//    |   |   |   |   |
//  p | - |   |   |   |
//    |   |   |   |   |
//  r | - |   |   |   |

// Control signals of each motor..
float c_sig_m1,c_sig_m2,c_sig_m3,c_sig_m4;
// The maximum angular velocities you want for the quad..//UNSCALED..
float ypr_setpoint[4]= {
  0,20,20,20}; 
float E[4];

// weightage are neccessary since the values of gyro and reciever are in different scales.
float wG = 1;// The weightage given to gyro inputs.
float wR = 0;// The weightage given to reciver inputs.

unsigned long looptime; // to monitor the refresh rate of the Quadcopter.


void setup()
{

  pinModeinit(); // Will initialise the pinMode and the interrupts PSCIN0_VECT
  // Signal for Gyro Calibration beginning..
  digitalWrite(13,1);
  //GYRO CALIBRATION BEGINS
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  Serial.begin(115200);
/*  Serial.println("Do you want to Calibrate the Tx-Rx pair? \n 1--> YES  0-->NO ");
  while (Serial.available()); // Wait until input is received.
 
  
    if (Serial.read == '1') // The calibration function is called...
    calibration_report();

    else if(Serial.read == '0')
    break;

  
  */
  calib();
  //Marker for end of Gyro Calibration..
  digitalWrite(13,0);

  // ARMING ALL THE MOTORS.
   for(int i = 4;i<8;i++)
   motorarm();
}

void loop()
{
  looptime = millis();
  gyro(); // Calculate the angle..
  pid_calculate(); // calculate the Control signal to reduce the error using PID Algorithm.
  Serial.print("1234\t");
  Serial.print(c_sig_m1);
  Serial.print("\t");
  Serial.print(c_sig_m2);
  Serial.print("\t");
  Serial.print(c_sig_m3);
  Serial.print("\t");
  Serial.print(c_sig_m4);
  Serial.print("\t");
/*  for(int i=1;i<4;i++)
  {
  Serial.print(gyro_ypr[i]);
  Serial.print("\t");
  }
  Serial.println();*/
  
  motor(motor_1,c_sig_m1);
  motor(motor_2,c_sig_m2);
  motor(motor_3,c_sig_m3);
  motor(motor_4,c_sig_m4);
 
  if (state == 0) // to switch off the Quadcopter since arming does take time??????
   motorarm();
  
  
   else ;
  Serial.println(millis() - looptime);
}


//---------------------------------ACTUAL MOTOR CONTROL GIVE VALUES BETWEEN 0 TO 1000 FOR THROTTLE-------------//
void motor(int motor_num,int throttle_)
{
  int throttle_time = map(throttle_,0,1000,min_throttle,max_throttle);
  timemotor = micros();
  while((micros() - timemotor) < throttle_time)
    digitalWrite(motor_num,1); 
    // No need to wait for a specific time to give zero in pulse since all the information about
    // the speed of motor in stored in the pulse width of the high signal.
    //timemotor = micros();
    //while(micros() - timemotor < 4000 -throttle_time)  
    digitalWrite(motor_num,0);
}

// Read about external interrupts on Arduino.cc
// then read about pin change interrupts.
// this basically calculates the time interval of the pulse received from the Tx(Transmitter) via the Rx(Receiver).
// all the information is hidden in the duration of the HIGH(+5 V) square wave sent to it.

//---------------------------THE INTERRUPT SERVICE ROUTINE FUNCTION FOR OBTAINING THE TRANSMITTER VALUES--------//
ISR(PCINT0_vect)
{
  current_time = micros();
  //Channel 1=========================================
  if(PINB & B00000001)
  {                                                                          //Is input 8 high?
    if(last_channel [1] == 0) 
    {                                                                        //Input 8 changed from 0 to 1.
      last_channel [1] = 1;                                                  //Remember current input state.
      timer[1] = current_time;                                               //Set timer_1 to current_time.
    }
  }
  else if(last_channel [1] == 1)
  {                                                                           //Input 8 is not high and changed from 1 to 0.
    last_channel [1] = 0;                                                     //Remember current input state.
    receiver_input[1] = current_time - timer[1];                              //Channel 1 is current_time - timer_1.

    // Thus when the pin changes from high to low. the complete time of the pulse is obtained and
    // hence what we have to do just map these time values to something more intuitive.
    // here it is the angular velocity.
    // Same goes for other channels.
    //Refer channel 3.

    if (receiver_input[1] < Tx_base_val[1][2])
      r_final[3] = map(receiver_input[1],Tx_base_val[1][2],Tx_base_val[1][3],0,70);

    else 
      r_final[3] = map(receiver_input[1],Tx_base_val[1][1],Tx_base_val[1][2],-70,0);


  }
  //Channel 2=========================================
  if(PINB & B00000010 )
  {                                                                           //Is input 9 high?
    if(last_channel [2] == 0){                                                //Input 9 changed from 0 to 1.
      last_channel [2] = 1;                                                   //Remember current input state.
      timer[2] = current_time;                                                //Set timer_2 to current_time.
    }
  }
  else if(last_channel [2] == 1)
  {                                                                           //Input 9 is not high and changed from 1 to 0.
    last_channel [2] = 0;                                                     //Remember current input state.
    receiver_input[2] = current_time - timer[2];                              //Channel 2 is current_time - timer_2.

    if (receiver_input[2] < Tx_base_val[2][2])
      r_final[2] = (-1)*map(receiver_input[2],Tx_base_val[2][2],Tx_base_val[2][3],0,70);

    else 
      r_final[2] =(-1)*map(receiver_input[2],Tx_base_val[2][1],Tx_base_val[2][2],-70,0);



  }
  //Channel 3=========================================
  if(PINB & B00000100 )
  {                                                    //Is input 10 high?
    if(last_channel [3] == 0)
    {                                                //Input 10 changed from 0 to 1.
      last_channel [3] = 1;                                                   //Remember current input state.
      timer[3] = current_time;                                               //Set timer[3] to current_time.
    }
  }
  else if(last_channel [3] == 1)
  {                                             //Input 10 is not high and changed from 1 to 0.
    last_channel [3] = 0;                                                     //Remember current input state.
    receiver_input[3] = current_time - timer[3];     //Channel 3 is current_time - timer[3].
    throttle = map(receiver_input[3],Tx_base_val[3][1],Tx_base_val[3][3],10,1000);
    // in the throttle channel we mapped it in terms of 1 to 700 (mx value is  70% of maximum throttle motor can give).

  }
  //Channel 4=========================================
  if(PINB & B00001000 )
  {                                                                          //Is input 11 high?
    if(last_channel [4] == 0)
    {                                                                        //Input 11 changed from 0 to 1.
      last_channel [4] = 1;                                                  //Remember current input state.
      timer[4] = current_time;                                               //Set timer[4] to current_time.
    }
  }
  else if(last_channel[4] == 1)
  {                                                                          //Input 11 is not high and changed from 1 to 0.
    last_channel[4] = 0;                                                     //Remember current input state.
    receiver_input[4] = current_time - timer[4];                             //Channel 4 is current_time - timer[4].

    if (receiver_input[4] <= Tx_base_val[4][2])
      r_final[1] = map(receiver_input[4],Tx_base_val[4][1],Tx_base_val[4][2],-70,0);

    else 
      r_final[1] = map(receiver_input[4],Tx_base_val[4][2],Tx_base_val[4][3],0,70);
  }

// Channel 5 ----------------------------------------------------------------------
 if(PINB & B00010000 )
  {                                                                          //Is input 12 high?
    if(last_channel [5] == 0)
    {                                                                        //Input 12 changed from 0 to 1.
      last_channel [5] = 1;                                                  //Remember current input state.
      timer[5] = current_time;                                               //Set timer[5] to current_time.
    }
  }
  else if(last_channel[5] == 1)
  {                                                                          //Input 12 is not high and changed from 1 to 0.
    last_channel[5] = 0;                                                     //Remember current input state.
    receiver_input[5] = current_time - timer[5];                             //Channel 5 is current_time - timer[5].

    if (receiver_input[5] <=1000)
     state = 0; 
    
    else if(receiver_input[5] > 1800)
      state = 1;
     }

}

//---------------------------------PID CALCULATIONS--------------------------------------------------------//
// To read about PID refer control of mobile robots course on coursera week 1 and 2. 
//---- the following function calculates the pid output for 
// The rest is just the arrayed implementation.
void pid_calculate()
{
  for(int i = 1;i<4;i++) // for yaw,pitch and roll successively.
  {
    // ERROR TO FEED INTO PID IS E = wG*GYRO_YPR - wR*r_final. they are specific weights given to the values from gyro and receiver
    // they are needed since both the values are in different scales..
    E[i] = wR*r_final[i] - wG*gyro_ypr[i]  ;
    // THE SUM OF THE ERRORS
    E_sum[i] = E_sum[i] + E[i];
    // THE DIFFERENCE OF THE CONSECUTIVE ERRORS FOR THE DERIVATIVE FUNCTION.
    E_diff[i] = E[i] - E_mem[i];
    
    // PID_OUTPUT_YPR  =  p*E + i*E_mem + d*E_diff/dt.
    
    //pid_output_ypr[i]=p_i_d__y_p_r[i][1]*E[i]+p_i_d__y_p_r[i][2]*E_sum[i]+ p_i_d__y_p_r[i][3]*E_diff[i];
    
    pid_output_ypr[i]=10*E[i]+0.0*E_sum[i]+ 0.0*E_diff[i]; // initial testing of just p algorithm..
    
    // SAVING THE USED ERROR VALUE IN MEMORY FOR THE DERIVATIVE CALCULATIONS..
    E_mem[i] = E[i];  

  }
 
  // the following code will calculate the controlsignal from 0 to 1000 in terms of throttle..to each motor.
  // for the motor naming vs placement conventions refer to the following diagram.
  //      3    2ccw
  //       \  /
  //        \/
  //        /\
  //       /  \
  //      4    1 cw


  // verify yaw..signs...
  // throttle signal has value from 1 to 700 i.e 70% of maximum throttle...
  // recangle[3]is the throtte values from the transmitter..
  c_sig_m1 = throttle - pid_output_ypr[3] + pid_output_ypr[2] - pid_output_ypr[1];
  c_sig_m2 = throttle - pid_output_ypr[3] - pid_output_ypr[2] + pid_output_ypr[1];  
  c_sig_m3 = throttle + pid_output_ypr[3] - pid_output_ypr[2] - pid_output_ypr[1];
  c_sig_m4 = throttle + pid_output_ypr[3] + pid_output_ypr[2] + pid_output_ypr[1];
  // the control signals are constrained between Low,High .
  // constrained control signal =  constrain(control signal,LOW, HIGH) between LOW and HIGH
 
  c_sig_m1 = constrain(c_sig_m1,0,1000); 
  c_sig_m2 = constrain(c_sig_m2,0,1000);
  c_sig_m3 = constrain(c_sig_m3,0,1000);
  c_sig_m4 = constrain(c_sig_m4,0,1000);
} 

void gyro()
{
  getraw();  //obtaining raw values
             // Subtracting the offset..
  gy= gy- gy_avg;
  gx= gx -gx_avg;
  gz = gz -gz_avg;
  lowpass(0.99);
 gyro_ypr[1] = gz;
 gyro_ypr[2] = gx;
 gyro_ypr[3] = gy;
  
/* Serial.print(gx);
 Serial.print(" ");
 Serial.print(gy); 
 Serial.println(" ");
*/
  
}



  
  



/* 
The raw values don't mean anything !!
Actually they can't make any sense to you unless they are divided by the sensitivity.
So for them to make any sense we at first need to choose our sensitivity scale in our case it will be 
+ 500 dps to -500 dps. (dps -- degrees per second)

so the raw values must be divided by 65.5 meaning the 
in the sensitivity we chose 65.5 is the output if the gyro is moving 1dps.


Similarly for the accelerometer  we chose the full scale range of +-4g hence the dividend
is 8192.

Refer the setup function to refer the sensitivity.
*/

/*
MPU6050 3 axis + gyro uses the I2c communication protocol. I2c stands for inter-integrated circuit.
Here the mpu6050 acts as a slave device but it can also act as a master for linking it with the magnetometer.
Please refer TWI on arduino.cc 
Also read the product specifications and datasheet of the mpu6050 module to get the best of it!!
*/

void lowpass(float lp_old)
{
  gx = lp_old*gx_m + (1-lp_old)*gx;
  gx_m = gx;

  gy = lp_old*gy_m + (1-lp_old)*gy;
  gy_m = gy;

  gz = lp_old*gz_m + (1-lp_old)*gz;
  gz_m = gz;

}


void getraw()
{
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x43);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,6,true);  // request a total of 14 registers
  
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  gx = GyX/g_sense;
  gy = GyY/g_sense;
  gz = GyZ/g_sense;

}
void calib()
{
  int i =0;
  while(i++<2000)
    {
      getraw();

gx_avg += gx;
gy_avg += gy;
gz_avg += gz;
delay(3);
    }
  
  gx_avg = gx_avg/2000;
  gy_avg = gy_avg/2000;
  gz_avg = gz_avg/2000;
    
}

// ----------------------------------SETUP FUNCTIONS----------------------------------------------------------//
//-----------------------------------PIN MODE INITIALISE FUNCTION---------------------------------------------//
void pinModeinit()
{
  for(int i = 4,j = 8;i<8;i++,j++)
  {
    pinMode(i,OUTPUT);
    pinMode(j,INPUT);
  }
  pinMode(13,1);

  PCICR  |= (1 << PCIE0);                    //Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= (1 << PCINT0);                   //Set PCINT0 (digital input 8) to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT1);                   //Set PCINT1 (digital input 9)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT2);                   //Set PCINT2 (digital input 10)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT3);                   //Set PCINT3 (digital input 11)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT4);                   //Set PCINT4 (i.e digital input 12) to trigger and interrupt on change.
}

//------------------------------------ARMING THE MOTOR IN SETUP FUNCTION---------------------------------------//
void motorarm()
{
  for(int j = 4;j<8;j++)
  {
    for(int i = 0; i<500;i++)
    {
      timemotor = micros();
      while((micros() - timemotor) < zero_throttle)
        digitalWrite(j,1); 
     // timemotor = micros();
     // while(micros() - timemotor < 100)  to save arming time 
        digitalWrite(j,0);
    }
  }
}
  
 

void magnetometer()
{
  
  
  
}






