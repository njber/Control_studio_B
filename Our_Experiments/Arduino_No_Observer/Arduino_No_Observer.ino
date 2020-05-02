/***************************************************************
 Control Studio B
 Arduino MKR board as Processor-In-the-Loop for Matlab/Simulink
 
 Dr Ricardo P. Aguilera
 University of Technology Sydney
 Sydnney, Australia
 April 2020

***************************************************************/

/*****************************
  Libraries
******************************/
#include <BasicLinearAlgebra.h>

/*****************************
  Static Variables  (Constants)
******************************/
//#define baud_rate 9600  //bits per second
#define baud_rate 115200  //bits per second

//Number of System States, Inputs, and Outputs
#define n 6   //number of states 
#define m 2   //number of inputs 
#define p 2   //number of outputs
#define G 300
#define G2 10000

/*****************************
   Global Variables
******************************/
byte data_in_byte[22]; // for incoming serial data in bytes (8 bits)
byte data_out_byte[22]; // for outgoing serial data in bytes (8 bits)
byte PIL_start = 0;

// Inputs to the board
float in1 = 0;
float in2 = 0;
float in3 = 0;
float in4 = 0;

float extra_in1 = 0;
float extra_in2 = 0;
float extra_in3 = 0;
float extra_in4 = 0;
float extra_in5 = 0;
float extra_in6 = 0;

// Outputs from the board
float out1 = 0;
float out2 = 0;
float out3 = 0;
float out4 = 0;

float extra_out1 = 0;
float extra_out2 = 0;
float extra_out3 = 0;
float extra_out4 = 0;
float extra_out5 = 0;
float extra_out6 = 0;

//Control Targets
float  y_star_w = 100;
float  y_star_x = 0.02;


// System Variables
BLA::Matrix<n,1> x_hat;  // u(k) \in R^n
BLA::Matrix<m,1> u_k;  // u(k) \in R^m
BLA::Matrix<p,1> y_k;
BLA::Matrix<n,1> x_k;

// State Feedback Controller
BLA::Matrix<m,n> F;
BLA::Matrix<n,1> xss;
BLA::Matrix<m,1> uss;

// Discretised Plant
BLA::Matrix<n,n> A;
BLA::Matrix<n,m> B;
BLA::Matrix<p,n> C;

//___________________________________________________________________________
//                             Setup
//          
//            Complete desired initializations on startup
//___________________________________________________________________________
void setup() {
  // put your setup code here, to run once:
  Serial.begin(baud_rate); // opens serial port, sets data rate

  // State Feedback Gain
  F <<   -0.0006,   -0.0645,   -9.0794,    2.0784,   -1.9243, -406.0190,
    0.0441,   -0.1092,   -1.3773,    2.1541,   -2.0004, -401.9299;


  A << 0.6371,    0.0005,    0.0112,   -0.2716,    0.2716,    5.2159,
    0.0005,    0.6371,   -0.0112,    0.2716,   -0.2716,   -5.2159,
    0.0000,   -0.0000,    0.9880,    0.0148,   -0.0148,   -3.1272,
    0.0032,    0.0000,    0.0000,    0.9994,    0.0006,    0.0112,
    0.0000,    0.0032,   -0.0000,    0.0006,    0.9994,   -0.0112,
    0.0000,   -0.0000,    0.0040,    0.0000,   -0.0000,    0.9937;

  B << 4.0255,    0.0009,
    0.0009,    4.0255,
    0.0000,   -0.0000,
    0.0087,    0.0000,
    0.0000,    0.0087,
    0.0000,   -0.000;

  C << 0.5000,    0.5000,         0,         0,         0,         0,
         0,         0,         0,         0,         0,   -1.0000;


 //Steady state calcs
  xss << y_star_w, y_star_w, 0, 0, 0, y_star_x;
  uss << (112.5*y_star_w - 1623.8*y_star_x)/1250, (112.5*y_star_w + 1623.8*y_star_x)/1250;
}

//___________________________________________________________________________
//                        Main loop
//
//              Wait until Simulink send data
//___________________________________________________________________________
void loop() {
  if (Serial.available() > 0) {
//When data is received from Simulink, code jumps to the Controller Section 
      Controller();
    }
}


//___________________________________________________________________________
//                              Controller
//          
//    The code inside this section will be VIRTUALLY run at every Ts
//    This will depends on the time Simulink takes to simulate your model
//    and the time required to interchange data between Simulink and Arduino
//___________________________________________________________________________
void Controller() {
  // Receive from Simulink
  // in1, in2, in3, in4 
  // extra_in1, extra_in2, extra_in3, extra_in4 
  read_Simulink(); 
  

  
  // Associate board inputs to states
  x_k(0)=in1;
  x_k(1)=in2;
  x_k(2)=in3;
  x_k(3)=in4;
  x_k(4)=extra_in1;
  x_k(5)=extra_in2;



  //State Feedback Controller
  u_k = (-F*x_k + (F*xss + uss))/G;
  

  // Associate system control inputs u(k) to board outputs
  out1 = saturate(u_k(0),-10,10);
  out2 = saturate(u_k(1),-10,10);
  out2 = 2;
  out3 = 3;
  out4 = 4;

  extra_out1 = 1.1234;
    
  // Send out1, out2, out3, out4 to Simulink
  write_Simulink(); 
}
// End of the Control Algorithm




//___________________________________________________________________________
//
//                Communication Functios
//
//    Make possible to interchange data between Simulink and Arduino
//___________________________________________________________________________

//Receive data from Simulink
void read_Simulink() {
  byte Header;
  float data_in[10];
  unsigned int data_in_int;
  for (int i = 0; i<22; i++){
    data_in_byte[i] = Serial.read();
  }
  
  Header=data_in_byte[0];

  if (Header==0xAA){ 
    for (int i = 0; i<10; i++){
      data_in_int =  (data_in_byte[2*i+1]<<8)+data_in_byte[2*i+2];
      data_in[i] = (float)(data_in_int*0.001)-10;
    }
    in1=data_in[0]*G2;
    in2=data_in[1]*G2;
    in3=data_in[2]*G2;
    in4=data_in[3]*G2;

    extra_in1=data_in[4]*G2;
    extra_in2=data_in[5]*G2;
    extra_in3=data_in[6];
    extra_in4=data_in[7];
    extra_in5=data_in[8];
    extra_in6=data_in[9];

    PIL_start = data_in_byte[21];
    
  }
  
}

//Send data to Simulink
void write_Simulink() {
  byte Header = 0x5A;  //0x5A = 90 in decimal and "Z" in ASCII
  byte LSB, MSB;
  float out[10], out_pos;
  unsigned int out_int;

  if (PIL_start==0)
    clear_all_outputs();
  
  out[0] = out1;
  out[1] = out2;
  out[2] = out3;
  out[3] = out4;

  out[4] = extra_out1;
  out[5] = extra_out2;
  out[6] = extra_out3;
  out[7] = extra_out4;
  out[8] = extra_out5;
  out[9] = extra_out6;

  for (int i = 0; i<10; i++){
    if (out[i]<-10)
      out[i]=-10;
    if (out[i]>10)
      out[i]=10;
  }
  
  data_out_byte[0]=Header;
  data_out_byte[21]=PIL_start;
  
  for (int i = 0; i<10; i++){
    out_pos=out[i]+10;     
    out_int = 1000*(out_pos);
    LSB=out_int;
    MSB=(out_int >>8);

    data_out_byte[2*i+1]=MSB;
    data_out_byte[2*i+2]=LSB;
  }
  
  for (int i = 0; i<22; i++){
    Serial.write(data_out_byte[i]);
  }
}

//Saturation funciton to add limits to a variable
float saturate(float nosat, float lb, float ub){
  //nosat:  original variable; 
  //lb: lower bound; ub: upper bound
  float sat, aux;
  if (ub<lb){//To ensure that ub > lb
    aux=lb;
    lb=ub;
    ub=aux;
  }

  sat = min(nosat,ub);
  sat = max(sat,lb);
  return sat;
}
void clear_all_outputs() {
  out1=0;
  out2=0;
  out3=0;
  out4=0;
}
