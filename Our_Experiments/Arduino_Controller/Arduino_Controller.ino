/***************************************************************
 Control Studio
 Arduino MKR board as Processor-In-the-Loop for Matlab/Simulink
 
 Dr Ricardo P. Aguilera
 University of Technology Sydney
 Sydnney, Australia
 March 2020

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

/*****************************
   Global Variables
******************************/
byte data_in_byte[20]; // for incoming serial data in bytes (8 bits)
byte data_out_byte[20]; // for outgoing serial data in bytes (8 bits)

// Inputs to the board
float in1 = 0;
float in2 = 0;
float in3 = 0;
float in4 = 0;


// Outputs from the board
float out1 = 0;
float out2 = 0;
float out3 = 0;
float out4 = 0;


// System Variables
BLA::Matrix<n,1> x_hat;  // u(k) \in R^n
BLA::Matrix<m,1> u_k;  // u(k) \in R^m
BLA::Matrix<p,1> y_k;

// State Feedback Controller
BLA::Matrix<m,n> F;
BLA::Matrix<n,p> L;

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
  F <<   -31.1527,  -16.5069,   31.3074,   16.7928,  -40.1742,  -34.0344,
  -51.6751,  -17.0642,   51.8748,   17.3504, -129.0792,  -35.1621;

  L <<        -1.2544,    0.0899,
   -0.4060,    1.8499,
   -0.8071,   -0.0587,
    0.6432,   -1.8495,
   -0.3119,    0.2452,
   -5.2473,    3.3310;

  A << 1.0000,    0.0100,   -0.0000,   -0.0000,    0.0080,    0.0000,
      -0.0020,    0.9939,   -0.0033,   -0.0000,    1.6073,    0.0080,
       0.0000,    0.0000,    1.0000,    0.0100,   -0.0080,   -0.0000,
      -0.0003,    0.0000,   -0.0020,    0.9939,   -1.6073,   -0.0080,
       0.0034,    0.0000,   -0.0034,   -0.0000,    1.0068,    0.0100,
       0.6702,    0.0033,   -0.6702,   -0.0033,    1.3621,    0.9926;

  B <<     0.0003,   -0.0000,
    0.0680,   -0.0000,
    0.0000,    0.0003,
    0.0000,    0.0680,
    0.0000,   -0.0000,
    0.0001,   -0.0001;

  C << 0,    0.5000,         0,    0.5000,         0,         0,
       0,         0,         0,         0,    1.0000,         0;
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
  // Receive in1, in2, in3, in4 from Simulink
  read_Simulink();  
  
  // Associate board inputs to states
//  x_k(1)=in1;
//  x_k(3)=in2;
//  x_k(4)=in3;
//  x_k(5)=in4;
  // Go off just the outputs
  y_k(0) = in1; // pully speed
  y_k(1) = in2; // tension

  static BLA::Matrix<n,1> x_hat_k; //TODO: maybe initialise
  x_hat = x_hat_k;
  

  //State Feedback Controller
  //u_k = -F*x_hat + 0.1; //TODO: add r
  // Open Loop
  u_k << 1/300, 1.5/300;

  //Update estimated states
  x_hat_k = A*x_hat +B*u_k + L*(y_k - C*x_hat);

  // Associate board outputs to system inputs to be applied

  out1 = saturate(u_k(0),-10,10);
  out2 = saturate(u_k(1),-10,10);

  out3 = 3;
  out4 = 4;

  // Send out1, out2, out3, out4 to Simulink
  write_Simulink(); 
}


// Communication Functions
//___________________________________________________________________________
//
//                Communication Functions
//
//    Make possible to interchange data between Simulink and Arduino
//___________________________________________________________________________

//Receive data from Simulink
void read_Simulink() {
  byte Header;
  float in[4];
  unsigned int in_int;
  for (int i = 0; i<9; i++){
    data_in_byte[i] = Serial.read();
  }
  
  Header=data_in_byte[0];

  if (Header==0xAA){ 
    for (int i = 0; i<4; i++){
      in_int =  (data_in_byte[2*i+1]<<8)+data_in_byte[2*i+2];
      in[i] = (float)(in_int*0.001)-10;
    }
    in1=in[0];
    in2=in[1];
    in3=in[2];
    in4=in[3];
  }
  
}

//Send data to Simulink
void write_Simulink() {
  byte Header = 0x5A;  //0x5A = 90 in decimal and "Z" in ASCII
  byte LSB, MSB;
  float out[4], out_pos;
  unsigned int out_int;
  
  out[0] = out1;
  out[1] = out2;
  out[2] = out3;
  out[3] = out4;
  
  data_out_byte[0]=Header;
  
  for (int i = 0; i<4; i++){
    out_pos=out[i]+10;     
    out_int = 1000*(out_pos);
    LSB=out_int;
    MSB=(out_int >>8);

    data_out_byte[2*i+1]=MSB;
    data_out_byte[2*i+2]=LSB;
  }
  
  for (int i = 0; i<9; i++){
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
