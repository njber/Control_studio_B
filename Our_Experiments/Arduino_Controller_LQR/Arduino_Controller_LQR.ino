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

// Steady state values
BLA::Matrix<m,1> r; //To DO; make changeable

// Torque to voltage conversion

float G = 300;


// System Variables
BLA::Matrix<n,1> x_hat;  // u(k) \in R^n
BLA::Matrix<m,1> u_k;  // u(k) \in R^m
BLA::Matrix<p,1> y_k;

BLA::Matrix<n,1> x_hat_k; //TODO: maybe initialise

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
  F <<  0.0088,   -0.0193,    0.0032,   -0.0004,   -0.0005,   -0.0002,
        0.0096,   -0.0188,    0.0031,   -0.0004,   -0.0005,   -0.0003;

  L <<        0.0428,    0.8223,
   -0.2583,    0.3972,
   -0.0538,   -0.0187,
    0.0004,   -0.1689,
   -0.0251,    0.1068,
    0.0104,   -0.1257;

  L = L * 0.000001;

  r << 11.9694,
  12.4853;

  x_hat_k << 0, 0, 0, 0, 0, 0;

  A << 0.8285,    0.2964,   -0.0518,   -0.0256,    0.0416,   -0.0266,
    0.2311,    0.4957,    0.0790,   -0.0212,    0.0041,   -0.0108,
   -0.0739,    0.1577,    0.9509,    0.0321,    0.0287,    0.0051,
   -0.1174,   -0.0701,   -0.0186,    0.5455,    0.2308,   -0.3369,
    0.0769,    0.0352,   -0.0283,    0.3097,    0.7851,   -0.0997,
   -0.0209,   -0.0424,   -0.0016,    0.0945,    0.2603,    0.9208;

  B << 2.5683,    3.4307,
      -4.7615,   -4.0866,
       1.9837,    3.8997,
      -3.6562,    4.9878,
       2.0031,   -2.9871,
      -0.8871,    0.3195;

  C <<  0.2843,   -0.5892,    0.0969,   -0.0119,   -0.0159,   -0.0082,
        0.0218,    0.0128,   -0.0027,   -0.0027,   -0.0007,   -0.0020;
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


  BLA::Matrix<n,1> x_k;
  // Associate board inputs to states
//  x_k(0)=in1;
//  x_k(1)=in2;
//  x_k(4)=in3;
//  x_k(5)=in4;
  // Go off just the outputs
  y_k(0) = in1; // pully speed
  y_k(1) = in2; // tension

  y_k(0) = y_k(0) * 100;
  y_k(1) = y_k(1) / 100;

  x_hat = x_hat_k;
  

  //TODO impliment r here in future

  //State Feedback Controller
  u_k = (-F*x_hat + r);
//  u_k(0) = saturate(u_k(0),-10,10);
//  u_k(1) = saturate(u_k(1),-10,10);

  //Update estimated states
  x_hat_k = A*x_hat +B*u_k + L*(y_k - C*x_hat);
  
  u_k = u_k/G;
  
  // Associate board outputs to system inputs to be applied

  out1 = saturate(u_k(0),-10,10);
  out2 = saturate(u_k(1),-10,10);
  // out1 = u_k(0);
  // out2 = u_k(1);

  out3 = 4;
  out4 = 5;

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
