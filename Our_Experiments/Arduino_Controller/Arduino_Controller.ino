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
#define n 2   //number of states 
#define m 1   //number of inputs 
#define p 1   //number of outputs

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
BLA::Matrix<n,1> x_k;  // u(k) \in R^n
BLA::Matrix<m,1> u_k;  // u(k) \in R^m

// State Feedback Controller
BLA::Matrix<m,n> K;


//___________________________________________________________________________
//                             Setup
//          
//            Complete desired initializations on startup
//___________________________________________________________________________
void setup() {
  // put your setup code here, to run once:
  Serial.begin(baud_rate); // opens serial port, sets data rate

  // State Feedback Gain
  K << 23.9539,   52.1018; 
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
  x_k(0)=in1;
  x_k(1)=in2;

  //State Feedback Controller
  u_k = -K*x_k;

  // Associate board outputs to system inputs to be applied
  out1 = u_k(0);
  out1 = saturate(out1,-10,10);

  out2 = 2;
  out3 = 3;
  out4 = 4;

  // Send out1, out2, out3, out4 to Simulink
  write_Simulink(); 
}


// Communication Functions
//___________________________________________________________________________
//
//                Communication Functios
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
