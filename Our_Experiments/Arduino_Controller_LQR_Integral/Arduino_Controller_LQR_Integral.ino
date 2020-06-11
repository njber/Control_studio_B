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

// Steady state value calculation matrixes
BLA::Matrix<m,1> r; //To DO; make changeable

// Torque to voltage conversion

float G = 30;


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

// r calculation
BLA::Matrix<m,1> star;
BLA::Matrix<n,1> xss;
BLA::Matrix<m,1> uss;
BLA::Matrix<m,m> Nu;
BLA::Matrix<n,m> Nx;


// Integral action
BLA::Matrix<2,1> q_k;
BLA::Matrix<2,2> Ki;

//___________________________________________________________________________
//                             Setup
//          
//            Complete desired initializations on startup
//___________________________________________________________________________
void setup() {
  // put your setup code here, to run once:
  Serial.begin(baud_rate); // opens serial port, sets data rate

  star <<  80,   0.02;

  F <<     -59.244944370242045, -34.626618572718435,   6.886680576936058,   7.522078676428836,   1.516555125506129,   2.056436423830827,
  59.345515914010051,  34.418157924708495,  -6.852381145881474,  -7.526283282759932,  -1.522183583088243,  -2.059339179918633;


  L <<           0.000001243594173,  12.743821721145618,
  -0.000003805161388,   6.608918897791039,
  -0.000000898814646,  -0.863506047629504,
  -0.000000375758287,  -9.082622717986967,
  -0.000000666406156,  -6.049789893837215,
   0.000000093600464,  -0.241481242953477;


  q_k << 0, 0;

  Ki << -0.001923905105586,  -3.877007921026134,
   0.004592976459198,   3.877007095763732;

   Ki = Ki * 100;

  Nu << 0.090027961915934, -13.022536982629868,
   0.089972039437138,  13.022530279489462;

  Nx << 0.000951909680044,   0.067192702289631,
   0.000684684302741,   0.226118697794490,
   0.011590401308861,   1.172461453699500,
   0.000449445255857,   0.170099361738449,
  -0.000155802008671,  -0.103642582507235,
  -0.001474126485304,  -0.101787047900273;

   Nx = Nx * 1000;

  x_hat_k << 0, 0, 0, 0, 0, 0;

  A << 0.828490331527960,   0.296441015569965,  -0.051834879366315,  -0.025622003777837,   0.041592965959481,  -0.026584523527247,
   0.231089081296047,   0.495671734801444,   0.079026480209750,  -0.021202444617181,   0.004107381358831,  -0.010789223600151,
  -0.073917292717985,   0.157746377502415,   0.950861405942210,   0.032067563574971,   0.028724791694483,   0.005091138798166,
  -0.117423900182753,  -0.070132797514398,  -0.018649781324601,   0.545452912865091,   0.230828872722433,  -0.336883997627721,
   0.076862597352242,   0.035157719668214,  -0.028341727564894,   0.309667401639336,   0.785079863414509,  -0.099725094403509,
  -0.020938390517172,  -0.042435840631523,  -0.001598688701669,   0.094523947709141,   0.260256407825795,   0.920834617385560;

  B << 2.568297816786385,   3.430717438337842,
  -4.761540729042112,  -4.086630749106535,
   1.983745236169887,   3.899737662364028,
  -3.656168987026917,   4.987768181646191,
   2.003104141397267,  -2.987067031678157,
  -0.887144193525567,   0.319534624930044;

  C <<  0.284296065909090,  -0.589150784937325,   0.096936042901401,  -0.011889341568374,  -0.015908514533567,  -0.008205203895977,
   0.021775733904432,   0.012767345044936,  -0.002700886358837,  -0.002689978349243,  -0.000689148429575,  -0.001991587209107;


  uss = Nu * star;
  xss = Nx * star;


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
  r = -Ki*q_k;
  //State Feedback Controller
  u_k = (-F*x_hat + r);

  //Update estimated states
  x_hat_k = A*x_hat +B*u_k + L*(y_k - C*x_hat);
  
  u_k = u_k/G;

  q_k = q_k + (y_k-star);
  
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
