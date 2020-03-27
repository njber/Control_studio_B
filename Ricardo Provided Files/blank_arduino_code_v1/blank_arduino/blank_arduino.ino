
/*********************************************************
 Base Arduino Code for implementing a Digital Controller
 University of Technology Sydney (UTS)
 Board: MKR1000
 Based: bAC18
 Code:  v1

 Created by Ricardo P. Aguiilera, 
            Manh (Danny) Duong Phung,

 Amended by Ellis Tsekouras

 Date: 31/07/2019
 

 Hardward tools: Extended Arduino Board for MKR1000
 Software tools: It requires the following libraries:
    MatrixMath
    PWM_MKR1000_AdvCtrl_UTS
 *******************************************************/

#include <Timer5.h>
#include <PWM_MKR1000_AdvCtrl_UTS.h>


#define fs 200 //Sampling frequency [Hz] , Ts = 1/fs [s]

// Control Effort
float u_k;    // u[k]
float u_k1;   // u[k-1]

// Plant Outputs
float y1_k;   // y1[k]
float y1_k1;  // y1[k-1]

float y2_k;   // y2[k]
float y2_k1;  // y1[k-1]

// Control Error
float e_k;    // e[k]
float e_k1;   // e[k-1]

// Controller Variables
float a1;
float a2;
float a3;
float b1;
float b2;
float b3;

float z_k;    // z[k]
float z_k1;   // z[k-2]  


// uC Inputs
float in1 = 0;
float in2 = 0;
float in3 = 0;
float in4 = 0;

// uC Outputs
float out1 = 0;
float out2 = 0;
float out3 = 0;
float out4 = 0;


//___________________________________________________________________________
//
//                                  setup
//          
//            Complete desired initializations on startup
//___________________________________________________________________________
void setup()
{  
  //Initialize Serial Bus
    Serial.begin(9600); 

  //Sampling Time Ts=1/fs [s]
    float Ts = 1/float(fs); //in seconds      
  
    
  // Initialize controller variables
    a1 = 0;
    a2 = 0;
    a3 = 0;
    b1 = 0;
    b2 = 0;
    b3 = 0;

    
  //Initialize I/O pins to measure execution time
    pinMode(LED_BUILTIN,OUTPUT);
    pinMode(A3,OUTPUT);
    digitalWrite(A3,LOW); 

  //ADC Resolution                                                      \
    The Due, Zero and MKR Family boards have 12-bit ADC capabilities    \
    that can be accessed by changing the resolution to 12.              \
    12 bits will return values from analogRead() between 0 and 4095.    \
    11 bits will return values from analogRead() between 0 and 2047.    \
    10 bits will return values from analogRead() between 0 and 1023.    \
    Default resolution if not used is 10bits.

    int res = 12;
    analogReadResolution(res); //If commented, default resolution is 10bits

   
  // Configure PWM for outputs
    init_PWM_MKR1000_UTS();
  
  // define timer5 interruption freq (sampling time)
    MyTimer5.begin(fs);   //Argument is freq in Hz

  // define the interrupt callback function
    MyTimer5.attachInterrupt(Controller);
  
  // start the timer
    MyTimer5.start();   //Always start timer at the end
    
}

//___________________________________________________________________________
//
//                              Controller
//          
//            The code inside this section will be run at every Ts
//            This is the timer ISR (Interrupt Service Routine).
//___________________________________________________________________________
void Controller(void)
{

// The code inside this section will be run at every Ts


  //Start measuring execution time
  digitalWrite(A3,HIGH);  

/*
Board Inputs
*/
    //It is possible to adjust offset and gain of
    //the measurements. Also, disp = 1 will display
    //individual input in serial monitor
    
    //read_inputx(offset, gain, disp)
    in1 = read_input1(0, 1, 0);  // 0 -> 12v
    in2 = read_input2(0, 1, 0);  // 0 -> 12v
    in3 = read_input3(0, 1, 0);  // -12v -> 12v
    in4 = read_input4(0, 1, 0);  // -12v -> 12v

    disp_inputs_all();

    // Only display inputs for calibration. 
    // Do not display them when running the controller

  

// Discrete-time Controller

/* EXAMPLE - YOU MUST CHANGE */

    // Output Measurement
    y1_k = in1;
    y2_k = in2;

    // Control Algorithim

    u_k = a1*y1_k - b2*y2_k1;

    // Map Contol Effort to output

    out1 = u_k;

    // Store variables for next iteration
    y1_k1 = y1_k;


/*
Board Outputs
*/
    //It is possible to adjust offset and gain of
    //each output. Also, disp = 1 will display
    //individual output in serial monitor
     
  //write_outx(value, offset, gain, disp) 
    write_out1(out1,  0,  1, 0);      // Pin 5  -12V to 12V
    write_out2(out2,  0,  1, 0);      // Pin 4  -12V to 12V
    write_out3(out3,  0,  1, 0);      // Pin 7    0V to 12V
    write_out4(out4,  0,  1, 0);      // Pin 6    0V to 12V

    // disp_outputs_all();
    // Only display outputs for calibration. 
    // Do not display them when running the controller   

//Stop measuring calculation time
  digitalWrite(A3,LOW);   


}

//___________________________________________________________________________
//
//                                  loop
//
//              Does nothin - main loop left intentionally empty!
//___________________________________________________________________________
void loop()
{ 


}

//___________________________________________________________________________
//
//                              disp_inputs_all
//
//                Displays all inputs on serial monitor
//___________________________________________________________________________
 void disp_inputs_all()
{
      Serial.print("In1: ");
      Serial.print(in1,3);
      Serial.print(" [V]  ");
      Serial.print("In2: ");
      Serial.print(in2,3);
      Serial.print(" [V]  ");
      Serial.print("In3: ");
      Serial.print(in3,3);
      Serial.print(" [V]  ");
      Serial.print("In4: ");
      Serial.print(in4,3);
      Serial.println(" [V]");
}



//___________________________________________________________________________
//                              disp_outputs_all
//
//              Displays all outputs on serial monitor
//___________________________________________________________________________
 void disp_outputs_all()
{
      Serial.print("Out1: ");
      Serial.print(out1);
      Serial.print(" [V]  ");
      Serial.print("Out2: ");
      Serial.print(out2);
      Serial.print(" [V]  ");
      Serial.print("Out3: ");
      Serial.print(out3);
      Serial.print(" [V]  ");
      Serial.print("Out4: ");
      Serial.print(out4);
      Serial.println(" [V]");
}



//___________________________________________________________________________
//                              read_inputx
//
//        Reads respective analog pin and returns an actual voltage.
//        The raw bits must be scaled!
//___________________________________________________________________________

float read_input1(float offset, float gain, int disp)
{
  float in_float = 0;
  int in = analogRead(A1); // Read raw input 1 (0 –> 12V)
  
  in_float = (float)(in);

  //  Here, you must enter the experimental linear relationship
  //  between the raw bits and the actual voltage.
  //  acutal voltage = in_float*m +/- 
  float p1 = 0;
  float p2 = 0;
  in_float = in_float*p1 + p2;

  in_float = (in_float + offset)*gain;

  if (disp == 1)
  {
    Serial.print("In1: ");
    Serial.print(in1);
    Serial.println(" [V]");
  }
  
  return in_float; 
}


float read_input2(float offset, float gain, int disp)
{
  float in_float = 0;
  int in = analogRead(A2); // Read input 2 (0 –> 12V)
  
  in_float = (float)(in);

  //  Here, you must enter the experimental linear relationship
  //  between the raw bits and the actual voltage.
  float p1 = 0;
  float p2 = 0;
  in_float = in_float*p1 + p2;

  in_float = (in_float + offset)*gain;

    if (disp == 1)
    {
      Serial.print("In2: ");
      Serial.print(in2);
      Serial.println(" [V]");
    }

  return in_float; 
}


float read_input3(float offset, float gain, int disp)
{
  float in_float = 0;
  int in = analogRead(A5); // Read input 3 (0 -> +/-12)

  in_float = (float)(in);

  //  Here, you must enter the experimental linear relationship
  //  between the raw bits and the actual voltage.
  float p1 =  0;
  float p2 =  0;
  in_float = in_float*p1 + p2;

  in_float = (in_float+ offset)*gain;
  
  if (disp == 1)
  {
    Serial.print("In3: ");
    Serial.print(in3);
    Serial.println(" [V]");
  }

  return in_float; 
}


float read_input4(float offset, float gain, int disp) {
  float in_float = 0;
  int in = analogRead(A6); // Read input 4 (0 -> +/-12)

  in_float = (float)(in);

  //  Here, you must enter the experimental linear relationship
  //  between the raw bits and the actual voltage.
  float p1 =  0;
  float p2 =  0;
  in_float = in_float*p1 + p2;

  in_float = (in_float + offset)*gain;

    if (disp == 1) {
      Serial.print("In4: ");
      Serial.print(in4);
      Serial.println(" [V]");
  }
  
 return in_float;
}

//___________________________________________________________________________
//                              write_outx
//
//        Given a desired output value, it will scale this to an output
//        value for PWM generation.
//        The raw bits must be scaled!
//___________________________________________________________________________
void write_out1(float out, float offset, float gain, int disp)
{
  float d;  // Pin 4

  //  Here, you must enter the experimental linear relationship
  //  between the requested and actual voltage.
  float p1 = 0;
  float p2 = 0;
  d = out*p1 + p2;

  d = (d + offset)*gain;

  if (d < 0)
  {
    d = 0;
  }

  REG_TCC0_CCB2 = (int)((REG_TCC0_PER + 1)*d); 
    
  if (disp==1)
  {
    Serial.print("Out2 : ");
    Serial.print(out2);
    Serial.println(" [V]");
  }

}


void write_out2(float out, float offset, float gain, int disp)
{
  float d = 0;  // Pin 5

  //  Here, you must enter the experimental linear relationship
  //  between the requested and actual voltage.
  float p1 = 0;
  float p2 = 0;
  d = out*p1 + p2;

  d = (d + offset)*gain;

  if (d < 0)
  {
    d = 0;
  }

  REG_TCC0_CCB3= (int)( (REG_TCC0_PER + 1)*d); 

  if (disp == 1)
  {
    Serial.print("Out1 : ");
    Serial.print(out1);
    Serial.println(" [V]");
  }
}


void write_out3(float out, float offset, float gain, int disp)
{
  float d;  // Pin 6

  //  Here, you must enter the experimental linear relationship
  //  between the requested and actual voltage.
  float p1 = 0;
  float p2 = 0;
  d = out*p1 + p2;

  d = (d + offset)*gain;

  if (d < 0)
  {
    d = 0;
  }

  REG_TCC0_CCB0 = (int)((REG_TCC0_PER+1)*d); 
    
  if (disp == 1)
  {
    Serial.print("Out4 : ");
    Serial.print(out4);
    Serial.println(" [V]");
  }
 
}

void write_out4(float out, float offset, float gain, int disp)
{
  float d;  // Pin 7

  //  Here, you must enter the experimental linear relationship
  //  between the requested and actual voltage.
  //  d = in_out*m +/- b
  float p1 = 0;
  float p2 = 0;
  d = out*p1 + p2;

  d = (d + offset)*gain;

  if (d < 0)
  {
    d = 0;
  }

  REG_TCC0_CCB1 = (int)((REG_TCC0_PER+1)*d);

  if (disp == 1)
  {
    Serial.print("Out3 : ");
    Serial.print(out3);
    Serial.println(" [V]");
  }

}


/*PWM Explanation
The sawtooth carrier incrases from 0 to Tmax

Tmax = REG_TCC0_PER + 1

Tmax    _____________________
             /|     /|     /|
            / |    / |----/-|
CMP_i   ---/--|   /  |   /  |
          /   |  /   |  /   |
         /    |-/----| /    |
    0   /     |/     |/     └───────> t
       |      |      |      |
       |      |      |      |
PWM i Output  |      |      |
       ┌──┐   ┌┐     ┌────┐ 
       │  │   ││     │    │ 
       │  │   ││     │    │ 
       ┘  └───┘└─────┘    └───────> t
*/
