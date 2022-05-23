

/////////////////////////////////////
// Measuring velocity from Infrared (IR) sensor and Magnetic (M) sensor) using a constant sampling time
/////////////////////////////////////
// Integer calculations are used due to slower performance on floating points
// The velocity send over serial is multiplied by 1000 for integer purposes
/////////////////////////////////////

/////////////////////////////////////
// Settings that can be changed

// Changing the constant below determines the time between velocity measurements
const double dt = 0.05;

/////////////////////////////////////
// Variables for the program

// Creating global variables for the detection pins, these need to be volatile
volatile unsigned long i_IR = 0; 
volatile unsigned long i_M = 0;
volatile unsigned long IR_LastTimeWeMeasured = micros();
volatile unsigned long M_LastTimeWeMeasured = micros();
volatile unsigned long IR_SumPeriods = 0;
volatile unsigned long M_SumPeriods = 0;


// Variables for storing the incremented detection values, so they are not changed during calculations
unsigned long IR_store = 0;
unsigned long M_store = 0;
unsigned long IR_SumPeriods_store = 0;
unsigned long M_SumPeriods_store = 0;


// Variables for timing the loop period
const unsigned long period = dt * pow(10,6);       // conversion from seconds to microseconds
unsigned long time_now = 0;

// Variables for converting detections to velocities
const double conversion = 0.041291804;   // Conversion from gear and from wheel rotation to wheel motion
const unsigned long IR_conversion_period = pow(10,9) * conversion / 20;   // For integer division storage multiplied by 10^9
const unsigned long M_conversion_period = pow(10,9) * conversion / 8;     // For integer division storage multiplied by 10^9

// Variables for storing the velocities
unsigned long IR_vel_period = 0;
unsigned long M_vel_period = 0;
unsigned long period_vel = 0;
unsigned long filter_vel = 0;


////////////////////////////////////
// Setup for the serial communication and the digital interrupt pins
void setup()  // Start of setup:
{
  // Begin serial communication
  Serial.begin(115200);
  // Create an input pullup for the hall sensor
  pinMode(3, INPUT_PULLUP);
  pinMode(4, OUTPUT);
  digitalWrite(4,HIGH);
  // Attach an interrupt function to pin 2 for the IR sensor, call this function when the state changes on the pin
  attachInterrupt(digitalPinToInterrupt(2), Pulse_Event_IR, CHANGE);  
  // Attach an interrupt function to pin 3 for the M sensor, call this function when the state changes on the pin
  attachInterrupt(digitalPinToInterrupt(3), Pulse_Event_M, CHANGE);  
}  // End of setup.



///////////////////////////////////
// Run the main loop
void loop() {
  // Reset the timer for the sampling time
  time_now = micros();

  // Store the incremented detection variables for futher calculations
  IR_store = IR_SumPeriods/i_IR;
  i_IR = 0;
  IR_SumPeriods = 0;
  M_store = M_SumPeriods/i_M;
  i_M = 0;
  M_SumPeriods = 0;

  

  // Convert incremented detections to velocities
  IR_vel_period = IR_conversion_period / IR_store;  // Velocity is multiplied by 1000
  M_vel_period = M_conversion_period / M_store;      // Velocity is multiplied by 1000

  period_vel = (IR_vel_period + M_vel_period) / 2;
   

  // Send the velocities over serial connection
  Serial.println(period_vel);
  Serial.flush();

  // Run while loop for remaining time
  while( micros() < time_now + period)
  {
    // Wait until the sampling period 
  }

} // End of loop


///////////////////////////////////
// Functions for the interrupt pins

// Run this function when pin 2 (IR sensor) is interrupted
void Pulse_Event_IR()
{
  IR_SumPeriods = IR_SumPeriods + (micros() - IR_LastTimeWeMeasured);  // Check the timing between the last interrupt and this interrupt
  IR_LastTimeWeMeasured = micros();                              // Store the new interrupt time
  
  i_IR++;
}

// Run this function when pin 3 (M sensor) is interrupted
void Pulse_Event_M()
{
  M_SumPeriods = M_SumPeriods + (micros() - M_LastTimeWeMeasured);  // Check the timing between the last interrupt and this interrupt
  M_LastTimeWeMeasured = micros();                              // Store the new interrupt time
  
  i_M++;
}
