#include <Arduino.h>
#include <Adafruit_CircuitPlayground.h>

#define DEBUG_ETHAN 1
#define SOUND 1   // 1 to turn on sound, 0 to turn off sound (debugging)
#define NODE 2    // Changed this to Node 2 for testing (Terry)

#define NEOPIXEL_BRIGHTNESS 20  // default is 30

// Ethan
#define MOTION_1G               9.8
#define MOTION_IDLE_ACCELERROR  1.0
#define MOTION_MEAS_ACCELERROR  0.4
#define MOTION_1G_IDLE_LOBOUND  MOTION_1G - MOTION_IDLE_ACCELERROR
#define MOTION_1G_IDLE_HIBOUND  MOTION_1G + MOTION_IDLE_ACCELERROR
#define MOTION_1G_MEAS_LOBOUND  MOTION_1G - MOTION_MEAS_ACCELERROR
#define MOTION_1G_MEAS_HIBOUND  MOTION_1G + MOTION_MEAS_ACCELERROR
#define MOTION_IDLE_FILTERSZ    7

float MOTION_X_OFFSET = 0;
float MOTION_Y_OFFSET = 0;
float MOTION_Z_OFFSET = 0;

struct motionAvg {
  float x = 0;
  float y = 0;
  float z = 0;
};

// ===== Notes =================================================================

// Just to make it clear: 
// The movement to message mapping is different for each person and is assumed accessible outside of code.
// Movement are directly mapped onto LEDs - Movement 1 is NeoPixel 0, Movement 2 is NeoPixel 1 and so on...

// Ethan (12/12/2023):
/*
I am assuming that the microcontroller is always oriented with the USB port
pointing directly DOWNWARDS. Normal force acceleration messes up the average
values and the only trivial way to get rid of the normal force is to add
an offset.
*/



// prototypes
char checkSubMovement();
char checkMovement();
void debug_check_submovement(char &sm);
// void print_accelerometer_values();   // Debug purpose

// Ethan
float square(float num) { return num * num; }
float motionMagSq();        // gets square of magnitude of current accel
void motionIdleWait();      // waits while board is idle (not moving)
void motionMagLED(float mag2);        // lights the neopixels
motionAvg motionGetAvg();   // gets average accel during movement and outputs it


// Person 1: Terry
void setup() {
    CircuitPlayground.begin();                              // For accelerometer and Neopixels
    CircuitPlayground.setAccelRange(LIS3DH_RANGE_4_G);      // Set the accelerometer range to +- 4g
    CircuitPlayground.setBrightness(NEOPIXEL_BRIGHTNESS);   // Set up initial brightness
    Serial.begin(9600);
}

// Person 5: Seoyoon
void loop() {

  // Ethan's tester code, don't uncomment
  // while (!(CircuitPlayground.leftButton())); delay(1000); checkSubMovement(); 

  // char message_index = 0;
  
  // if (PIND & (1 << 4)) {    // left button pressed
  //   // char message_index = checkMovement();
  //   // Serial.println(message_index);
  // }

  // don't do anything until button pressed. then once it is pressed, wait a second and then check
  while (!(CircuitPlayground.leftButton())); delay(1000);

  // Initialization
  CircuitPlayground.clearPixels();
  char message_index = checkMovement() - 1;                    // Get the movement(message) number in char
  // print_accelerometer_values();                         // Debug purpose
  // Serial.println(message_index);
  // First decide which node it is
  // switch(NODE){
  //   case 1:
  //     Serial.println("I am Node 1, I am going to pick up Node 2's microcontroller and make some movements!");
  //     delay(2000);
  //     break; 

  //   case 2:
  //     CircuitPlayground.setPixelColor(messages2[(int)message_index], 0, 255, 0);   // Light up the corresponding NEOPIXEL in green
  //     delay(2000);    // We need to add some delays so that we can see the LED light up before going into the next loop!
  //     break;

  //   case 3:
  //     CircuitPlayground.setPixelColor(messages3[(int)message_index], 0, 255, 0);   // Light up the corresponding NEOPIXEL in green
  //     delay(2000);
  //     break;

  //   case 4:
  //     CircuitPlayground.setPixelColor(messages4[(int)message_index], 0, 255, 0);   // Light up the corresponding NEOPIXEL in green
  //     delay(2000);
  //     break;

  //   case 5:
  //     CircuitPlayground.setPixelColor(messages5[(int)message_index], 0, 255, 0);   // Light up the corresponding NEOPIXEL in green
  //     delay(2000);
  //     break;

  //   default:
  //     Serial.println("Node not found");
  // }
  if(message_index==-1){
    for(char i = 0; i<10; i++)
      CircuitPlayground.setPixelColor(i, 255, 0, 0);
  }
  else{
    CircuitPlayground.setPixelColor(
      message_index, 
      (message_index)*28,       // R: no red at 0, very red at 9
      252-((message_index)*28), // G: very green at 0, no green at 9
      0
    );
  }
}

// =============================================================================

// Person 2: Ahmed
char checkMovement() {
    // check movement via accelerometer and returns a value 0-9
    char sub1 = checkSubMovement();

    delay(1000);
    char sub2 = checkSubMovement();

    delay(1000);
    char sub3 = checkSubMovement();

    char movement = 0;  
    
    if(sub1 == 4 && sub2 == 2 && sub3 == 3) {       // -X, -Y, +X
        movement = 1;
    }
    else if(sub1 == 4 && sub2 == 2 && sub3 == 4) {  // -X, -Y, -X
        movement = 2; 
    } 
    else if(sub1 == 3 && sub2 == 4 && sub3 == 2) {  // +X, -X, -Y
        movement = 3;
    }
    else if(sub1 == 6 && sub2 == 2 && sub3 == 5) {  // -Z, -Y, +Z
        movement = 4;
    }
    else if(sub1 == 1 && sub2 == 4 && sub3 == 5) {  // +Y, -X, +Z
        movement = 5;
    }
    else if(sub1 == 6 && sub2 == 4 && sub3 == 5) {  // -Z, -X, +Z
        movement = 6;
    }
    else if(sub1 == 6 && sub2 == 5 && sub3 == 6) {  // -Z, +Z, -Z
        movement = 7;
    }
    else if(sub1 == 2 && sub2 == 1 && sub3 == 6) {  // -Y, +Y, -Z
        movement = 8;
    }
    else if(sub1 == 2 && sub2 == 6 && sub3 == 1) {  // -Y, -Z, +Y
        movement = 9;
    }
    else if(sub1 == 3 && sub2 == 6 && sub3 == 2) {  // +X, -Z, -Y
        movement = 10;
    }

    Serial.print("The movement is: "); Serial.println((uint8_t)movement);
    return movement;
}

// Ethan
// Returns the square of the magnitude of acceleration
// This is essentially the magnitude calculation:
//      sqrt(x^2 + y^2 + z^2)
// but without the sqrt because we don't need it
float motionMagSq() {
  float x2 = 0, y2 = 0, z2 = 0, mag2 = 0;
  x2 = square(CircuitPlayground.motionX());
  y2 = square(CircuitPlayground.motionY());
  z2 = square(CircuitPlayground.motionZ());
  mag2 = x2 + y2 + z2;

  return mag2;
}

// Ethan
// checks if the current magnitude of motion is within the idle bounds
// idle bounds are set via macro definitions at the top of this file
// 
void motionIdleWait() {
  // set all neopixels to purple to indicate idle
  CircuitPlayground.setBrightness(10);
  for (uint8_t i = 0; i < 10; i++) CircuitPlayground.setPixelColor(i, 64, 0, 64);

  // running average filter and fill the filter with idle values
  float motionIdleFilter[MOTION_IDLE_FILTERSZ] = {0};
  for (uint8_t i = 0; i < MOTION_IDLE_FILTERSZ; i++) 
    motionIdleFilter[i] = square(MOTION_1G);

  // loop, when not idle then return from loop
  while (1) {
    float mag2 = 0;   // average magnitude squared

    // update filter
    for (uint8_t i = 1; i < MOTION_IDLE_FILTERSZ; i++) {
      motionIdleFilter[i - 1] = motionIdleFilter[i];
      mag2 += motionIdleFilter[i - 1];
    }

    // add new measurement to filter
    motionIdleFilter[MOTION_IDLE_FILTERSZ - 1] = motionMagSq();
    mag2 += motionIdleFilter[MOTION_IDLE_FILTERSZ - 1];
    
    mag2 /= float(MOTION_IDLE_FILTERSZ); // filtered movement magnitude sample

    // debug
    // if (DEBUG_ETHAN) {
    //   Serial.println("motionIdleWait()");
    //   Serial.print("Lo bound is: "); Serial.print(square(MOTION_1G_IDLE_LOBOUND)); Serial.print("\t");
    //   Serial.print("Mi value is: "); Serial.print(square(MOTION_1G));         Serial.print("\t");
    //   Serial.print("Hi bound is: "); Serial.println(square(MOTION_1G_IDLE_HIBOUND));
    //   Serial.print("Mag2 is: "); Serial.println(mag2);
    // }


    // if within idle bounds, do nothing. else, return, because we are not idle
    if ( 
      (mag2 > square(MOTION_1G_IDLE_LOBOUND)) && 
      (mag2 < square(MOTION_1G_IDLE_HIBOUND)) ) {
      // debug
      // Serial.println("Idle");
      MOTION_X_OFFSET = -CircuitPlayground.motionX();
      MOTION_Y_OFFSET = -CircuitPlayground.motionY();
      MOTION_Z_OFFSET = -CircuitPlayground.motionZ();
    }
    else {
      //debug
      // Serial.println("Not Idle");
      CircuitPlayground.setBrightness(NEOPIXEL_BRIGHTNESS);
      return;
    }
  }
}

// lights LEDs based on current motion
void motionMagLED(float mag2) {
  CircuitPlayground.clearPixels();

  // range is 0g to 1g, not 2g because we offset gravity away
  // gradient from neopixels at bottom being green to neopixels at top being
  // red. redder neopixels light up depending on the magnitude of acceleration
  // >0g   : light up green
  // >0.2g : light up greenish yellow
  // >0.4g : light up yellow
  // >0.6g : light up yellowish red
  // >0.8g : light up red
  CircuitPlayground.setPixelColor(4, 0, 255, 0);
  CircuitPlayground.setPixelColor(5, 0, 255, 0);
  if (mag2 > square(float(2))) {
    CircuitPlayground.setPixelColor(3, 64, 192, 0);
    CircuitPlayground.setPixelColor(6, 64, 192, 0);
  }
  if (mag2 > square(float(4))) {
    CircuitPlayground.setPixelColor(2, 128, 128, 0);
    CircuitPlayground.setPixelColor(7, 128, 128, 0);
  }
  if (mag2 > square(float(6))) {
    CircuitPlayground.setPixelColor(1, 192, 64, 0);
    CircuitPlayground.setPixelColor(8, 192, 64, 0);
  }
  if (mag2 > square(float(8))) {
    CircuitPlayground.setPixelColor(0, 255, 0, 0);
    CircuitPlayground.setPixelColor(9, 255, 0, 0);
  }
}

// Person 3: Ethan (accelerometer measurements)
motionAvg motionGetAvg() {
  // Ethan's part
  motionAvg buf;
  uint16_t samples = 0;
  uint16_t samplesLimit = 50;
  uint16_t w = 0;
  uint16_t wLimit = 10;
  float vx = 0, vy = 0, vz = 0;

    // play tone to indicate start of sampling: 500Hz 0.2, 1kHz 0.2
  if (SOUND) {
    CircuitPlayground.playTone(500, 50);
    CircuitPlayground.playTone(1000, 50);
  }

  // wait while idle
  motionIdleWait();

  // break when either:
  //    samplesLimit amount of samples collected OR
  //    wLimit amount of good values collected
  while (samples < samplesLimit) {

    float ax = CircuitPlayground.motionX() + MOTION_X_OFFSET;
    float ay = CircuitPlayground.motionY() + MOTION_Y_OFFSET;
    float az = CircuitPlayground.motionZ() + MOTION_Z_OFFSET;

    if (w < wLimit) {
      // if nonidle by some margin, add one to the weight to indicate good measurement
      if ((square(ax) > 4) ||    
          (square(ay) > 4) ||  
          (square(az) > 4)) w++;

      // update velocities and tally velocities for averaging later
      vx += ax;    vy += ay;    vz += az;
      buf.x += vx; buf.y += vy; buf.z += vz;
    }

    // debug
    if (DEBUG_ETHAN) {
      Serial.print("motionGetAvg(), a\t");
      Serial.print(ax); Serial.print("\t");
      Serial.print(ay); Serial.print("\t");
      Serial.println(az);
    }

    // flash lights to show magnitude of acceleration
    float mag2 = square(ax) + square(ay) + square(az);
    motionMagLED(mag2);

    // idle
    if (w >= wLimit) break;
    samples++;
  }

  // clears LEDs so that they aren't stuck to last measurement
  CircuitPlayground.clearPixels();
  // play tone to indicate start of sampling: 1kHz 0.1
  if (SOUND) CircuitPlayground.playTone(1000, 100);
  
  // divide by number of good samples taken to get average velocity
  float bufferAvg = abs(buf.x) + abs(buf.y) + abs(buf.z);
  buf.x = (buf.x * 100.0) / bufferAvg;
  buf.y = (buf.y * 100.0) / bufferAvg;
  buf.z = (buf.z * 100.0) / bufferAvg;

  // debug
  if (DEBUG_ETHAN) {
    Serial.print("motionGetAvg() w:\t"); Serial.println(w);
    Serial.print("motionGetAvg() avg:\t");
    Serial.print("Xavg: "); Serial.print(buf.x); Serial.print("\t");
    Serial.print("Yavg: "); Serial.print(buf.y); Serial.print("\t");
    Serial.print("Zavg: "); Serial.print(buf.z); Serial.print("\t");
    Serial.print("Avg:  "); Serial.print(bufferAvg); 
    Serial.println();
  }

  // delay(4000);

  return buf;
}

// Person 4: Jaif (analysis)
char checkSubMovement() {
  motionAvg avg = motionGetAvg();
  /*
  avg.x is the signed normalized average of movement in the x direction
  avg.y is the signed normalized average of movement in the y direction
  avg.z is the signed normalized average of movement in the z direction
  signed normalized average: all values are in the interval [-100, 100] and
    they represent the average acceleration in their respective direction.
    The magnitudes of all values add up to 100. See the examples of valid values
    of avg below:
      X: -50, Y:  20, Z:  30    because 50 + 20 + 30 = 100
        NORMALIZED: X: -71 Y: 29
      X: 100, Y:   0, Z:  0     because 100 + 0 + 0 = 100
        NORMALIZED: X: -71 Y: 29
      X: -10, Y: -85, Z:  5     because 10 + 85 + 5 = 100
        NORMALIZED: X: -11 Y: -89
  In the first example above, 
    X: -50 indicates that 50% of the total magnitude of the motion was in the -x direction
    Y: 20 indicates that 20% of the total magnitude of the motion was in the +y direction
    Z: 30 indicates that 30% of the total magnitude of the motion was in the +z direction
  */

  // at this point, bufferNavg holds the normalized average of the N direction

  // Jaif's part
  /*
  Comparisons are made with the assumnption that the micro usb port is facing up
  and the board facing away from the user. Positive Y axis of motion is parallel
  to the ground and right from the perspective of the user. Positive X is up.
  */
  
    //Move left
  char direction;
  bool sign;   // 0 means negative and 1 means positive

  if      ( (abs(avg.x) > abs(avg.y)) && (abs(avg.x)>abs(avg.z)) ) direction = 'x';
  else if ( (abs(avg.y) > abs(avg.x)) && (abs(avg.y)>abs(avg.z)) ) direction = 'y';
  else if ( (abs(avg.z) > abs(avg.x)) && (abs(avg.z)>abs(avg.y)) ) direction = 'z';

  if (direction == 'x') {       //Set sign if x
    if (avg.x < 0) sign = 0;
    else           sign = 1;
  }
  else if (direction == 'y') {  //Set sign if y
    if (avg.y < 0) sign = 0;
    else           sign = 1;
  }
  else if (direction == 'z') {  // Set sign if z
    if (avg.z < 0) sign = 0;
    else           sign = 1;
  }
  
  //move left, +Y
  if(direction == 'y' && sign){
    Serial.print("The submovement is: "); Serial.println(1);
    return 1; 
  }
  //Move right, -Y
  else if(direction == 'y' && !sign){
    Serial.print("The submovement is: "); Serial.println(2);
    return 2;
  }
  //Move up, +X
  else if(direction == 'x' && sign){
    Serial.print("The submovement is: "); Serial.println(3);
    return 3;
  }
  //Move down, -X
  else if(direction == 'x' && !sign){
    Serial.print("The submovement is: "); Serial.println(4);
    return 4;
  }
  //Move away, +Z
  else if(direction == 'z' && sign){
    Serial.print("The submovement is: "); Serial.println(5);
    return 5;
  }
  //Move closer, -Z
  else if(direction == 'z' && !sign){
    Serial.print("The submovement is: "); Serial.println(6);
    return 6;
  }
  return 0;     // If none of the submovement was detected, return 0
}

// ===== DEBUG STUFF ===========================================================

void debug_print_accelerometer_values() {
  float x, y, z;
  x = CircuitPlayground.motionX();
  y = CircuitPlayground.motionY();
  z = CircuitPlayground.motionZ();
  Serial.print(x);
  Serial.print(" ");
  Serial.print(y);
  Serial.print(" ");
  Serial.println(z);
  delay(100);
}


void debug_check_submovement(char &sm){ //function can be used to check if submovement detection is working
//For each submovement, the color is red but a different Neopixel lights up.
//works for 5 submovemets, but can easily be extend/used for fewer submovements. 
    CircuitPlayground.clearPixels();
    switch(sm){
        case 1:
            CircuitPlayground.setPixelColor(1, 255, 0, 0);
            delay(1000);
            CircuitPlayground.setPixelColor(1, 0, 0, 0);
            break;

        case 2:
            CircuitPlayground.setPixelColor(2, 255, 0, 0);
            delay(1000);
            CircuitPlayground.setPixelColor(2, 0, 0, 0);
            break;

        case 3:
            CircuitPlayground.setPixelColor(3, 255, 0, 0);
            delay(1000);
            CircuitPlayground.setPixelColor(3, 0, 0, 0);
            break;

        case 4:
            CircuitPlayground.setPixelColor(4, 255, 0, 0);
            delay(1000);
            CircuitPlayground.setPixelColor(4, 0, 0, 0);
            break;
            
        case 5:
            CircuitPlayground.setPixelColor(5, 255, 0, 0);
            delay(1000);
            CircuitPlayground.setPixelColor(5, 0, 0, 0);
            break;

        case 6:
            CircuitPlayground.setPixelColor(6, 255, 0, 0);
            delay(1000);
            CircuitPlayground.setPixelColor(6, 0, 0, 0);
            break;
    }

}
