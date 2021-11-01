#include <Arduino.h>
#include<Wire.h>
#include<math.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2
#define gravityConstant 9.8
#define DISTANCE_BETWEEN_PHOTOGATES 5

//Gyroscope
const int anglePin = A0;
const int angleCompensate = 90; //Offset by 90  
const int angleDefs[4] = {927, 96, -90 , 90}; //Used to map input from gyroscope to a angle. First 2 values are the input map and second two are the angle map



//Scope display
int scopeOffset=0; //Ammount of pixels to offset the scope by
int xOffset = 0;
int yOffset = 1;
int crosshairSize = 1; //One is default isze
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
const int sizePinUp = 2;
const int sizePindown = 4;
bool isDown = false;
//const int scopeDisplayPos [,] = new int[] {{10, 10}, {20,20}}

int timer = 0;
bool isTiming = false;
int photogateStart = 1;
int photogateEnd = 2;

int handleDegrees();
void updateSightData();
void setVoltage();
void handleShot();
void printData(String text, int x , int y, int size);
void displayGyroscope();
int getDistance(int velocity, int initialAngle);
int getVelocity(int time);
int setGyroscopePosition(int yMaxHeight, int yMinHeight, float gyro);
void drawCrosshair();


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  //Timer
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1B |= (1<<CS12); //prescaler 256
  TIMSK1 |= (1<<TOIE1); //Enable time overflow

  //Wire
  Wire.begin();

  Serial.println("Calibrating gyro");

  //TODO: Display message for calibration
  Serial.println("Calculating done");


}

/**
 *Is called whenever the timer overflows 
 */
ISR (TIMER1_OVF_vect){
  if(isTiming){
    timer += millis();
  }
  //TCNT1 = 3036;
}


void loop() {
  // put your main code here, to run repeatedly:
  updateSightData();

  //Start timing
  if(digitalRead(photogateStart) == HIGH){
    isTiming = true;
  }
  //End timing
  if(isTiming && digitalRead(photogateEnd) == HIGH){
    handleShot();
    isTiming = false;
  }

  //Sight Size stuff
  if(digitalRead(sizePindown) == HIGH  && !isDown){
    crosshairSize -= 0.1;
    isDown = true;
    updateSightData();
  } else if (digitalRead(sizePindown) == HIGH && !isDown){
    crosshairSize += 0.1;
        isDown = true;
    updateSightData();
  } else if(digitalRead(sizePindown) == LOW  || digitalRead(sizePindown) == LOW  && isDown){
    isDown = false;
  }

}

//test

/**
 * Handles all the data for the shooting
 */
void handleShot(){

  int distance = getDistance(getVelocity(timer), handleDegrees());  //Get distance

  char intToDist [5]; //Create a string to copy distance to
  //Convert to string
  itoa(distance, intToDist, 10); //Convert int to char*
  char* distanceString = (char*)"Distance: ";  //Original string
  strcpy(distanceString, intToDist); // Copy  int to the distance string to get:  Distance: int

  //TODO: handle data
}


/**
 * Gets degrees of tilt for the gun.
 * TODO: display on sight
 */
int handleDegrees(){
  //Map input to degrees
  //F(x)=(((A - B))/x) * (C-D)) + Offset
  int angle = floor((abs(angleDefs[2] - angleDefs[3]) * (angleDefs[0] - angleDefs[1]) / analogRead(A0))+ angleCompensate);
  return angle;
}

/**
 * Updates the sight including all of the values
 */
void updateSightData(){
  display.clearDisplay();
  drawCrosshair(); //Sets crosshair
  setVoltage();
  handleDegrees(); //Sets display for degrees


  display.display();
}

void setVoltage(){
  
}

void printData(String text, int x , int y, int size){
  display.setTextSize(size);
    // display.setTextColor(WHITE);
    // display.setCursor(x,y);
    // display.println(text);
}


void displayGyroscope(){

}

/**
 * Gets the distance that it will take for projectile to reach ground
 *  @returns Distance 
 */
int getDistance(int velocity, int initialAngle){

  //Equation: Distance= |u^2 * sin(initialAngle)2 / gravityConstant|
  int distance = abs((sqrt(velocity) * sin(initialAngle)*2) / gravityConstant);
  return distance;
}

int getVelocity(int time){
  int velocity = DISTANCE_BETWEEN_PHOTOGATES / time;
  return velocity;
}

/**
 * Returns the x position that the gyroscope should be at
 * TODO: This requires more testing for which vaule gyro should be divided by
 */
int setGyroscopePosition(int yMaxHeight, int yMinHeight, float gyro){
  //Offsets the angle at a certain position
  int xPosition = int(floor(double(gyro / 360) * abs(yMinHeight - yMaxHeight)) + yMinHeight);
  return xPosition;
}

/**
 * Displays the crosshair with the valid offset and draws it relative  to size
 */
void drawCrosshair(){


  //Horizontal line
  //Calculate base points
  int baseXStartingPoint = floor(double(SCREEN_WIDTH * .25));
  int baseXEndingPoint = floor(double(SCREEN_WIDTH * .75));  
  int startingYPos = floor(double(SCREEN_HEIGHT / 2) + yOffset);

  int offset = ((baseXEndingPoint - baseXStartingPoint) * crosshairSize) / 2;
  int midpointX = (baseXStartingPoint + baseXEndingPoint) / 2; 

  //Stretch out the line or shrink it by offset
  int newXPos [2] = {(midpointX - offset) + xOffset, (midpointX + offset)  + xOffset};

  //Draw line
  display.drawLine(newXPos[0], startingYPos, newXPos[1], startingYPos, WHITE);


  //Vertical line
  int xPos = floor(double(SCREEN_WIDTH / 2) + xOffset);
  int startingYPosH = floor(double(SCREEN_HEIGHT * .25));
  int endingYPosH = floor(double(SCREEN_HEIGHT * .75));

  //Calculate midpoint and offset
  int offsetY = ((endingYPosH - startingYPosH) * crosshairSize) / 2;
  int midpointY = (startingYPosH + endingYPosH) / 2;

  //Calculate new line position using the offset
  int newYPos [2] = {(midpointY - offsetY) + yOffset, (midpointY + offsetY) + yOffset};

  display.drawLine(xPos, newYPos[0], xPos, newYPos[1], WHITE);

}

// /**
//  * Flips bytes 
//  * @return Returns inverted array
//  */
// byte** invertData(byte** arr){
//   for(int x = 0; x < SCREEN_WIDTH; x++){
//     for(int y = 0; y < SCREEN_HEIGHT; y++){
//       arr[x][y] = arr[x][SCREEN_HEIGHT - y]; // Swaps data
//     }
//   }

//   return arr;
// }
