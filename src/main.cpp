
//---------------------------BEGIN VARIABLES--------------------------------------
#pragma region Variables
#include <Arduino.h>
#include<Wire.h>
#include<math.h>
#include "OLED_Driver.h"
#include "OLED_GFX.h"

//#include <Adafruit_GFX.h>
//#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 128 // OLED display height, in pixels
#include <SPI.h>

//COLORS
#define	BLACK           0x0000
#define	BLUE            0x001F
#define	RED             0xF800
#define	GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0  
#define WHITE           0xFFFF

#define OLED_RESET 4
OLED_GFX display = OLED_GFX();

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
int scopeOffset=0; //Ammount of pixels to offset the scope by
int xOffset = 0;
int yOffset = 1;
int crosshairSize = 1; //One is default isze
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

int timer = 0;
bool isTiming = false;
int photogateStart = 1;
int photogateEnd = 2;



void handleShot();
void updateSightData();
void setVoltage();
void printData(String text, int x , int y, int size);
int getDistance(int velocity, int initialAngle);
int getVelocity(int time);
int setGyroscopePosition(int yMaxHeight, int yMinHeight, float gyro);
void drawCrosshair();
void DotInMiddleWithLines();
void DotInMiddleWithoutLines();
void DisplayLogo();

//Array of void pointers that display the crosshair
void (*crosshairDisplayModes[])() ={&drawCrosshair, &DotInMiddleWithLines, &DotInMiddleWithoutLines};
int pointerIndex = 0;
bool hasChanged = false;
int8_t displayChangePin = 0;

#pragma endregion
//---------------------------END VARIABLES--------------------------------------

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
  pinMode(displayChangePin, INPUT);

    //Init GPIO
    pinMode(oled_cs, OUTPUT);
    pinMode(oled_rst, OUTPUT);
    pinMode(oled_dc, OUTPUT);
#if INTERFACE_4WIRE_SPI
  //Init SPI
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.begin();

#elif INTERFACE_3WIRE_SPI

  pinMode(oled_sck, OUTPUT);
  pinMode(oled_din, OUTPUT);

#endif
  display.Device_Init();

  DisplayLogo();
  delay(100);
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
  //Only changed if the user has pressed it and hasnt changed it so we dont change each cycle
  if(analogRead(displayChangePin) == HIGH && !hasChanged){
    hasChanged = true;
    pointerIndex;
  }
  //Reset bool if user has let go of button
  if(analogRead(displayChangePin) == LOW && hasChanged){
    hasChanged = false;
  }

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

}

/**
 * Handles all the data for the shooting
 */
void handleShot(){

  int distance = getDistance(getVelocity(timer),  floor((abs(angleDefs[2] - angleDefs[3]) * (angleDefs[0] - angleDefs[1]) / analogRead(A0))+ angleCompensate));  //Get distance

  //Reset timer
  timer=0;


  char intToDist [5]; //Create a string to copy distance to
  //Convert to string
  itoa(distance, intToDist, 10); //Convert int to char*
  char* distanceString = (char*)"Distance: ";  //Original string
  strcpy(distanceString, intToDist); // Copy  int to the distance string to get:  Distance: int

  //TODO: handle data
}



/**
 * Updates the sight including all of the values
 */
void updateSightData(){
  display.Clear_Screen();
  setVoltage();

  crosshairDisplayModes[pointerIndex]();

}

void setVoltage(){

}

void printData(String text, int x , int y, FONT_SIZE size){
  //display.Set_FontSize();
    // display.setTextColor(WHITE);
    // display.setCursor(x,y);
    // display.println(text);
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
 * TODO: This requires more testing for which vaule gyro should be divided by. Also kinda requires a gyroscope
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
  display.Draw_Line(newXPos[0], startingYPos, newXPos[1], startingYPos);


  //Vertical line
  int xPos = floor(double(SCREEN_WIDTH / 2) + xOffset);
  int startingYPosH = floor(double(SCREEN_HEIGHT * .25));
  int endingYPosH = floor(double(SCREEN_HEIGHT * .75));

  //Calculate midpoint and offset
  int offsetY = ((endingYPosH - startingYPosH) * crosshairSize) / 2;
  int midpointY = (startingYPosH + endingYPosH) / 2;

  //Calculate new line position using the offset
  int newYPos [2] = {(midpointY - offsetY) + yOffset, (midpointY + offsetY) + yOffset};

  display.Draw_Line(xPos, newYPos[0], xPos, newYPos[1]);

}

void DotInMiddleWithLines(){
  //Circle
  int16_t circleRaidus = floor(1 *crosshairSize);
  int16_t midpoint = double(SCREEN_WIDTH / 2) + xOffset;
  int16_t circleYPos = double(SCREEN_HEIGHT / 2) +yOffset;
  display.Fill_Circle(midpoint, circleYPos, circleRaidus);


  //X Axis
  int16_t length = ((double(SCREEN_WIDTH / 2) * crosshairSize) / 2) - 4;

  int16_t xLineYPos =  floor(double(SCREEN_HEIGHT * .75));
  int16_t xLineOneXPos = floor((midpoint - (length + 4)) + xOffset);
  int16_t xLineTwoXPos = floor((midpoint + 4) + xOffset);

  display.Draw_FastHLine(xLineOneXPos, xLineYPos, length);
  display.Draw_FastHLine(xLineTwoXPos, xLineYPos, length);



  //Y Axis
  int16_t yLength = ((double(SCREEN_HEIGHT / 2) * crosshairSize) / 2) - 4;

  int16_t yLineXPos =  floor(double(SCREEN_HEIGHT * .75));
  int16_t yLineOneYPos = floor((midpoint - (yLength + 4)) + yOffset);
  int16_t yLineTwoYPos = floor((midpoint + 4) + yOffset);

  display.Draw_FastVLine(yLineXPos, yLineOneYPos, yLength);
  display.Draw_FastVLine(yLineXPos, yLineTwoYPos, yLength);

}

void DotInMiddleWithoutLines(){
  int16_t xPos = ((SCREEN_WIDTH / 2) + xOffset);
  int16_t yPos = (SCREEN_HEIGHT / 2) + yOffset;
  int16_t raidus = (1 * crosshairSize) * 2;

  display.Fill_Circle(xPos, yPos, raidus);
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

void DisplayLogo(void)  {
  display.Clear_Screen();
  int color = 0xF800;
  int t;
  int w = SCREEN_WIDTH/2;
  int x = SCREEN_HEIGHT-1;
  int y = 0;
  int z = SCREEN_WIDTH;
  for(t = 0 ; t <= 15; t+=1) {
    display.Draw_Triangle(w, y, y, x, z, x);
    x-=4;
    y+=4;
    z-=4;
    color+=100;
    display.Set_Color(color);
    delay(10);
  }
  delay(3000);

}



