/*
   LIBRARY
   Wire.h untuk
   math.h untuk rumus inverse kinematics
   TimerOne.h untuk sensor UVtron, SRF, dan SharpGP
   TimerThree.h untuk photodiode dan line sensor
   Adafruit_GFX.h untuk OLED 128x64
   Adafruit_SSD1306.h untuk OLED 128x64
   Pixy2SPI_SS.h untuk cam pixy2
*/
#include <Wire.h>
#include <TimerOne.h>
#include <TimerThree.h>
#include <Pixy2SPI_SS.h>
#include <Adafruit_GFX.h>
#include <DynamixelSerial.h>
#include <Adafruit_SSD1306.h>

/*
   CYCLE TIME SERIAL DATA
   cycle_time1 untuk TimerOne
   cycle_time3 untuk TimerThreee
*/
#define cycle_time1 10000 //us
#define cycle_time3 10000 //us

// PACKET SERVO DECLARATION
#define ON  1
#define OFF 0
#define AX  0
#define XL  1

// GAIT DECLARATION
#define rad_per_deg   0.017453293
#define deg_per_rad   57.29577951
#define pulse_per_deg 10.0

#define l1 30
#define l2 90

#define min_coxa 127.5
#define max_coxa 167.5

//float min_coxa, max_coxa;

float step_x, step_y, step_z;
float teta, teta1, teta2;
char angkat = 0;
int delay_ms = 140;
int setpoint_kaki[4][3];
float pulsa_kaki[4][3];

// PIXY DECLARATION
Pixy2SPI_SS pixy;
#define calDistance 7.1 //in inches 24inches or 2 foot
#define calDistance2 11.811

const int signature1 = 1 ;
const int signature2 = 2 ;

int cont = 0;
int signature ;
int calWidth = 100; //Calibrated width reading (Cuman mencatat hasil pengukuran)
int calHeight = 90; //Calibrated height reading (Cuman mencatat hasil pengukuran)
int calWidth2 = 80; //Calibrated width reading (Cuman mencatat hasil pengukuran)

int signature1Found = 0;
int signature2Found = 0;
int activeSignature = 0;
int pixelsWidth;   //read by the camera
int pixelsHeight; //read by the camera
float distanceWidth;   //calculated distance based on the width of the object
float distanceWidth2;   //calculated distance based on the width of the object
float widthOfObject = 1.6; //inches (1.6 inches) real size of your object
float heightOfObject = 4.0; //inches (4.0 inches) real size of your object
float widthOfObject2 = 11.02; //inches (1.6 inches) real size of your object
int focalLengthWidth;  //calculated focal length for width
int focalLengthWidth2;  //calculated focal length for width

// SENSOR DECLARATION
int uvstat;
int srf1, srf2, srf3, srf4, srf5, srf6, srf7, srf8;
int sgp1, sgp2, sgp3, sgp4, sgp5, sgp6, sgp7, sgp8;
int mux1, mux2, mux3, mux4, mux5, mux6, mux7, mux8, mux9, mux10, mux11, mux12, mux13, mux14, mux15;
int line1, line2, line3;

// COMPASS DECLARATION
#define   CMPS12_ADDRESS   0x60
#define   ANGLE_8          1

#define   BARAT            330
#define   TIMUR            90
#define   UTARA            14
#define   SELATAN          265

// OLED DECLARATION
#define   SCREEN_WIDTH     128
#define   SCREEN_HEIGHT    64

#define   OLED_RESET       4
#define   SCREEN_ADDRESS   0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ELSE
#define buzzer 5
int buttonState = 0;
const int buttonPin = 24;

// BUZZER CHECK
void cek_buzzer() {
  digitalWrite(buzzer, HIGH);
  delay(1000);
  digitalWrite(buzzer, LOW);
  delay(1000);
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial3.begin(115200);
  Serial5.begin(115200);
  first_setup();
  XLsetup();
  pixy.init();
  focalLengthWidth = (calWidth * calDistance) / widthOfObject;
  focalLengthWidth2 = (calWidth2 * calDistance2) / widthOfObject2;

  pinMode(29, OUTPUT);  //define enable pin here
  digitalWrite(29, ON); //and here too

  // DYNAMIXEL
  Dynamixel.setSerial(&Serial4); // &Serial - Arduino UNO/NANO/MICRO, &Serial1, &Serial2, &Serial3 - Arduino Mega
  Dynamixel.begin(1000000, 29); // Inicialize the servo at 1 Mbps and Pin Control 2

  // PIN IN OUT
  pinMode(buzzer, OUTPUT);
  pinMode(14, OUTPUT);
  pinMode(15, OUTPUT);
  pinMode(16, OUTPUT);
  pinMode(17, OUTPUT);
  pinMode(A18, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);

  //  setPosisi90();
  step_z = 45; //use this to change height step//Higher the number, higher the step//default is 20
  center();
  setPosisi90();

  // ARM INIT
  arm(1);

  // OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }
  display.display();

  // SERIAL DATA
  Timer1.initialize(cycle_time1);
  Timer1.attachInterrupt(SerialReadSlave);
  Timer3.initialize(cycle_time3);
  Timer3.attachInterrupt(SerialRead);
}

void loop() {
  // put your main code here, to run repeatedly:
  /*
    kalau detect korban < 20, bakal berhenti
    kalau ngge, maju terus
  */

    maju(40);
    victimCheck();

    // Serial.print(pixyDistance()); Serial.print("\n");
    //  delay(1000);
    //  victimCheck();
    //  lurusin();
    //    telusurKiri();
    //    telusurKanan();
    //  cek_move();
    //  fireDetection();
    //    srfDebug();
  }
