#include <Adafruit_DotStar.h>
#include <SPI.h>    
// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//


// Modifed by marius for hat


// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */



#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

long lastFrameTime;

VectorFloat velocity;
Quaternion orientation;
const int NPIX = 37, FIRST_PIX_OFF = NPIX/2; // First pix is back
const float RADIUS = 0.2, ELLIPTICITY = (15.0/20);
float circleSin[NPIX], circleCos[NPIX];
bool settle;

const int NSMOOTH=20;
int /*axs[NSMOOTH] = {}, ays[NSMOOTH] = {}, */ azs[NSMOOTH] = {};
long axsum=0, aysum=0, azsum=0; // sum of last NSMOOTH values
int aptr=0; // current index into the axs, .. arrays
int axsm=0, aysm=0, azsm=0; // smoothed ax (high pass filter)
float x=0, y=0, z=0;
float vx=0, vy=0, vz=0;

float sinCrazyAngle = 0.9092974268256817;
float cosCrazyAngle = -0.4161468365471424;

long micSumLong = 0;
bool micPhase = false, triggered = false;
long micTriggers = 0;
int olx=0, oly=0, olz=0;

// Speed check heuristic to bring speed back to zero in certain cases
const int NCHECK = 25;
// Square of axsm values for evaluating whether last period is of 
// large or small acceleration
unsigned int ax2s[NCHECK] = {}, ay2s[NCHECK] = {}, az2s[NCHECK] = {};
long ax2sum=0, ay2sum=0, az2sum=0;
int a2ptr=0;


// Here's how to control the LEDs from any two pins:
#define DATAPIN    4
#define CLOCKPIN   5
Adafruit_DotStar strip = Adafruit_DotStar(NPIX);


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    pinMode(4, INPUT_PULLUP);
    strip.begin();
    for (int i=0; i<NPIX; ++i) {
      strip.setPixelColor(i, 1, 1, 1);
    }
    strip.show();
    for (int i=0; i<NPIX; ++i) {
      circleSin[i] = 128*RADIUS*ELLIPTICITY*sin(((i+FIRST_PIX_OFF)*2*PI)/NPIX);
      circleCos[i] = 128*RADIUS*cos(((i+FIRST_PIX_OFF)*2*PI)/NPIX);
    }
  
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for settle..."));
        dmpReady = true;
        settle = false;
        ax2sum = ay2sum = az2sum = 2000; // So it doesn't settle on first round
        ax2s[0] = 2000;
        ay2s[0] = 2000;
        az2s[0] = 2000;
        a2ptr++;
        delay(5000);
        
        for (int i=0; i<NPIX; ++i) {
          strip.setPixelColor(i, 22, 22, 0);
        }
        strip.show();
        
        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}

byte rand(byte x) {
  x ^= (x << 5);
  x ^= (x >> 3);
  x ^= (x << 2);
  return (byte)x;
}

/*
byte rand(int x) {
  return (byte)(1103515245 * x + 12345);
}*/

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================


void loop() {

  /*long now2 = millis();  
  if (now2 - cp > 1000) {
    cp = now2;
    Serial.print(counter);
    Serial.println("loops");
    counter = 0;
  }
  counter++;*/
    int micVal = analogRead(A0) - 337;
    micSumLong = (micSumLong * 100) >> 7;
    if (micPhase) micSumLong += micVal;
    else micSumLong -= micVal;
    micPhase = !micPhase;
    Serial.println(micSumLong);

    if (abs(micSumLong) > 500) {
      if (!triggered) {
        micTriggers += 1000;
      }
      if (micTriggers > 3000) {
        olz = 100;
      }
      triggered = true;
    }
    else {
      triggered = false;
    }
    olz = max(0, olz-1);
    
    micTriggers--;
    
  
    // if programming failed, don't try to do anything
    if (!dmpReady) return;


    // wait for MPU interrupt or extra packet(s) available
    if (!mpuInterrupt && fifoCount < packetSize) {
      if (!settle) {
        long test = ax2sum + ay2sum;
        if (test < 300) {
          settle = true;
          lastFrameTime = micros();
        }
        else {
          //Serial.println(test);
          int endi = min(NPIX, (NPIX*test)/20000);
          strip.clear();
          for (int i=0; i<endi; ++i) {
            strip.setPixelColor(i, 22, 22, 0);
          }
          strip.show();
        }
      }
      else {
        long now = micros();
        long dt = now - lastFrameTime;
        lastFrameTime = now;

        const float scaling = 9.8/8192;
        float dtsec = dt * 1.0e-6;
        vx += scaling * axsm * dtsec;
        vy += scaling * aysm * dtsec;
        vz += scaling * azsm * dtsec;

        if (ax2sum + ay2sum + az2sum < 800) {
          vx *= 0.95;
          vy *= 0.95;
          vz *= 0.95;
        }

        x += vx*dtsec;
        y += vy*dtsec;
        z += vz*dtsec;


        // Rotate unit vectors by quaternion (mangled from Microsoft code)

        float num12 = q.x + q.x;
        float num2 = q.y + q.y;
        float num = q.z + q.z;
        float num11 = q.w * num12;
        float num10 = q.w * num2;
        float num9 = q.w * num;
        float num8 = q.x * num12;
        float num7 = q.x * num2;
        float num6 = q.x * num;
        float num5 = q.y * num2;
        float num4 = q.y * num;
        float num3 = q.z * num;
        
        //float num15 = ((value.X * ((1f - num5) - num3)) + (value.Y * (num7 - num9))) + (value.Z * (num6 + num10));
        float uxx = (1.0 - num5) - num3;
        float uyx = num7 - num9;
        //float uzx = num6 + num10;

        //float num14 = ((value.X * (num7 + num9)) + (value.Y * ((1f - num8) - num3))) + (value.Z * (num4 - num11));
        float uxy = num7 + num9;
        float uyy = (1.0 - num8) - num3;
        //float uzy = num4 - num11;
        
        //float num13 = ((value.X * (num6 - num10)) + (value.Y * (num4 + num11))) + (value.Z * ((1f - num8) - num5));
        float uxz = num6 - num10;
        float uyz = num4 + num11;
        //float uzz = (1.0 - num8) - num5;


        byte* pixels = strip.getPixels();
        int pixel_index = 0;
        float xk=x*128, yk=y*128, zk = z*128;

        int full = 3 + min((ax2sum + ay2sum) >> 8, 80);
        for (int i=0; i<NPIX; ++i) {
          int r=0, g=0, b=0;
          long lx = (xk + uyx * circleSin[i] + uxx * circleCos[i]);
          long ly = (yk + uyy * circleSin[i] + uxy * circleCos[i]);
          long lz = (zk + uyz * circleSin[i] + uxz * circleCos[i]);

          int scene = (abs((lz)>>9) & 3);
          
          if (scene == 0 || scene == 1) {
            // Default grønn bg, rød og gule kuler (sylindere)
            
            int glev = rand((lx*5 + lz*2)>>10)>>1;
            int glev2 = rand((ly*6 + lz*2)>>10)>>1;
            
            g = (full*min(glev + glev2, 256)) >> 8;
            
            if (rand((30*lx)>>10) > 225 && rand((30*ly)>>10) > 220) {
              r = (full*2)/3; // kule!
              g = 0;
            }
            else if (rand((30*lx + 4000)>>10) > 220 && rand((30*ly + 11000)>>10) > 220) {
              // gul kule
              r = g = (full*2/3);
            }
            else if (scene == 1) {
              if (g == 0) {
                // blå himmel bakgrunn 
                b = 1;
              }
            }
          }
          else if (scene == 2) {
            // blå stjernehimmel
            b = 1;
            int star1 = rand((60*lx + 4)>>10);
            int star2 = rand((66*ly + 11)>>10);
            
            if (star1 + star2 > 400) {
              r = full; 
              g = full;
              b = full;
            }
          }
          else if (scene == 3) {
            // crazy planes
            if ((lx>>11 & 1) != 0) {
              int xt = (lx*50 + ly*50) >> 10;
              if ( (xt ) & 1 ) {
                g = b = full;
              }
              else {
                r = b = full;
              }
            }
            else {
              if ( ((lx*20 + ly*20 + lz*20)>>10 & 1) == 1) {
                r = full;
                g = b = 0;
              }
            }
          }
          pixels[pixel_index] = min(olz+b, 255);
          pixels[pixel_index+1] = min(olz+g, 255);
          pixels[pixel_index+2] = min(olz+r, 255);
          pixel_index+=3;
        }
        strip.show();
/*
            Serial.print("p\t");
            Serial.print(ax2sum);
            Serial.print("\t");
            Serial.print(ay2sum);
            Serial.print("\t");
            Serial.print(az2sum);
            Serial.print("\tv\t");
            Serial.print(vx);
            Serial.print("\t");
            Serial.print(vy);
            Serial.print("\t");
            Serial.print(vz);
            Serial.print("\tp\t");
            Serial.print(x);
            Serial.print("\t");
            Serial.print(y);
            Serial.print("\t");
            Serial.print(z);
            Serial.print("\tdt_cpu\t");
            Serial.println( micros()-now);
  */    }
    }
    
    while (!mpuInterrupt && fifoCount < packetSize) {
    }
    
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continu0e cleanly
        mpu.resetFIFO();
        fifoCount = 0;
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        while (fifoCount >= packetSize) {
          mpu.getFIFOBytes(fifoBuffer, packetSize);
          
          // track FIFO count here in case there is > 1 packet available
          // (this lets us immediately read more without waiting for an interrupt)
          fifoCount -= packetSize;
        }


            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

        axsm = aaWorld.x ;/*- (axsum/NSMOOTH);
        axsum = (axsum - axs[aptr]) + aaWorld.x;
        axs[aptr] = aaWorld.x;*/
        
        aysm = aaWorld.y ;/*- (aysum/NSMOOTH);
        aysum = (aysum - ays[aptr]) + aaWorld.y;
        ays[aptr] = aaWorld.y;*/
        
        azsm = aaWorld.z - (azsum/NSMOOTH);
        azsum = (azsum - azs[aptr]) + aaWorld.z;
        azs[aptr] = aaWorld.z;

        aptr = (aptr + 1) % NSMOOTH;

        unsigned int ax2 = abs(axsm);
        ax2sum = (ax2sum - ax2s[a2ptr]) + ax2;
        ax2s[a2ptr] = ax2;
        
        unsigned int ay2 = abs(aysm);
        ay2sum = (ay2sum - ay2s[a2ptr]) + ay2;
        ay2s[a2ptr] = ay2;
        
        unsigned int az2 = abs(azsm);
        az2sum = (az2sum - az2s[a2ptr]) + az2;
        az2s[a2ptr] = az2;
        
        a2ptr = (a2ptr + 1) % NCHECK;
        
        
        // weird overflow error on a*2sum, reset to zero
        if (ax2sum > 1000000) {
          ax2sum = 0;
          for (int i=0; i<NCHECK; ++i) {
            ax2sum += ax2s[i];
          }
        }
        if (ay2sum > 1000000) {
          ay2sum = 0;
          for (int i=0; i<NCHECK; ++i) {
            ay2sum += ay2s[i];
          }
        }
        if (az2sum > 1000000) {
          az2sum = 0;
          for (int i=0; i<NCHECK; ++i) {
            az2sum += az2s[i];
          }
        }
        
    }
}

