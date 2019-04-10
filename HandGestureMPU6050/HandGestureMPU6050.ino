// MPU6050_DMP6 EXAMPLE CODE!!
// SCROLL DOWN UNTIL YOU SEE COMMENTS THAT SAY THAT IT'S NOT PART OF THE EXAMPLE CODE ANYMORE FOR THE INTERESTING CODE!!


#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

#define WaterPlantLight 9
#define GetDataLight 11
bool blinkState = false;
bool showlight = false;

bool GetDataSinglePrint = true;
bool WaterPlantSinglePrint = true;

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

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };


volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
    
    mpu.initialize();
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    
    // configure LED for output
    pinMode(WaterPlantLight, OUTPUT);
    pinMode(GetDataLight, OUTPUT);
}

bool startpos1 = false, startpos2;
bool activeWaterPlant = false, activeGetData = false;
unsigned long WaterPlantTimer = 0, GetDataTimer = 0;
unsigned long activeWaterPlantTimer = 0, activeGetDataTimer;

void loop() {
    if (!dmpReady) return;

    while (!mpuInterrupt && fifoCount < packetSize) {
    }

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
        
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        
        
        // /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ \\
        //                                                                   \\
        // Almost Everything above is part of the MPU6050_DMP6 example code  \\
        //                                                                   \\
        // /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ \\
        //-------------------------------------------------------------------------------------------------------------------------------------------------------------
        
        // \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \\
        // BELOW IS IMPORTANT CODE WHICH IS NOT PART OF THE EXAMPLE CODE ANYMORE


        //Rotation variables
        float zrot = ypr[2] *180/M_PI;
        float xrot = ypr[1] *180/M_PI;

        //Check if gesture for watering plant is made
        if(zrot > 60){
          startpos1 = true;
          WaterPlantTimer = millis()/1000;
        } else {
          if(millis()/1000 - WaterPlantTimer > 0.5){
            startpos1 = false; 
          }
          if(startpos1 && zrot < -40){
            activeWaterPlant = true;
            activeWaterPlantTimer = millis()/1000;
            startpos1 = false;
          }
        }

        //Check if gesture for getting the data is made
        if(abs(xrot) < 20){
          startpos2 = true;
          GetDataTimer = millis()/1000;
        } else {
          if(millis()/1000 - GetDataTimer > 0.5){
            startpos2 = false; 
          }
          if(startpos2 && xrot > 60){
            activeGetData = true;
            activeGetDataTimer = millis()/1000;
            startpos2 = false;
          }
        }

        if(activeWaterPlant){
          WaterPlant();
        }

        if(activeGetData){
          GetData();
        }
    }
}

//Water the plant!
void WaterPlant(){
  if(WaterPlantSinglePrint){
    WaterPlantSinglePrint = false;
    Serial.println("WaterPlant");
  }
  digitalWrite(WaterPlantLight, HIGH);
  if(millis()/1000-activeWaterPlantTimer > 3){
    digitalWrite(WaterPlantLight, LOW);
    activeWaterPlant = false;
    WaterPlantSinglePrint = true;
  }
}

//Get all the data
void GetData(){
  if(GetDataSinglePrint){
    GetDataSinglePrint = false;
    Serial.println("GetData");
  }
  digitalWrite(GetDataLight, HIGH);
  if(millis()/1000-activeGetDataTimer > 3){
    digitalWrite(GetDataLight, LOW);
    activeGetData = false;
    GetDataSinglePrint = true;
  }
}
