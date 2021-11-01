#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <PID_v1.h>

#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C

#define    GYRO_FULL_SCALE_250_DPS    0x00  
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

#define    ACC_FULL_SCALE_2_G        0x00  
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18


double Setpoint, Input, Output;
//Define the aggressive and conservative Tuning Parameters
double aggKp=0.02, aggKi=0.2, aggKd=0.;
double consKp=0.01, consKi=0.4, consKd=0.0;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);
uint32_t timer;


// This function read Nbytes bytes from I2C device at address Address. 
// Put read bytes starting at register Register in the Data array. 
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
  
  // Read Nbytes
  Wire.requestFrom(Address, Nbytes); 
  uint8_t index=0;
  while (Wire.available())
    Data[index++]=Wire.read();
}


// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}

Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire, -1);
double angle = 0.0f;
double readAngle(){
  uint8_t Buf[14];
  I2Cread(MPU9250_ADDRESS,0x3B,14,Buf);
  
  // Accelerometer
  int16_t ax=-(Buf[0]<<8 | Buf[1]);
  int16_t ay=-(Buf[2]<<8 | Buf[3]);
  int16_t az=Buf[4]<<8 | Buf[5];

  // Gyroscope
  int16_t gx=-(Buf[8]<<8 | Buf[9]);
  int16_t gy=-(Buf[10]<<8 | Buf[11]);
  int16_t gz=Buf[12]<<8 | Buf[13];

  double a = double(ax)/131.0f;
  double b = double(gy)/182.0f;
  double dtC = 0.010f;
  double tau=0.075f;
  double A=tau/(tau+dtC);
  angle=A*(angle+b*0.02)+(1.0f-A)*a;
  return a;
}

double initAngle=0;
int in1 = 10;
int in2 = 11;
int ena = 9;
int in3 = 7;
int in4 = 6;
int enb = 8;
void setup()
{
  
  Setpoint = 0;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-100,100);
  pinMode(ena, OUTPUT);
  pinMode(enb, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  // Arduino initializations
  Wire.begin();
  Serial.begin(19200);
  // Configure gyroscope range
  I2CwriteByte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_250_DPS);
  // Configure accelerometers range
  I2CwriteByte(MPU9250_ADDRESS,28,ACC_FULL_SCALE_2_G);
  // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);
  
  // Request first magnetometer single measurement
  I2CwriteByte(MAG_ADDRESS,0x0A,0x01);
  angle=0.0;
  //enabled?
  int sumAngle=0;
  delay(100);
  for(int i=0;i<10;i++){
    sumAngle+=readAngle();
    delay(1);
  }
  initAngle = sumAngle/10;

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
        for (;;)
            ;
  }
  display.clearDisplay();
  display.setTextColor(WHITE, BLACK);
}


long int cpt=0;
// Main loop, read and display data
int latest=0;
bool blocked=false;
int block_time=0;
void loop()
{
  double angle = readAngle();
  Input = angle;
  if(abs(angle)<10)
  {  //we're close to setpoint, use conservative tuning parameters
    myPID.SetTunings(consKp, consKi, consKd);
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
     myPID.SetTunings(aggKp, aggKi, aggKd);
  }
  myPID.Compute();
  double output = Output;
  
  analogWrite(ena, 155+abs(output));
  analogWrite(enb, 155+abs(output));
  if(output>0){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }else{
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
  
  Serial.println(angle);
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print("     ");
  display.print(angle);
  display.setCursor(0, 18);
  display.print("     ");
  display.print(output);
  display.display();

//  delay(10);    
}
