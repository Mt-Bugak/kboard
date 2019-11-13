#include<Wire.h>

#define MPU6050_AXOFFSET 0
#define MPU6050_AYOFFSET 0
#define MPU6050_AZOFFSET 0
#define MPU6050_GXOFFSET 0
#define MPU6050_GYOFFSET 0
#define MPU6050_GZOFFSET 0
          

long sampling_timer;
const int MPU_addr=0x68;  // I2C address of the MPU-6050

int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ; // Raw data of MPU6050
int16_t AcXi,AcYi,AcZi,GyXi,GyYi,GyZi;
float GAcX, GAcY, GAcZ; // Convert accelerometer to gravity value
float Cal_GyX,Cal_GyY,Cal_GyZ; // Pitch, Roll & Yaw of Gyroscope applied time factor
float acc_pitch, acc_roll, acc_yaw; // Pitch, Roll & Yaw from Accelerometer
float angle_pitch, angle_roll, angle_yaw; // Angle of Pitch, Roll, & Yaw
float alpha = 0.96; // Complementary constant
unsigned long cur = 0;
unsigned long pre = 0;
unsigned long pre2 = 0;
bool LLed = 0;
bool RLed = 0;
int LEDLeft = 7;
int LEDRight = 8;

float flashtime = 0.5; // [sec]
float inittime = 2; // [sec]
float fitchlimit = 4;
float fsrange = 8192.0;

void setup(){
    Wire.begin();
    
    init_MPU6050();
    
    Serial.begin(115200);
    pre = millis();
    pre2 = millis();
    
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
    AcXi=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
    AcYi=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZi=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyXi=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyYi=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZi=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    
    pinMode(12, INPUT_PULLUP);
    pinMode(13, INPUT_PULLUP);
    pinMode(LEDLeft, OUTPUT);
    pinMode(LEDRight, OUTPUT);
}

void loop(){
    cur = millis();
    // Read raw data of MPU6050
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
    AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
    AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

    /*/ Raw data of accelerometer corrected by offset value
    AcX -= AcXi;//MPU6050_AXOFFSET;
    AcY -= AcYi;//MPU6050_AYOFFSET;
    AcZ -= AcZi;//MPU6050_AZOFFSET;
    GyX -= GyXi;
    GyY -= GyYi;
    GyZ -= GyZi;
    /*/

    // Convert accelerometer to gravity value
    GAcX = (float) AcX / fsrange;
    GAcY = (float) AcY / fsrange;
    GAcZ = (float) AcZ / fsrange;

    // Calculate Pitch, Roll & Yaw from Accelerometer value
    // Reference are 
    // https://engineering.stackexchange.com/questions/3348/calculating-pitch-yaw-and-roll-from-mag-acc-and-gyro-data
    // https://www.dfrobot.com/wiki/index.php/How_to_Use_a_Three-Axis_Accelerometer_for_Tilt_Sensing
    acc_pitch = atan ((GAcY - (float)AcYi/fsrange) / sqrt(GAcX * GAcX + GAcZ * GAcZ)) * 57.29577951; // 180 / PI = 57.29577951
    acc_roll = - atan ((GAcX - (float)AcXi/fsrange) / sqrt(GAcY * GAcY + GAcZ * GAcZ)) * 57.29577951; 
    //acc_yaw = atan ((GAcZ - (float)MPU6050_AZOFFSET/fsrange) / sqrt(GAcX * GAcX + GAcZ * GAcZ)) * 57.29577951;
    acc_yaw = atan (sqrt(GAcX * GAcX + GAcZ * GAcZ) / (GAcZ - (float)AcZi/fsrange)) * 57.29577951; 

    // Calculate Pitch, Roll & Yaw from Gyroscope value reflected cumulative time factor
    Cal_GyX += (float)(GyX - GyXi) * 0.000244140625; // 2^15 / 2000 = 16.384, 250Hz, 1 /(250Hz * 16.384LSB)
    Cal_GyY += (float)(GyY - GyYi) * 0.000244140625; // 2^15 / 2000 = 16.384, 250Hz, 1 /(250Hz * 16.384LSB)
    Cal_GyZ += (float)(GyZ - GyZi) * 0.000244140625; // 2^15 / 2000 = 16.384, 250Hz, 1 /(250Hz * 16.384LSB)

    // Calculate Pitch, Roll & Yaw by Complementary Filter
    // Reference is http://www.geekmomprojects.com/gyroscopes-and-accelerometers-on-a-chip/
    // Filtered Angle = α × (Gyroscope Angle) + (1 − α) × (Accelerometer Angle)     
    // where α = τ/(τ + Δt)   and   (Gyroscope Angle) = (Last Measured Filtered Angle) + ω×Δt
    // Δt = sampling rate, τ = time constant greater than timescale of typical accelerometer noise
    angle_pitch = alpha * (((float)(GyX - GyXi) * 0.000244140625) + angle_pitch) + (1 - alpha) * acc_pitch;
    angle_roll = alpha * (((float)(GyY - GyYi) * 0.000244140625) + angle_roll) + (1 - alpha) * acc_roll;
    angle_yaw += (float)(GyZ - GyZi) * 0.000244140625; // Accelerometer doesn't have yaw value
    
    // Print raw of accelerometer & gyroscope reflected cumulative time factor
    //  Serial.print("AcX = "); Serial.print(AcX);
    //  Serial.print(" | AcY = "); Serial.print(AcY);
    //  Serial.print(" | AcZ = "); Serial.println(AcZ);
    //  Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
    //  Serial.print(" | Cal_GyX = "); Serial.print(Cal_GyX);
    //  Serial.print(" | Cal_GyY = "); Serial.print(Cal_GyY);
    //  Serial.print(" | Cal_GyZ = "); Serial.println(Cal_GyZ);

    // Print calculated value of Pitch, Roll & Yaw from Accelerometer value
    //  Serial.print(" | acc_pitch = "); Serial.print(acc_pitch);
    //  Serial.print(" | acc_roll = "); Serial.print(acc_roll);
    //  Serial.print(" | acc_yaw = "); Serial.println(acc_yaw);

    // Print value of Pitch, Roll & Yaw reflected Complementary Filter
//    Serial.print(cur);
//    Serial.print(" | angle_yaw = "); Serial.print(angle_yaw);
//    Serial.print("\t | angle_pitch = ");Serial.print(angle_pitch);
//    Serial.print("\t | angle_roll = "); Serial.println(angle_roll);

    Serial.print(" | Switch = ");
    if(digitalRead(12)==0&&digitalRead(13)==0){
      Serial.print("L");
    }
    else if(digitalRead(12)==1&&digitalRead(13)==0){
      Serial.print("N");
    }
    else if(digitalRead(12)==1&&digitalRead(13)==1){
      Serial.print("R");
    }
    Serial.print(" | angle_pitch = ");Serial.print(angle_pitch);
    //Serial.print(" | angle_roll = "); Serial.print(angle_roll);
    //Serial.print(" | angle = "); Serial.print(angle_yaw);
          
    if(angle_pitch>-fitchlimit && angle_pitch<fitchlimit) pre2 = cur;

    if(digitalRead(12)==0&&digitalRead(13)==0){ //Switch L
        RLed = 0;
        digitalWrite(LEDRight, LOW);
        if(cur - pre > flashtime*1000){
            pre=cur;
            if (LLed==0) {
                LLed = 1;
                digitalWrite(LEDLeft, HIGH);
            }
            else {
                LLed = 0;
                digitalWrite(LEDLeft, LOW);
            }
        }
    }
    
    else if(digitalRead(12)==1&&digitalRead(13)==0){ //Switch N
        if(cur - pre > flashtime*1000){
            pre=cur;
            if(angle_pitch>fitchlimit && cur - pre2 > inittime*1000){ //Sensor R
                LLed = 0;
                digitalWrite(LEDLeft, LOW);
                if (RLed==0) {
                    RLed = 1;
                    digitalWrite(LEDRight, HIGH);
                }
                else {
                    RLed = 0;
                    digitalWrite(LEDRight, LOW);
                }
            }
            else if(angle_pitch<-fitchlimit && cur - pre2 > inittime*1000){ //Sensor L
                RLed = 0;
                digitalWrite(LEDRight, LOW);
                if (LLed==0) {
                    LLed = 1;
                    digitalWrite(LEDLeft, HIGH);
                }
                else {
                    LLed = 0;
                    digitalWrite(LEDLeft, LOW);
                }
            }
            else{ //LED OFF
                LLed = 0;
                RLed = 0;
                digitalWrite(LEDLeft, LOW);
                digitalWrite(LEDRight, LOW);
            }
        }
    }
    
    else if(digitalRead(12)==1&&digitalRead(13)==1){ //Switch R
        LLed = 0;
        digitalWrite(LEDLeft, LOW);
        if(cur - pre > flashtime*1000){
            pre=cur;
            if (RLed==0) {
                RLed = 1;
                digitalWrite(LEDRight, HIGH);
            }
            else {
                RLed = 0;
                digitalWrite(LEDRight, LOW);
            }
        }
    }
    

    Serial.print(" | LLed = ");Serial.print(LLed);
    Serial.print(" | RLed = ");Serial.println(RLed);
    
    // Sampling Timer
    //while(micros() - sampling_timer < 100000); //
    //sampling_timer = micros(); //Reset the sampling timer  

}

void init_MPU6050(){
    //MPU6050 Initializing & Reset
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B);  // PWR_MGMT_1 register
    Wire.write(0);     // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);

    //MPU6050 Clock Type
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B);  // PWR_MGMT_1 register
    Wire.write(0x03);     // Selection Clock 'PLL with Z axis gyroscope reference'
    Wire.endTransmission(true);

    //MPU6050 Gyroscope Configuration Setting
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x1B);  // Gyroscope Configuration register
    //Wire.write(0x00);     // FS_SEL=0, Full Scale Range = +/- 250 [degree/sec]
    Wire.write(0x08);     // FS_SEL=1, Full Scale Range = +/- 500 [degree/sec]
    //Wire.write(0x10);     // FS_SEL=2, Full Scale Range = +/- 1000 [degree/sec]
    //Wire.write(0x18);     // FS_SEL=3, Full Scale Range = +/- 2000 [degree/sec]
    Wire.endTransmission(true);

    //MPU6050 Accelerometer Configuration Setting
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x1C);  // Accelerometer Configuration register
    Wire.write(0x00);     // AFS_SEL=0, Full Scale Range = +/- 2 [g]
    //Wire.write(0x08);     // AFS_SEL=1, Full Scale Range = +/- 4 [g]
    //Wire.write(0x10);     // AFS_SEL=2, Full Scale Range = +/- 8 [g]
    //Wire.write(0x18);     // AFS_SEL=3, Full Scale Range = +/- 10 [g]
    Wire.endTransmission(true);

    //MPU6050 DLPF(Digital Low Pass Filter)
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x1A);  // DLPF_CFG register
    Wire.write(0x00);     // Accel BW 260Hz, Delay 0ms / Gyro BW 256Hz, Delay 0.98ms, Fs 8KHz 
    //Wire.write(0x01);     // Accel BW 184Hz, Delay 2ms / Gyro BW 188Hz, Delay 1.9ms, Fs 1KHz 
    //Wire.write(0x02);     // Accel BW 94Hz, Delay 3ms / Gyro BW 98Hz, Delay 2.8ms, Fs 1KHz 
    //Wire.write(0x03);     // Accel BW 44Hz, Delay 4.9ms / Gyro BW 42Hz, Delay 4.8ms, Fs 1KHz 
    //Wire.write(0x04);     // Accel BW 21Hz, Delay 8.5ms / Gyro BW 20Hz, Delay 8.3ms, Fs 1KHz 
    //Wire.write(0x05);     // Accel BW 10Hz, Delay 13.8ms / Gyro BW 10Hz, Delay 13.4ms, Fs 1KHz 
    //Wire.write(0x06);     // Accel BW 5Hz, Delay 19ms / Gyro BW 5Hz, Delay 18.6ms, Fs 1KHz 
    Wire.endTransmission(true);
}
