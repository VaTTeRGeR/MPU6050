#include <mpu6050.h>

void MPU6050::setup() {
  Wire.begin(I2C_MASTER, 0, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);    //Start I2C as master
    
  writeToMPU(107, B00000001);    //  no sleep and clock off gyro_X
  writeToMPU(108, B00000000);    //  no goofball sleep mode
  
  writeToMPU( 25, byte((8000 / (samplerate << 1)) - 1)); //  sample rate divider: sample rate = mstrClock / (1 +  divider)
  writeToMPU( 26, B00000000);            //  DLPF set.  (0 = 8kHz master clock else 1kHz master clock)

  writeToMPU( 27, 0x18);         //  2000 deg/s gyro
  writeToMPU( 28, 0x10);         //  16g accelerometer
  
  writeToMPU( 31, B00000000);    //  no motion detect
  writeToMPU( 35, B00000000);    //  no FIFO, just registers
  writeToMPU( 36, B00000000);    //  no master I2C
  writeToMPU( 55, B01110000);    //  configure interrupts
  writeToMPU( 56, B00000000);    //  disable interrupts
  writeToMPU(106, B00000000);    //  no silly stuff

  int32_t	cal_rounds	= 0;
  uint32_t	t_cal_start	= millis();
  
  while((millis() - t_cal_start < 500 && cal_rounds < 2048) || cal_rounds == 0) {
    readFromMPU();
    
    gyro_x_cal += gyro_x;
	gyro_y_cal += gyro_y;
    gyro_z_cal += gyro_z;
	
    still_length_acc += sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));

    cal_rounds++;
  }
  
  gyro_x_cal /= cal_rounds;
  gyro_y_cal /= cal_rounds;
  gyro_z_cal /= cal_rounds;
  
  still_length_acc /= cal_rounds;

  wait_target	= 1000/samplerate;
  wait_elapsed	= 0;
}

void MPU6050::update() {
  if(wait_elapsed >= wait_target) {

    wait_elapsed -= wait_target;
    
    readFromMPU();
  
    gyro_x -= gyro_x_cal;
    gyro_y -= gyro_y_cal;
    gyro_z -= gyro_z_cal;
  
    //float gyroToAngleChange = 1.0/((float)SAMPLERATE)/65.5;            // for 500 deg/s
    float gyroToAngleChange = 1.0/((float)samplerate)/16.375;            // for 2000 deg/s
  
    //Gyro angle calculations
    angle_pitch += ((float)gyro_x) * gyroToAngleChange;                  //Calculate the traveled pitch angle and add this to the angle_pitch variable
    angle_roll += ((float)gyro_y) * gyroToAngleChange;                   //Calculate the traveled roll angle and add this to the angle_roll variable
    
    float gyrozScl = gyroToAngleChange * Q_DEGRAD;                       //The Arduino sin function is in radians
    angle_pitch += angle_roll * sin(gyro_z * gyrozScl);                  //If the IMU has yawed transfer the roll angle to the pitch angel
    angle_roll -= angle_pitch * sin(gyro_z * gyrozScl);                  //If the IMU has yawed transfer the pitch angle to the roll angel
    
    //Accelerometer angle calculations
    acc_total_vector = sqrt( (acc_x*acc_x) + (acc_y*acc_y) + (acc_z*acc_z) ); //Calculate the total accelerometer vector
    angle_pitch_acc = asin((float)acc_y / acc_total_vector)*  -Q_RADDEG;    //Calculate the pitch angle
    angle_roll_acc = asin((float)acc_x / acc_total_vector) *  Q_RADDEG;    //Calculate the roll angle
    
    //Offset
    //angle_pitch_acc -= 0.0;                                              //Accelerometer calibration value for pitch
    //angle_roll_acc -= 0.0;                                               //Accelerometer calibration value for roll
  
    if(!gyro_angles_set){                                                //If the IMU is not already started
      angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle 
      angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle 
      gyro_angles_set = true;                                            //Set the IMU started flag
    } else {
		float mix, mix_i;
		
		float d_pitch = angle_pitch_acc - angle_pitch;
		float d_roll  = angle_roll_acc  - angle_roll;
		float d_max = max(abs(d_pitch),abs(d_roll));
		
		mix = -d_max*(0.01/20.0)+0.01;
		mix = constrain(mix, 0.0001, 0.01);
		mix_i = 1.0 - mix;
		
		angle_pitch	= angle_pitch * mix_i + angle_pitch_acc * mix;       //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
		angle_roll	= angle_roll  * mix_i + angle_roll_acc  * mix;          //Correct the drift of the gyro roll angle with the accelerometer roll angle
		
		/*Serial.print("ar ");
		Serial.println(angle_roll_acc,1);
		Serial.print("ap ");
		Serial.println(angle_pitch_acc,1);
		Serial.print("d ");
		Serial.println(d_max,1);
		Serial.print("m ");
		Serial.println(mix,4);*/
    }
  
	angle_pitch_acc_prev = angle_pitch_acc;
	angle_roll_acc_prev = angle_roll_acc;
    
    //To dampen the pitch and roll angles a complementary filter is used
    PITCH	=	angle_pitch_output	=	-1.0*(angle_pitch_output	* 0.25 + angle_pitch	* 0.75);   //Take 50% of the output pitch value and add 50% of the raw pitch value
    ROLL	=	angle_roll_output	=	angle_roll_output	* 0.25 + angle_roll		* 0.75;      //Take 50% of the output roll value and add 50% of the raw roll value
  
    //PITCH	=	angle_pitch_output	=	angle_pitch;
    //ROLL	=	angle_roll_output	=	angle_roll;
  }
}

void MPU6050::writeToMPU(byte address, byte val) {
	Wire.beginTransmission(0x68);
	Wire.write(address);
	Wire.write(val);
	Wire.endTransmission();
}

void MPU6050::readFromMPU() {
	Wire.beginTransmission(0x68);
	Wire.write(59);
	Wire.endTransmission();

	Wire.requestFrom(0x68, 14, true);

	while(Wire.available()<14);
    
	//READS IN THIS ORDER: AX, AY, AZ | TEMP | GX, GY, GZ
  
	acc_y = (int16_t)(Wire.read()<<8|Wire.read());                                  //Add the low and high byte to the acc_x variable
	acc_x = (int16_t)(Wire.read()<<8|Wire.read());                                  //Add the low and high byte to the acc_y variable
	acc_z = (int16_t)(Wire.read()<<8|Wire.read());                                  //Add the low and high byte to the acc_z variable

	temperature = (int16_t)(Wire.read()<<8|Wire.read());                            //Add the low and high byte to the temperature variable
  
	gyro_y = (int16_t)(Wire.read()<<8|Wire.read());                                 //Add the low and high byte to the gyro_x variable
	gyro_x = (int16_t)(Wire.read()<<8|Wire.read());                                 //Add the low and high byte to the gyro_y variable
	gyro_z = (int16_t)(Wire.read()<<8|Wire.read());                                 //Add the low and high byte to the gyro_z variable
}