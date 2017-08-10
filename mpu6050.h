#ifndef MPU6050_H
#define MPU6050_H

#include <i2c_t3.h>
#include <elapsedMillis.h>

#define Q_RADDEG    57.29577
#define Q_DEGRAD    0.01745329

class MPU6050 {
	public:
		MPU6050(uint16_t x):samplerate(x){}
		
		void setup();
		void update();
		
		float PITCH, ROLL;
		
	private:
		void writeToMPU(byte address, byte val);
		void readFromMPU();
	
		int32_t gyro_x_cal, gyro_y_cal, gyro_z_cal;

		int32_t gyro_x,	gyro_y,	gyro_z;
		int32_t acc_x,	acc_y,	acc_z;

		int32_t acc_total_vector;
		int32_t still_length_acc;

		float angle_pitch, angle_roll;
		float angle_pitch_output, angle_roll_output;
		float angle_roll_acc, angle_pitch_acc;
		float angle_roll_acc_prev, angle_pitch_acc_prev;

		int16_t temperature;

		bool gyro_angles_set;
		
		uint16_t samplerate;
		
		uint32_t		wait_target;
		elapsedMillis	wait_elapsed;
};
#endif