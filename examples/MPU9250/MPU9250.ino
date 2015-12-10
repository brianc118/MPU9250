#include <SPI.h>
#include <imuFilters.h>
#include <MPU9250.h>

#define SPI_CLOCK 8000000

#define SCK_PIN  14
#define SS_PIN   10 
#define INT_PIN  3
#define LED      13

#define WAITFORINPUT(){            \
	while(!Serial.available()){};  \
	while(Serial.available()){     \
		Serial.read();             \
	};                             \
}                                  \

MPU9250 mpu(SPI_CLOCK, SS_PIN);

imuFilter filter;

void setup() {
	
	Serial.begin(115200);

	pinMode(INT_PIN, INPUT);
	pinMode(LED, OUTPUT);
	digitalWriteFast(LED, HIGH);

	SPI.setSCK(SCK_PIN);
	SPI.begin();
	
	delay(1000);

	mpu.init(true);
	// mpu.set_acc_scale(BITS_FS_2G);
	// mpu.set_gyro_scale(BITS_FS_250DPS);

	WAITFORINPUT();
	
	uint8_t wai = mpu.whoami();
	if (wai == 104){
		Serial.println("Successful connection");
	}
	else{
		Serial.print("Failed connection: ");
		Serial.println(wai, HEX);
	}

	mpu.calib_acc();
	delay(10);
	mpu.AK8963_calib_Magnetometer();

	Serial.print(mpu.Magnetometer_ASA[0]);	Serial.print('\t');
	Serial.print(mpu.Magnetometer_ASA[1]);	Serial.print('\t');
	Serial.print(mpu.Magnetometer_ASA[2]);	Serial.print('\n');

	WAITFORINPUT();
}

void loop() {
	
	mpu.read_all();

	// mpu.Magnetometer[0] -= 1.515;
	// mpu.Magnetometer[1] -= 17.175;
	// mpu.Magnetometer[2] -= -27.39;

	// mpu.Magnetometer[0] /= 89.89;
	// mpu.Magnetometer[1] /= 105.59;
	// mpu.Magnetometer[2] /= 96.12;


	filter.MadgwickQuaternionUpdate(mpu.accelerometer_data[0],
	                                mpu.accelerometer_data[1],
	                                mpu.accelerometer_data[2],
	                                mpu.gyroscope_data[0],
	                                mpu.gyroscope_data[1],
	                                mpu.gyroscope_data[2],
	                                mpu.Magnetometer[0],
	                                mpu.Magnetometer[1],
	                                mpu.Magnetometer[2]
	                                );
	
	// Serial.print((int)(mpu.Magnetometer[0]*100));        Serial.print(',');
	// Serial.print((int)(mpu.Magnetometer[1]*100));        Serial.print(',');
	// Serial.print((int)(mpu.Magnetometer[2]*100));        Serial.print('\n');

	Serial.print(filter.yaw);        Serial.print('\t');
	Serial.print(filter.pitch);      Serial.print('\t');
	Serial.print(filter.roll);       Serial.print('\t');
	
	// Serial.print(mpu.gyroBias[0]);       Serial.print('\t');
	// Serial.print(mpu.gyroBias[1]);       Serial.print('\t');
	// Serial.print(mpu.gyroBias[2]);       Serial.print('\t');

	// Serial.print(mpu.accelBias[0]);       Serial.print('\t');
	// Serial.print(mpu.accelBias[1]);       Serial.print('\t');
	// Serial.print(mpu.accelBias[2]);       Serial.print('\t');

	Serial.print(mpu.accelerometer_data[0]);  Serial.print('\t');
	Serial.print(mpu.accelerometer_data[1]);  Serial.print('\t');
	Serial.print(mpu.accelerometer_data[2]);  Serial.print('\t');
	Serial.print(mpu.gyroscope_data[0]);      Serial.print('\t');
	Serial.print(mpu.gyroscope_data[1]);      Serial.print('\t');
	Serial.print(mpu.gyroscope_data[2]);      Serial.print('\t');
	Serial.print(mpu.Magnetometer[0]);        Serial.print('\t');
	Serial.print(mpu.Magnetometer[1]);        Serial.print('\t');
	Serial.print(mpu.Magnetometer[2]);        Serial.print('\t');
	Serial.println(mpu.Temperature);
	delay(5);
}
