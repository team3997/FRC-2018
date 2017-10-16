/**
 * 
 */
package org.usfirst.frc.team3997.robot.hardware;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.livewindow.LiveWindowSendable;
import edu.wpi.first.wpilibj.*;

/**
 * @author peter
 *
 */
public class MPU9250Gyro {

	// ==============================================================================
	// ====== Set of useful function to access acceleration. gyroscope,
	// magnetometer,
	// ====== and temperature data
	// ==============================================================================
	I2C comms;

	// See also MPU-9250 Register Map and Descriptions, Revision 4.0,
	// RM-MPU-9250A-00, Rev. 1.4, 9/9/2013 for registers not listed in above
	// document; the MPU9250 and MPU9150 are virtually identical but the latter
	// has
	// a different register map

	// Magnetometer Registers
	int AK8963_ADDRESS = 0x0C;
	int WHO_AM_I_AK8963 = 0x49; // (AKA WIA) should return 0x48
	int INFO = 0x01;
	int AK8963_ST1 = 0x02; // data ready status bit 0
	int AK8963_XOUT_L = 0x03; // data
	int AK8963_XOUT_H = 0x04;
	int AK8963_YOUT_L = 0x05;
	int AK8963_YOUT_H = 0x06;
	int AK8963_ZOUT_L = 0x07;
	int AK8963_ZOUT_H = 0x08;
	int AK8963_ST2 = 0x09; // Data overflow bit 3 and data read error status bit
							// 2
	int AK8963_CNTL = 0x0A; // Power down (0000), single-measurement (0001),
							// self-test (1000) and Fuse ROM (1111) modes on
							// bits 3:0
	int AK8963_ASTC = 0x0C; // Self test control
	int AK8963_I2CDIS = 0x0F; // I2C disable
	int AK8963_ASAX = 0x10; // Fuse ROM x-axis sensitivity adjustment value
	int AK8963_ASAY = 0x11; // Fuse ROM y-axis sensitivity adjustment value
	int AK8963_ASAZ = 0x12; // Fuse ROM z-axis sensitivity adjustment value

	int SELF_TEST_X_GYRO = 0x00;
	int SELF_TEST_Y_GYRO = 0x01;
	int SELF_TEST_Z_GYRO = 0x02;

	/*
	 * int X_FINE_GAIN 0x03 // [7:0] fine gain int Y_FINE_GAIN 0x04 int
	 * Z_FINE_GAIN 0x05 int XA_OFFSET_H 0x06 // User-defined trim values for
	 * accelerometer int XA_OFFSET_L_TC 0x07 int YA_OFFSET_H 0x08 int
	 * YA_OFFSET_L_TC 0x09 int ZA_OFFSET_H 0x0A int ZA_OFFSET_L_TC 0x0B
	 */

	int SELF_TEST_X_ACCEL = 0x0D;
	int SELF_TEST_Y_ACCEL = 0x0E;
	int SELF_TEST_Z_ACCEL = 0x0F;

	int SELF_TEST_A = 0x10;

	int XG_OFFSET_H = 0x13; // User-defined trim values for gyroscope
	int XG_OFFSET_L = 0x14;
	int YG_OFFSET_H = 0x15;
	int YG_OFFSET_L = 0x16;
	int ZG_OFFSET_H = 0x17;
	int ZG_OFFSET_L = 0x18;
	int SMPLRT_DIV = 0x19;
	int CONFIG = 0x1A;
	int GYRO_CONFIG = 0x1B;
	int ACCEL_CONFIG = 0x1C;
	int ACCEL_CONFIG2 = 0x1D;
	int LP_ACCEL_ODR = 0x1E;
	int WOM_THR = 0x1F;

	// Duration counter threshold for motion interrupt generation, 1 kHz rate,
	// LSB = 1 ms
	int MOT_DUR = 0x20;
	// Zero-motion detection threshold bits [7:0]
	int ZMOT_THR = 0x21;
	// Duration counter threshold for zero motion interrupt generation, 16 Hz
	// rate,
	// LSB = 64 ms
	int ZRMOT_DUR = 0x22;

	int FIFO_EN = 0x23;
	int I2C_MST_CTRL = 0x24;
	int I2C_SLV0_ADDR = 0x25;
	int I2C_SLV0_REG = 0x26;
	int I2C_SLV0_CTRL = 0x27;
	int I2C_SLV1_ADDR = 0x28;
	int I2C_SLV1_REG = 0x29;
	int I2C_SLV1_CTRL = 0x2A;
	int I2C_SLV2_ADDR = 0x2B;
	int I2C_SLV2_REG = 0x2C;
	int I2C_SLV2_CTRL = 0x2D;
	int I2C_SLV3_ADDR = 0x2E;
	int I2C_SLV3_REG = 0x2F;
	int I2C_SLV3_CTRL = 0x30;
	int I2C_SLV4_ADDR = 0x31;
	int I2C_SLV4_REG = 0x32;
	int I2C_SLV4_DO = 0x33;
	int I2C_SLV4_CTRL = 0x34;
	int I2C_SLV4_DI = 0x35;
	int I2C_MST_STATUS = 0x36;
	int INT_PIN_CFG = 0x37;
	int INT_ENABLE = 0x38;
	int DMP_INT_STATUS = 0x39; // Check DMP interrupt
	int INT_STATUS = 0x3A;
	int ACCEL_XOUT_H = 0x3B;
	int ACCEL_XOUT_L = 0x3C;
	int ACCEL_YOUT_H = 0x3D;
	int ACCEL_YOUT_L = 0x3E;
	int ACCEL_ZOUT_H = 0x3F;
	int ACCEL_ZOUT_L = 0x40;
	int TEMP_OUT_H = 0x41;
	int TEMP_OUT_L = 0x42;
	int GYRO_XOUT_H = 0x43;
	int GYRO_XOUT_L = 0x44;
	int GYRO_YOUT_H = 0x45;
	int GYRO_YOUT_L = 0x46;
	int GYRO_ZOUT_H = 0x47;
	int GYRO_ZOUT_L = 0x48;
	int EXT_SENS_DATA_00 = 0x49;
	int EXT_SENS_DATA_01 = 0x4A;
	int EXT_SENS_DATA_02 = 0x4B;
	int EXT_SENS_DATA_03 = 0x4C;
	int EXT_SENS_DATA_04 = 0x4D;
	int EXT_SENS_DATA_05 = 0x4E;
	int EXT_SENS_DATA_06 = 0x4F;
	int EXT_SENS_DATA_07 = 0x50;
	int EXT_SENS_DATA_08 = 0x51;
	int EXT_SENS_DATA_09 = 0x52;
	int EXT_SENS_DATA_10 = 0x53;
	int EXT_SENS_DATA_11 = 0x54;
	int EXT_SENS_DATA_12 = 0x55;
	int EXT_SENS_DATA_13 = 0x56;
	int EXT_SENS_DATA_14 = 0x57;
	int EXT_SENS_DATA_15 = 0x58;
	int EXT_SENS_DATA_16 = 0x59;
	int EXT_SENS_DATA_17 = 0x5A;
	int EXT_SENS_DATA_18 = 0x5B;
	int EXT_SENS_DATA_19 = 0x5C;
	int EXT_SENS_DATA_20 = 0x5D;
	int EXT_SENS_DATA_21 = 0x5E;
	int EXT_SENS_DATA_22 = 0x5F;
	int EXT_SENS_DATA_23 = 0x60;
	int MOT_DETECT_STATUS = 0x61;
	int I2C_SLV0_DO = 0x63;
	int I2C_SLV1_DO = 0x64;
	int I2C_SLV2_DO = 0x65;
	int I2C_SLV3_DO = 0x66;
	int I2C_MST_wait_CTRL = 0x67;
	int SIGNAL_PATH_RESET = 0x68;
	int MOT_DETECT_CTRL = 0x69;
	int USER_CTRL = 0x6A; // Bit 7 enable DMP, bit 3 reset DMP
	int PWR_MGMT_1 = 0x6B; // Device defaults to the SLEEP mode
	int PWR_MGMT_2 = 0x6C;
	int DMP_BANK = 0x6D; // Activates a specific bank in the DMP
	int DMP_RW_PNT = 0x6E; // Set read/write pointer to a specific start address
							// in specified DMP bank
	int DMP_REG = 0x6F; // Register in DMP from which to read or to which to
						// write
	int DMP_REG_1 = 0x70;
	int DMP_REG_2 = 0x71;
	int FIFO_COUNTH = 0x72;
	int FIFO_COUNTL = 0x73;
	int FIFO_R_W = 0x74;
	int WHO_AM_I_MPU9250 = 0x75; // Should return 0x71
	int XA_OFFSET_H = 0x77;
	int XA_OFFSET_L = 0x78;
	int YA_OFFSET_H = 0x7A;
	int YA_OFFSET_L = 0x7B;
	int ZA_OFFSET_H = 0x7D;
	int ZA_OFFSET_L = 0x7E;

	// Using the MPU-9250 breakout board, ADO is set to 0
	// Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1
	int ADO = 0;

	int READ_FLAG = 0x80;
	int NOT_SPI = -1;
	int SPI_DATA_RATE = 1000000; // 1MHz is the max speed of the MPU-9250
	// int SPI_DATA_RATE 1000000 // 1MHz is the max speed of the MPU-9250

	// Set initial input parameters
	protected enum Ascale
	{
	      AFS_2G = 0,
	      AFS_4G,
	      AFS_8G,
	      AFS_16G
	    };

	protected enum Gscale
	{
	      GFS_250DPS = 0,
	      GFS_500DPS,
	      GFS_1000DPS,
	      GFS_2000DPS
	    };

	protected enum Mscale
	{
	      MFS_14BITS = 0, // 0.6 mG per LSB
	      MFS_16BITS      // 0.15 mG per LSB
	    };

	protected enum M_MODE
	{
	      M_8HZ = 0x02,  // 8 Hz update
	      M_100HZ = 0x06 // 100 Hz continuous magnetometer
	    };

	// TODO: Add setter methods for this hard coded stuff
	// Specify sensor full scale
	protected int Gscale = GFS_250DPS;
	protected int Ascale = AFS_2G;
	// Choose either 14-bit or 16-bit magnetometer resolution
	protected int Mscale = MFS_16BITS;

	// 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
	protected int Mmode = M_8HZ;

	// SPI chip select pin
	protected int _csPin;

	public float pitch, yaw, roll;
	public float temperature; // Stores the real internal chip temperature in
	// Celsius
	public int tempCount; // Temperature raw count output
	public int delt_t = 0; // Used to control display output rate

	public int count = 0, sumCount = 0; // used to control display output rate
	float deltat = 0.0f, sum = 0.0f; // integration interval for both filter
										// schemes
	public int lastUpdate = 0, firstUpdate = 0; // used to calculate
												// integration interval
	public int Now = 0; // used to calculate integration interval

	public int gyroCount[3]; // Stores the 16-bit signed gyro sensor output
	public int magCount[3]; // Stores the 16-bit signed magnetometer sensor
							// output
	// Scale resolutions per LSB for the sensors
	public float aRes, igRes, mRes;
	// Variables to hold latest sensor data values
	public float ax, ay, az, gx, gy, gz, mx, my, mz;
	// Factory mag calibration and mag bias
	public float factoryMagCalibration[3]=
	{0, 0, 0},factoryMagBias[3]=
	{0, 0, 0};
	// Bias corrections for gyro, accelerometer, and magnetometer
	public float gyroBias[3]=
	{0, 0, 0},accelBias[3]=
	{0, 0, 0},magBias[3]=
	{0, 0, 0},magScale[3]=
	{0, 0, 0};float selfTest[6];
	// Stores the 16-bit signed accelerometer sensor output
	int accelCount[3];

	bool isInI2cMode() {
		return _csPin == -1;
	}

	MPU9250Gyro(I2CPort port, int deviceAddress) // Uses I2C communication by
													// default
	{

		comms = new I2C(port, deviceAddress);

	}

	void getMres() {
		switch (Mscale) {
		// Possible magnetometer scales (and their register bit settings)
		// are:
		// 14 bit resolution (0) and 16 bit resolution (1)
		case MFS_14BITS:
			mRes = 10.0f * 4912.0f / 8190.0f; // Proper scale to return
												// milliGauss
			break;
		case MFS_16BITS:
			mRes = 10.0f * 4912.0f / 32760.0f; // Proper scale to return
												// milliGauss
			break;
		}
	}

	void getGres() {
		switch (Gscale) {
		// Possible gyro scales (and their register bit settings) are:
		// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS (11).
		// Here's a bit of an algorith to calculate DPS/(ADC tick) based on
		// that
		// 2-bit value:
		case GFS_250DPS:
			gRes = 250.0f / 32768.0f;
			break;
		case GFS_500DPS:
			gRes = 500.0f / 32768.0f;
			break;
		case GFS_1000DPS:
			gRes = 1000.0f / 32768.0f;
			break;
		case GFS_2000DPS:
			gRes = 2000.0f / 32768.0f;
			break;
		}
	}

	void getAres() {
		switch (Ascale) {
		// Possible accelerometer scales (and their register bit settings)
		// are:
		// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs (11).
		// Here's a bit of an algorith to calculate DPS/(ADC tick) based on
		// that
		// 2-bit value:
		case AFS_2G:
			aRes = 2.0f / 32768.0f;
			break;
		case AFS_4G:
			aRes = 4.0f / 32768.0f;
			break;
		case AFS_8G:
			aRes = 8.0f / 32768.0f;
			break;
		case AFS_16G:
			aRes = 16.0f / 32768.0f;
			break;
		}
	}

	void readAccelData(int destination)
	{
	  int rawData[6];  // x/y/z accel register data stored here
	  // Read the six raw data registers into data array
	  readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);

	  // Turn the MSB and LSB into a signed 16-bit value
	  destination[0] = ((int)rawData[0] << 8) | rawData[1] ;
	  destination[1] = ((int)rawData[2] << 8) | rawData[3] ;
	  destination[2] = ((int)rawData[4] << 8) | rawData[5] ;
	}

	void readGyroData(int destination)
	{
	  int rawData[6];  // x/y/z gyro register data stored here
	  // Read the six raw data registers sequentially into data array
	  readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);

	  // Turn the MSB and LSB into a signed 16-bit value
	  destination[0] = ((int)rawData[0] << 8) | rawData[1] ;
	  destination[1] = ((int)rawData[2] << 8) | rawData[3] ;
	  destination[2] = ((int)rawData[4] << 8) | rawData[5] ;
	}

	void readMagData(int destination)
	{
	  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end
	  // of data acquisition
	  int rawData[7];
	  // Wait for magnetometer data ready bit to be set
	  if (readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01)
	  {
	    // Read the six raw data and ST2 registers sequentially into data array
	    readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);
	    int c = rawData[6]; // End data read by reading ST2 register
	    // Check if magnetic sensor overflow set, if not then report data
	    if (!(c & 0x08))
	    {
	      // Turn the MSB and LSB into a signed 16-bit value
	      destination[0] = ((int)rawData[1] << 8) | rawData[0];
	      // Data stored as little Endian
	      destination[1] = ((int)rawData[3] << 8) | rawData[2];
	      destination[2] = ((int)rawData[5] << 8) | rawData[4];
	    }
	  }
	}

	int readTempData() {
			int rawData[2]; // x/y/z gyro register data stored here
			// Read the two raw data registers sequentially into data array
			readBytes(MPU9250_ADDRESS,TEMP_OUT_H,2,rawData[0]);
			// Turn the MSB and LSB into a 16-bit value
			return((int)rawData[0]<<8)|rawData[1];
		}

	// Calculate the time the last update took for use in the quaternion
	// filters
	// TODO: This doesn't really belong in this class.
	void updateTime() {
		Now = micros();

		// Set integration time by time elapsed since last filter update
		deltat = ((Now - lastUpdate) / 1000000.0f);
		lastUpdate = Now;

		sum += deltat; // sum for averaging filter update rate
		sumCount++;
	}

	void initAK8963(float destination)
	{
	  // First extract the factory calibration for each magnetometer axis
	  int rawData[3];  // x/y/z gyro calibration data stored here
	  // TODO: Test this!! Likely doesn't work
	  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
	  wait(10);
	  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
	  wait(10);
	  
	  // Read the x-, y-, and z-axis calibration values
	  readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);

	  // Return x-axis sensitivity adjustment values, etc.
	  destination[0] =  (float)(rawData[0] - 128)/256. + 1.;
	  destination[1] =  (float)(rawData[1] - 128)/256. + 1.;
	  destination[2] =  (float)(rawData[2] - 128)/256. + 1.;
	  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
	  wait(10);

	  // Configure the magnetometer for continuous read and highest resolution.
	  // Set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL
	  // register, and enable continuous mode data acquisition Mmode (bits [3:0]),
	  // 0010 for 8 Hz and 0110 for 100 Hz sample rates.

	  // Set magnetometer data resolution and sample ODR
	  writeByte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode);
	  wait(10);
	}

	void initMPU9250() {
		// wake up device
		// Clear sleep mode bit (6), enable all sensors
		writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);
		wait(100); // Wait for all registers to reset

		// Get stable time source
		// Auto select clock source to be PLL gyroscope reference if ready
		// else
		writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
		wait(200);

		// Configure Gyro and Thermometer
		// Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42
		// Hz,
		// respectively;
		// minimum wait time for this setting is 5.9 ms, which means sensor
		// fusion
		// update rates cannot be higher than 1 / 0.0059 = 170 Hz
		// DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz
		// for both
		// With the MPU9250, it is possible to get gyro sample rates of 32
		// kHz (!),
		// 8 kHz, or 1 kHz
		writeByte(MPU9250_ADDRESS, CONFIG, 0x03);

		// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
		// Use a 200 Hz rate; a rate consistent with the filter update rate
		// determined inset in CONFIG above.
		writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);

		// Set gyroscope full scale range
		// Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are
		// left-shifted into positions 4:3

		// get current GYRO_CONFIG register value
		int c = readByte(MPU9250_ADDRESS, GYRO_CONFIG);
		// c = c & ~0xE0; // Clear self-test bits [7:5]
		c = c & ~0x02; // Clear Fchoice bits [1:0]
		c = c & ~0x18; // Clear AFS bits [4:3]
		c = c | Gscale << 3; // Set full scale range for the gyro
		// Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0
		// of
		// GYRO_CONFIG
		// c =| 0x00;
		// Write new GYRO_CONFIG value to register
		writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c);

		// Set accelerometer full-scale range configuration
		// Get current ACCEL_CONFIG register value
		c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG);
		// c = c & ~0xE0; // Clear self-test bits [7:5]
		c = c & ~0x18; // Clear AFS bits [4:3]
		c = c | Ascale << 3; // Set full scale range for the accelerometer
		// Write new ACCEL_CONFIG register value
		writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c);

		// Set accelerometer sample rate configuration
		// It is possible to get a 4 kHz sample rate from the accelerometer
		// by
		// choosing 1 for accel_fchoice_b bit [3]; in this case the
		// bandwidth is
		// 1.13 kHz
		// Get current ACCEL_CONFIG2 register value
		c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2);
		c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits
						// [2:0])
		c = c | 0x03; // Set accelerometer rate to 1 kHz and bandwidth to 41
						// Hz
		// Write new ACCEL_CONFIG2 register value
		writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c);
		// The accelerometer, gyro, and thermometer are set to 1 kHz sample
		// rates,
		// but all these rates are further reduced by a factor of 5 to 200
		// Hz because
		// of the SMPLRT_DIV setting

		// Configure Interrupts and Bypass Enable
		// Set interrupt pin active high, push-pull, hold interrupt pin
		// level HIGH
		// until interrupt cleared, clear on read of INT_STATUS, and enable
		// I2C_BYPASS_EN so additional chips can join the I2C bus and all
		// can be
		// controlled by the Arduino as master.
		writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
		// Enable data ready (bit 0) interrupt
		writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);
		wait(100);
	}

	// Function which accumulates gyro and accelerometer data after device
	// initialization. It calculates the average of the at-rest readings and
	// then
	// loads the resulting offsets into accelerometer and gyro bias
	// registers.
	void calibrateMPU9250(float gyroBias, float accelBias)
	{
	  int data[12]; // data array to hold accelerometer and gyro x, y, z, data
	  int ii, packet_count, fifo_count;
	  int gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

	  // reset device
	  // Write a one to bit 7 reset bit; toggle reset device
	  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, READ_FLAG);
	  wait(100);

	  // get stable time source; Auto select clock source to be PLL gyroscope
	  // reference if ready else use the internal oscillator, bits 2:0 = 001
	  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
	  writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
	  wait(200);

	  // Configure device for bias calculation
	  // Disable all interrupts
	  writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x00);
	  // Disable FIFO
	  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);
	  // Turn on internal clock source
	  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);
	  // Disable I2C master
	  writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00);
	  // Disable FIFO and I2C master modes
	  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x00);
	  // Reset FIFO and DMP
	  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x0C);
	  wait(15);

	  // Configure MPU6050 gyro and accelerometer for bias calculation
	  // Set low-pass filter to 188 Hz
	  writeByte(MPU9250_ADDRESS, CONFIG, 0x01);
	  // Set sample rate to 1 kHz
	  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);
	  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);
	  // Set accelerometer full-scale to 2 g, maximum sensitivity
	  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);

	  int  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
	  int  accelsensitivity = 16384; // = 16384 LSB/g

	  // Configure FIFO to capture accelerometer and gyro data for bias calculation
	  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x40);  // Enable FIFO
	  // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in
	  // MPU-9150)
	  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x78);
	  wait(40);  // accumulate 40 samples in 40 milliseconds = 480 bytes

	  // At end of sample accumulation, turn off FIFO sensor read
	  // Disable gyro and accelerometer sensors for FIFO
	  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);
	  // Read FIFO sample count
	  readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]);
	  fifo_count = ((int)data[0] << 8) | data[1];
	  // How many sets of full gyro and accelerometer data for averaging
	  packet_count = fifo_count/12;

	  for (ii = 0; ii < packet_count; ii++)
	  {
	    int accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
	    // Read data for averaging
	    readBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]);
	    // Form signed 16-bit integer for each sample in FIFO
	    accel_temp[0] = (int) (((int)data[0] << 8) | data[1]  );
	    accel_temp[1] = (int) (((int)data[2] << 8) | data[3]  );
	    accel_temp[2] = (int) (((int)data[4] << 8) | data[5]  );
	    gyro_temp[0]  = (int) (((int)data[6] << 8) | data[7]  );
	    gyro_temp[1]  = (int) (((int)data[8] << 8) | data[9]  );
	    gyro_temp[2]  = (int) (((int)data[10] << 8) | data[11]);

	    // Sum individual signed 16-bit biases to get accumulated signed 32-bit
	    // biases.
	    accel_bias[0] += (int) accel_temp[0];
	    accel_bias[1] += (int) accel_temp[1];
	    accel_bias[2] += (int) accel_temp[2];
	    gyro_bias[0]  += (int) gyro_temp[0];
	    gyro_bias[1]  += (int) gyro_temp[1];
	    gyro_bias[2]  += (int) gyro_temp[2];
	  }
	  // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
	  accel_bias[0] /= (int) packet_count;
	  accel_bias[1] /= (int) packet_count;
	  accel_bias[2] /= (int) packet_count;
	  gyro_bias[0]  /= (int) packet_count;
	  gyro_bias[1]  /= (int) packet_count;
	  gyro_bias[2]  /= (int) packet_count;

	  // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
	  if (accel_bias[2] > 0L)
	  {
	    accel_bias[2] -= (int) accelsensitivity;
	  }
	  else
	  {
	    accel_bias[2] += (int) accelsensitivity;
	  }

	  // Construct the gyro biases for push to the hardware gyro bias registers,
	  // which are reset to zero upon device startup.
	  // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input
	  // format.
	  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF;
	  // Biases are additive, so change sign on calculated average gyro biases
	  data[1] = (-gyro_bias[0]/4)       & 0xFF;
	  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
	  data[3] = (-gyro_bias[1]/4)       & 0xFF;
	  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
	  data[5] = (-gyro_bias[2]/4)       & 0xFF;

	  // Push gyro biases to hardware registers
	  writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
	  writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
	  writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
	  writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
	  writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
	  writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);

	  // Output scaled gyro biases for display in the main program
	  gyroBias[0] = (float) gyro_bias[0]/(float) gyrosensitivity;
	  gyroBias[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
	  gyroBias[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

	  // Construct the accelerometer biases for push to the hardware accelerometer
	  // bias registers. These registers contain factory trim values which must be
	  // added to the calculated accelerometer biases; on boot up these registers
	  // will hold non-zero values. In addition, bit 0 of the lower byte must be
	  // preserved since it is used for temperature compensation calculations.
	  // Accelerometer bias registers expect bias input as 2048 LSB per g, so that
	  // the accelerometer biases calculated above must be divided by 8.

	  // A place to hold the factory accelerometer trim biases
	  int accel_bias_reg[3] = {0, 0, 0};
	  // Read factory accelerometer trim values
	  readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]);
	  accel_bias_reg[0] = (int) (((int)data[0] << 8) | data[1]);
	  readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
	  accel_bias_reg[1] = (int) (((int)data[0] << 8) | data[1]);
	  readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
	  accel_bias_reg[2] = (int) (((int)data[0] << 8) | data[1]);

	  // Define mask for temperature compensation bit 0 of lower byte of
	  // accelerometer bias registers
	  int mask = 1uL;
	  // Define array to hold mask bit for each accelerometer bias axis
	  int mask_bit[3] = {0, 0, 0};

	  for (ii = 0; ii < 3; ii++)
	  {
	    // If temperature compensation bit is set, record that fact in mask_bit
	    if ((accel_bias_reg[ii] & mask))
	    {
	      mask_bit[ii] = 0x01;
	    }
	  }

	  // Construct total accelerometer bias, including calculated average
	  // accelerometer bias from above
	  // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g
	  // (16 g full scale)
	  accel_bias_reg[0] -= (accel_bias[0]/8);
	  accel_bias_reg[1] -= (accel_bias[1]/8);
	  accel_bias_reg[2] -= (accel_bias[2]/8);

	  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	  data[1] = (accel_bias_reg[0])      & 0xFF;
	  // preserve temperature compensation bit when writing back to accelerometer
	  // bias registers
	  data[1] = data[1] | mask_bit[0];
	  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	  data[3] = (accel_bias_reg[1])      & 0xFF;
	  // Preserve temperature compensation bit when writing back to accelerometer
	  // bias registers
	  data[3] = data[3] | mask_bit[1];
	  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	  data[5] = (accel_bias_reg[2])      & 0xFF;
	  // Preserve temperature compensation bit when writing back to accelerometer
	  // bias registers
	  data[5] = data[5] | mask_bit[2];

	  // Apparently this is not working for the acceleration biases in the MPU-9250
	  // Are we handling the temperature correction bit properly?
	  // Push accelerometer biases to hardware registers
	  writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
	  writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
	  writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
	  writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
	  writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
	  writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);

	  // Output scaled accelerometer biases for display in the main program
	  accelBias[0] = (float)accel_bias[0]/(float)accelsensitivity;
	  accelBias[1] = (float)accel_bias[1]/(float)accelsensitivity;
	  accelBias[2] = (float)accel_bias[2]/(float)accelsensitivity;
	}

	// Accelerometer and gyroscope self test; check calibration wrt factory
	// settings
	// Should return percent deviation from factory trim values, +/- 14 or
	// less
	// deviation is a pass.
	void MPU9250SelfTest(float destination)
	{
	  int rawData[6] = {0, 0, 0, 0, 0, 0};
	  int selfTest[6];
	  int gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
	  float factoryTrim[6];
	  int FS = 0;

	  // Set gyro sample rate to 1 kHz
	  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);
	  // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
	  writeByte(MPU9250_ADDRESS, CONFIG, 0x02);
	  // Set full scale range for the gyro to 250 dps
	  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 1<<FS);
	  // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
	  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x02);
	  // Set full scale range for the accelerometer to 2 g
	  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 1<<FS);

	  // Get average current values of gyro and acclerometer
	  for (int ii = 0; ii < 200; ii++)
	  {
	Serial.print("BHW::ii = ");
	Serial.println(ii);
	    // Read the six raw data registers into data array
	    readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);
	    // Turn the MSB and LSB into a signed 16-bit value
	    aAvg[0] += (int)(((int)rawData[0] << 8) | rawData[1]) ;
	    aAvg[1] += (int)(((int)rawData[2] << 8) | rawData[3]) ;
	    aAvg[2] += (int)(((int)rawData[4] << 8) | rawData[5]) ;

	    // Read the six raw data registers sequentially into data array
	    readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);
	    // Turn the MSB and LSB into a signed 16-bit value
	    gAvg[0] += (int)(((int)rawData[0] << 8) | rawData[1]) ;
	    gAvg[1] += (int)(((int)rawData[2] << 8) | rawData[3]) ;
	    gAvg[2] += (int)(((int)rawData[4] << 8) | rawData[5]) ;
	  }

	  // Get average of 200 values and store as average current readings
	  for (int ii =0; ii < 3; ii++)
	  {
	    aAvg[ii] /= 200;
	    gAvg[ii] /= 200;
	  }

	  // Configure the accelerometer for self-test
	  // Enable self test on all three axes and set accelerometer range to +/- 2 g
	  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0xE0);
	  // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
	  writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0xE0);
	  wait(25);  // wait a while to let the device stabilize

	  // Get average self-test values of gyro and acclerometer
	  for (int ii = 0; ii < 200; ii++)
	  {
	    // Read the six raw data registers into data array
	    readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);
	    // Turn the MSB and LSB into a signed 16-bit value
	    aSTAvg[0] += (int)(((int)rawData[0] << 8) | rawData[1]) ;
	    aSTAvg[1] += (int)(((int)rawData[2] << 8) | rawData[3]) ;
	    aSTAvg[2] += (int)(((int)rawData[4] << 8) | rawData[5]) ;

	    // Read the six raw data registers sequentially into data array
	    readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);
	    // Turn the MSB and LSB into a signed 16-bit value
	    gSTAvg[0] += (int)(((int)rawData[0] << 8) | rawData[1]) ;
	    gSTAvg[1] += (int)(((int)rawData[2] << 8) | rawData[3]) ;
	    gSTAvg[2] += (int)(((int)rawData[4] << 8) | rawData[5]) ;
	  }

	  // Get average of 200 values and store as average self-test readings
	  for (int ii =0; ii < 3; ii++)
	  {
	    aSTAvg[ii] /= 200;
	    gSTAvg[ii] /= 200;
	  }

	  // Configure the gyro and accelerometer for normal operation
	  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);
	  writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0x00);
	  wait(25);  // wait a while to let the device stabilize

	  // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
	  // X-axis accel self-test results
	  selfTest[0] = readByte(MPU9250_ADDRESS, SELF_TEST_X_ACCEL);
	  // Y-axis accel self-test results
	  selfTest[1] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_ACCEL);
	  // Z-axis accel self-test results
	  selfTest[2] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_ACCEL);
	  // X-axis gyro self-test results
	  selfTest[3] = readByte(MPU9250_ADDRESS, SELF_TEST_X_GYRO);
	  // Y-axis gyro self-test results
	  selfTest[4] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_GYRO);
	  // Z-axis gyro self-test results
	  selfTest[5] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_GYRO);

	  // Retrieve factory self-test value from self-test code reads
	  // FT[Xa] factory trim calculation
	  factoryTrim[0] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[0] - 1.0) ));
	  // FT[Ya] factory trim calculation
	  factoryTrim[1] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[1] - 1.0) ));
	  // FT[Za] factory trim calculation
	  factoryTrim[2] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[2] - 1.0) ));
	  // FT[Xg] factory trim calculation
	  factoryTrim[3] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[3] - 1.0) ));
	  // FT[Yg] factory trim calculation
	  factoryTrim[4] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[4] - 1.0) ));
	  // FT[Zg] factory trim calculation
	  factoryTrim[5] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[5] - 1.0) ));

	  // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim
	  // of the Self-Test Response
	  // To get percent, must multiply by 100
	  for (int i = 0; i < 3; i++)
	  {
	    // Report percent differences
	    destination[i] = 100.0 * ((float)(aSTAvg[i] - aAvg[i])) / factoryTrim[i]
	      - 100.;
	    // Report percent differences
	    destination[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3]
	      - 100.;
	  }
	}

	// Function which accumulates magnetometer data after device
	// initialization.
	// It calculates the bias and scale in the x, y, and z axes.
	void magCalMPU9250(float bias_dest, float scale_dest)
	{
	  int ii = 0, sample_count = 0;
	  int mag_bias[3]  = {0, 0, 0},
	          mag_scale[3] = {0, 0, 0};
	  int mag_max[3]  = {0x8000, 0x8000, 0x8000},
	          mag_min[3]  = {0x7FFF, 0x7FFF, 0x7FFF},
	          mag_temp[3] = {0, 0, 0};

	  // Make sure resolution has been calculated
	  getMres();

	  Serial.println(F("Mag Calibration: Wave device in a figure 8 until done!"));
	  Serial.println(
	      F("  4 seconds to get ready followed by 15 seconds of sampling)"));
	  wait(4000);

	  // shoot for ~fifteen seconds of mag data
	  // at 8 Hz ODR, new mag data is available every 125 ms
	  if (Mmode == M_8HZ)
	  {
	    sample_count = 128;
	  }
	  // at 100 Hz ODR, new mag data is available every 10 ms
	  if (Mmode == M_100HZ)
	  {
	    sample_count = 1500;
	  }

	  for (ii = 0; ii < sample_count; ii++)
	  {
	    readMagData(mag_temp);  // Read the mag data

	    for (int jj = 0; jj < 3; jj++)
	    {
	      if (mag_temp[jj] > mag_max[jj])
	      {
	        mag_max[jj] = mag_temp[jj];
	      }
	      if (mag_temp[jj] < mag_min[jj])
	      {
	        mag_min[jj] = mag_temp[jj];
	      }
	    }

	    if (Mmode == M_8HZ)
	    {
	      wait(135); // At 8 Hz ODR, new mag data is available every 125 ms
	    }
	    if (Mmode == M_100HZ)
	    {
	      wait(12);  // At 100 Hz ODR, new mag data is available every 10 ms
	    }
	  }

	  // Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
	  // Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
	  // Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

	  // Get hard iron correction
	  // Get 'average' x mag bias in counts
	  mag_bias[0]  = (mag_max[0] + mag_min[0]) / 2;
	  // Get 'average' y mag bias in counts
	  mag_bias[1]  = (mag_max[1] + mag_min[1]) / 2;
	  // Get 'average' z mag bias in counts
	  mag_bias[2]  = (mag_max[2] + mag_min[2]) / 2;

	  // Save mag biases in G for main program
	  bias_dest[0] = (float)mag_bias[0] * mRes * factoryMagCalibration[0];
	  bias_dest[1] = (float)mag_bias[1] * mRes * factoryMagCalibration[1];
	  bias_dest[2] = (float)mag_bias[2] * mRes * factoryMagCalibration[2];

	  // Get soft iron correction estimate
	  // Get average x axis max chord length in counts
	  mag_scale[0]  = (mag_max[0] - mag_min[0]) / 2;
	  // Get average y axis max chord length in counts
	  mag_scale[1]  = (mag_max[1] - mag_min[1]) / 2;
	  // Get average z axis max chord length in counts
	  mag_scale[2]  = (mag_max[2] - mag_min[2]) / 2;

	  float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
	  avg_rad /= 3.0;

	  scale_dest[0] = avg_rad / ((float)mag_scale[0]);
	  scale_dest[1] = avg_rad / ((float)mag_scale[1]);
	  scale_dest[2] = avg_rad / ((float)mag_scale[2]);

	  Serial.println(F("Mag Calibration done!"));
	}

	// Wire.h read and write protocols
	int writeByte(int deviceAddress, int registerAddress, int data) {
		if (_csPin != NOT_SPI) {
			return writeByteSPI(registerAddress, data);
		} else {
			return writeByteWire(deviceAddress, registerAddress, data);
		}
	}

	int writeByteSPI(int registerAddress, int writeData) {
			int returnVal;

			SPI.beginTransaction(SPISettings(SPI_DATA_RATE,MSBFIRST,SPI_MODE));select();

			SPI.transfer(registerAddress);returnVal=SPI.transfer(writeData);

			deselect();SPI.endTransaction();#ifdef SERIAL_DEBUG Serial.print("writeByteSPI slave returned: 0x");Serial.println(returnVal,HEX);#endif return returnVal;
		}

	int writeByteWire(int deviceAddress, int registerAddress, int data) {
		comms.
		Wire.beginTransmission(deviceAddress); // Initialize the Tx buffer
		Wire.write(registerAddress); // Put slave register address in Tx
										// buffer
		Wire.write(data); // Put data in Tx buffer
		Wire.endTransmission(); // Send the Tx buffer
		// TODO: Fix this to return something meaningful
		return NULL;
	}

	// Read a byte from given register on device. Calls necessary SPI or I2C
	// implementation. This was configured in the constructor.
	int readByte(int deviceAddress, int registerAddress) {
		if (_csPin != NOT_SPI) {
			return readByteSPI(registerAddress);
		} else {
			return readByteWire(deviceAddress, registerAddress);
		}
	}

	// Read a byte from the given register address from device using I2C
	int readByteWire(int deviceAddress, int registerAddress) {
		int data; // `data` will store the register data

		// Initialize the Tx buffer
		Wire.beginTransmission(deviceAddress);
		// Put slave register address in Tx buffer
		Wire.write(registerAddress);
		// Send the Tx buffer, but send a restart to keep connection alive
		Wire.endTransmission(false);
		// Read one byte from slave register address
		Wire.requestFrom(deviceAddress, (int) 1);
		// Fill Rx buffer with result
		data = Wire.read();
		// Return data read from slave register
		return data;
	}

	// Read a byte from the given register address using SPI
	int readByteSPI(int registerAddress) {
		return writeByteSPI(registerAddress | READ_FLAG,
				0xFF /* 0xFF is arbitrary */);
	}

	// Read 1 or more bytes from given register and device using I2C
	int readBytesWire(int deviceAddress, int registerAddress, int count, int dest) {
		// Initialize the Tx buffer
		Wire.beginTransmission(deviceAddress);
		// Put slave register address in Tx buffer
		Wire.write(registerAddress);
		// Send the Tx buffer, but send a restart to keep connection alive
		Wire.endTransmission(false);

		int i = 0;
		// Read bytes from slave register address
		Wire.requestFrom(deviceAddress, count);
		while (Wire.available()) {
			// Put read results in the Rx buffer
			dest[i++] = Wire.read();
		}

		return i; // Return number of bytes written
	}

	// Select slave IC by asserting CS pin
	void select() {
		digitalWrite(_csPin, LOW);
	}

	// Select slave IC by deasserting CS pin
	void deselect() {
		digitalWrite(_csPin, HIGH);
	}

	int readBytesSPI(int registerAddress, int count,int dest) {
			SPI.beginTransaction(SPISettings(SPI_DATA_RATE,MSBFIRST,SPI_MODE));select();

			SPI.transfer(registerAddress|READ_FLAG);

			int i;

			for(i=0;i<count;i++){dest[i]=SPI.transfer(0x00);#ifdef SERIAL_DEBUG Serial.print("readBytesSPI::Read byte: 0x");Serial.println(dest[i],HEX);#endif}

			SPI.endTransaction();deselect();

			waitMicroseconds(50);

			return i; // Return number of bytes written

			/*
			 * #ifdef SERIAL_DEBUG
			 * Serial.print("writeByteSPI slave returned: 0x");
			 * Serial.println(returnVal, HEX); #endif return returnVal;
			 */

			/*
			 * // Set slave address of AK8963 and set AK8963 for read
			 * writeByteSPI(I2C_SLV0_ADDR, AK8963_ADDRESS | READ_FLAG);
			 * Serial.print("\nBHW::I2C_SLV0_ADDR set to: 0x");
			 * Serial.println(readByte(MPU9250_ADDRESS, I2C_SLV0_ADDR), HEX); //
			 * Set address to start read from writeByteSPI(I2C_SLV0_REG,
			 * registerAddress); // Read bytes from magnetometer //
			 * Serial.print("\nBHW::I2C_SLV0_CTRL gets 0x");
			 * Serial.println(READ_FLAG | count, HEX); // Read count bytes from
			 * registerAddress via I2C_SLV0
			 * Serial.print("BHW::readBytesSPI: return value test: ");
			 * Serial.println(writeByteSPI(I2C_SLV0_CTRL, READ_FLAG | count));
			 */
		}

	int readBytes(int deviceAddress, int registerAddress, int count, int dest) {
		if (_csPin == NOT_SPI) // Read via I2C
		{
			return readBytesWire(deviceAddress, registerAddress, count, dest);
		} else // Read
				// using
				// SPI
		{
			return readBytesSPI(registerAddress, count, dest);
		}
	}

	bool magInit() {
			// Reset registers to defaults, bit auto clears
			writeByteSPI(0x6B,0x80);
			// Auto select the best available clock source
			writeByteSPI(0x6B,0x01);
			// Enable X,Y, & Z axes of accel and gyro
			writeByteSPI(0x6C,0x00);
			// Config disable FSYNC pin, set gyro/temp bandwidth to 184/188 Hz
			writeByteSPI(0x1A,0x01);
			// Self tests off, gyro set to +/-2000 dps FS
			writeByteSPI(0x1B,0x18);
			// Self test off, accel set to +/- 8g FS
			writeByteSPI(0x1C,0x08);
			// Bypass DLPF and set accel bandwidth to 184 Hz
			writeByteSPI(0x1D,0x09);
			// Configure INT pin (active high / push-pull / latch until read)
			writeByteSPI(0x37,0x30);
			// Enable I2C master mode
			// TODO Why not do this 11-100 ms after power up?
			writeByteSPI(0x6A,0x20);
			// Disable multi-master and set I2C master clock to 400 kHz
			// https://developer.mbed.org/users/kylongmu/code/MPU9250_SPI/ calls
			// says
			// enabled multi-master... TODO Find out why
			writeByteSPI(0x24,0x0D);
			// Set to write to slave address 0x0C
			writeByteSPI(0x25,0x0C);
			// Point save 0 register at AK8963's control 2 (soft reset) register
			writeByteSPI(0x26,0x0B);
			// Send 0x01 to AK8963 via slave 0 to trigger a soft restart
			writeByteSPI(0x63,0x01);
			// Enable simple 1-byte I2C reads from slave 0
			writeByteSPI(0x27,0x81);
			// Point save 0 register at AK8963's control 1 (mode) register
			writeByteSPI(0x26,0x0A);
			// 16-bit continuous measurement mode 1
			writeByteSPI(0x63,0x12);
			// Enable simple 1-byte I2C reads from slave 0
			writeByteSPI(0x27,0x81);

			// TODO: Remove this code
			int ret=ak8963WhoAmI_SPI();#ifdef SERIAL_DEBUG Serial.print("magInit to return ");Serial.println((ret==0x48)?"true":"false");#endif return ret==0x48;
		}

	// Write a null byte w/o CS assertion to get SPI hardware to idle high
	// (mode 3)
	void kickHardware() {
		SPI.beginTransaction(SPISettings(SPI_DATA_RATE, MSBFIRST, SPI_MODE));
		SPI.transfer(0x00); // Send null byte
		SPI.endTransaction();
	}

	bool begin() {
		kickHardware();
		return magInit();
	}

	// Read the WHOAMI (WIA) register of the AK8963
	// TODO: This method has side effects
	int ak8963WhoAmI_SPI()
	{
	  int response, oldSlaveAddress, oldSlaveRegister, oldSlaveConfig;
	  // Save state
	  oldSlaveAddress  = readByteSPI(I2C_SLV0_ADDR);
	  oldSlaveRegister = readByteSPI(I2C_SLV0_REG);
	  oldSlaveConfig   = readByteSPI(I2C_SLV0_CTRL);
	#ifdef SERIAL_DEBUG
	  Serial.print("Old slave address: 0x");
	  Serial.println(oldSlaveAddress, HEX);
	  Serial.print("Old slave register: 0x");
	  Serial.println(oldSlaveRegister, HEX);
	  Serial.print("Old slave config: 0x");
	  Serial.println(oldSlaveConfig, HEX);
	#endif

	  // Set the I2C slave addres of AK8963 and set for read
	  response = writeByteSPI(I2C_SLV0_ADDR, AK8963_ADDRESS|READ_FLAG);
	  // I2C slave 0 register address from where to begin data transfer
	  response = writeByteSPI(I2C_SLV0_REG, 0x00);
	  // Enable 1-byte reads on slave 0
	  response = writeByteSPI(I2C_SLV0_CTRL, 0x81);
	  waitMicroseconds(1);
	  // Read WIA register
	  response = writeByteSPI(WHO_AM_I_AK8963|READ_FLAG, 0x00);

	  // Restore state
	  writeByteSPI(I2C_SLV0_ADDR, oldSlaveAddress);
	  writeByteSPI(I2C_SLV0_REG, oldSlaveRegister);
	  writeByteSPI(I2C_SLV0_CTRL, oldSlaveConfig);

	  return response;
	/*private static final double kSamplePeriod = 0.001;
	private static final double kCalibrationSampleTime = 5.0;
	private static final double kDegreePerSecondPerLSB = 0.0125;
	 
	private static final byte REG_GYRO_CONFIG = 0x1B;
	private static final byte REG_GYRO_ZOUT_H = 0x47;
	private static final byte REG_GYRO_ZOUT_L = 0x48;

	
	
	I2C comms;
	byte[] buffer;
	int config;
	int rate;
	
	
	/**
	 * 
	 *
	public MPU9250Gyro(I2C.Port port, Range range, int deviceAddress) {

	  
		comms = new I2C(port, deviceAddress);
	    setRange(range);

		comms.read(REG_GYRO_ZOUT_H, 2, buffer);
		rate = (buffer[0] * 256) + buffer[1];
		
		comms.write(REG_GYRO_CONFIG, 0b00010000);
	}

	@Override
	  public void setRange(Range range) {
	    final byte value;

	    switch (range) {
	      case k2G:
	        value = 0;
	        break;
	      case k4G:
	        value = 1;
	        break;
	      case k8G:
	        value = 2;
	        break;
	      case k16G:
	        value = 3;
	        break;
	      default:
	        throw new IllegalArgumentException(range + " unsupported range type");
	    }
	@Override
	public void calibrate() {
		// TODO Auto-generated method stub
		
	}


	@Override
	public double getAngle() {
		
		return 0;
	}


	@Override
	public double getRate() {
		
		return data;
	}*/

	
	public void reset() {
		// TODO Auto-generated method stub

	}

}
