/**
 * 
 */
package org.usfirst.frc.team3997.robot.hardware;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.*;
import java.lang.Math;

/**
 * @author peter
 *
 */
public class MPU9250Gyro extends GyroBase {

	// Magnetometer Registers
	int AK8963_ADDRESS = 0x0C;
	int AK8963_WHO_AM_I = 0x00; // should return = 0x48
	int AK8963_INFO = 0x01;
	int AK8963_ST1 = 0x02; // data ready status bit 0
	int AK8963_XOUT_L = 0x03; // data
	int AK8963_XOUT_H = 0x04;
	int AK8963_YOUT_L = 0x05;
	int AK8963_YOUT_H = 0x06;
	int AK8963_ZOUT_L = 0x07;
	int AK8963_ZOUT_H = 0x08;
	int AK8963_ST2 = 0x09; // Data overflow bit 3 and data read error status bit 2
	int AK8963_CNTL = 0x0A; // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM
							// (1111) modes on bits 3:0
	int AK8963_ASTC = 0x0C; // Self test control
	int AK8963_I2CDIS = 0x0F; // I2C disable
	int AK8963_ASAX = 0x10; // Fuse ROM x-axis sensitivity adjustment value
	int AK8963_ASAY = 0x11; // Fuse ROM y-axis sensitivity adjustment value
	int AK8963_ASAZ = 0x12; // Fuse ROM z-axis sensitivity adjustment value

	int SELF_TEST_X_GYRO = 0x00;
	int SELF_TEST_Y_GYRO = 0x01;
	int SELF_TEST_Z_GYRO = 0x02;

	/*
	 * int X_FINE_GAIN = 0x03 // [7:0] fine gain int Y_FINE_GAIN = 0x04 int
	 * Z_FINE_GAIN = 0x05 int XA_OFFSET_H = 0x06 // User-defined trim values for
	 * accelerometer int XA_OFFSET_L_TC = 0x07 int YA_OFFSET_H = 0x08 int
	 * YA_OFFSET_L_TC = 0x09 int ZA_OFFSET_H = 0x0A int ZA_OFFSET_L_TC = 0x0B
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

	int MOT_DUR = 0x20; // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB =
						// 1 ms
	int ZMOT_THR = 0x21; // Zero-motion detection threshold bits [7:0]
	int ZRMOT_DUR = 0x22; // Duration counter threshold for zero motion interrupt generation, 16 Hz rate,
							// LSB = 64 ms

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
	int I2C_MST_DELAY_CTRL = 0x67;
	int SIGNAL_PATH_RESET = 0x68;
	int MOT_DETECT_CTRL = 0x69;
	int USER_CTRL = 0x6A; // Bit 7 enable DMP, bit 3 reset DMP
	int PWR_MGMT_1 = 0x6B; // Device defaults to the SLEEP mode
	int PWR_MGMT_2 = 0x6C;
	int DMP_BANK = 0x6D; // Activates a specific bank in the DMP
	int DMP_RW_PNT = 0x6E; // Set read/write pointer to a specific start address in specified DMP bank
	int DMP_REG = 0x6F; // Register in DMP from which to read or to which to write
	int DMP_REG_1 = 0x70;
	int DMP_REG_2 = 0x71;
	int FIFO_COUNTH = 0x72;
	int FIFO_COUNTL = 0x73;
	int FIFO_R_W = 0x74;
	int WHO_AM_I_MPU9250 = 0x75; // Should return = 0x71
	int XA_OFFSET_H = 0x77;
	int XA_OFFSET_L = 0x78;
	int YA_OFFSET_H = 0x7A;
	int YA_OFFSET_L = 0x7B;
	int ZA_OFFSET_H = 0x7D;
	int ZA_OFFSET_L = 0x7E;
	int MPU9250_ADDRESS = 0x00;
	int MS5637_ADDRESS = 0x00;
	double gRes = 0;
	// Using the MPU9250Teensy 3.1 Add-On shield, ADO is set to 0
	// Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1
	int ADO = 0;
	int[] gyroCount = new int[3];
	int[] gyroData = new int[3];
	double[] gyroBias = new double[3];
	boolean SystemDebug = true; // set to true to get System.out output for debugging

	int ADC_256 = 0x00; // define pressure and temperature conversion rates
	int ADC_512 = 0x02;
	int ADC_1024 = 0x04;
	int ADC_2048 = 0x06;
	int ADC_4096 = 0x08;
	int ADC_8192 = 0x0A;
	int ADC_D1 = 0x40;
	int ADC_D2 = 0x50;

	double GyroMeasError = Math.PI * (4.0f / 180.0f); // gyroscope measurement error in rads/s (start at 40 deg/s)
	double GyroMeasDrift = Math.PI * (0.0f / 180.0f); // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
	// There is a tradeoff in the beta parameter between accuracy and response
	// speed.
	// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError
	// of 2.7 degrees/s) was found to give optimal accuracy.
	// However, with this value, the LSM9SD0 response time is about 10 seconds to a
	// stable initial quaternion.
	// Subsequent changes also require a longish lag time to a stable output, not
	// fast enough for a quadcopter or robot car!
	// By increasing beta (GyroMeasError) by about a factor of fifteen, the response
	// time constant is reduced to ~2 sec
	// I haven't noticed any reduction in solution accuracy. This is essentially the
	// I coefficient in a PID control sense;
	// the bigger the feedback coefficient, the faster the solution converges,
	// usually at the expense of accuracy.
	// In any case, this is the free parameter in the Madgwick filtering and fusion
	// scheme.
	double beta = Math.sqrt(3.0f / 4.0f) * GyroMeasError; // compute beta
	double zeta = Math.sqrt(3.0f / 4.0f) * GyroMeasDrift; // compute zeta, the other free parameter in the Madgwick
															// scheme usually set to a small or zero value
	double yaw, pitch, roll;
	int delt_t = 0, count = 0, sumCount = 0; // used to control display output rate
	double a12, a22, a31, a32, a33; // rotation matrix coefficients for Euler angles and gravity components
	double deltat = 0.0;
	double sum = 0.0; // integration interval for both filter schemes
	int lastUpdate = 0;
	int firstUpdate = 0; // used to calculate integration interval
	int Now = 0; // used to calculate integration interval
	double Kp = 5 * 2;
	double Ki = 0;
	double gx, gy, gz; // variables to hold latest sensor data values
	double q[] = { 1.0f, 0.0f, 0.0f, 0.0f }; // vector to hold quaternion
	double eInt[] = { 0.0f, 0.0f, 0.0f }; // vector to hold integral error for Mahony method
	// Set initial input parameters
	double currentTime = 0;
	double lastTime = 0;
	final int GFS_250DPS = 0;
	final int GFS_500DPS = 0x02;
	final int GFS_1000DPS = 0x04;
	final int GFS_2000DPS = 0x06;

	I2C i2c;
	int gScale;

	public MPU9250Gyro(Port port) {
		yaw = 0;
		gz = 0;
		gScale = GFS_250DPS;
		if (ADO == 1) {
			MPU9250_ADDRESS = 0x69; // Device address when ADO = 1
			AK8963_ADDRESS = 0x0C; // Address of magnetometer
			MS5637_ADDRESS = 0x76; // Address of altimeter
		} else {
			MPU9250_ADDRESS = 0x68; // Device address when ADO = 0
			AK8963_ADDRESS = 0x0C; // Address of magnetometer
			MS5637_ADDRESS = 0x76; // Address of altimeter
		}
		i2c = new I2C(port, MPU9250_ADDRESS);
		setName("MPU9250Gyro", port.value);
		if (i2c.addressOnly()) {
			getGyroRes(gScale);
			calibrate();
			init();
			SmartDashboard.putString("GYRO", "FOUND");

		} else {
			SmartDashboard.putString("GYRO", "NOTFOUND");
		}

	}

	byte[] readBytes(int registerAddress, int count, byte[] destination) {
		i2c.read(registerAddress, count, destination);
		return destination;

	}

	byte readByte(int registerAddress) {
		byte[] destination = new byte[1];
		i2c.read(registerAddress, 1, destination);
		return destination[0];

	}

	boolean writeByte(int registerAddress, int data) {
		return i2c.write(registerAddress, data);
	}

	void getGyroRes(int gScale) {
		// Possible gyro scales (and their register bit settings) are:
		// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS (11).
		// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit
		// value:
		switch (gScale) {
		case GFS_250DPS:
			gRes = 250.0 / 32768.0;
			break;
		case GFS_500DPS:
			gRes = 500.0 / 32768.0;
			break;
		case GFS_1000DPS:
			gRes = 1000.0 / 32768.0;
			break;
		case GFS_2000DPS:
			gRes = 2000.0 / 32768.0;
			break;
		}
	}

	void readGyroData(int[] destination) {
		SmartDashboard.putBoolean("Raw Boolean", true);

		byte[] rawData = new byte[6]; //// x/y/z gyro register data stored here
		readBytes(GYRO_XOUT_H, 6, rawData); // Read the six raw data registers sequentially into data array
		double[] dashboardData = new double[rawData.length];
		for (int i = 0; i < rawData.length; i++) {
			dashboardData[i] = (double) rawData[i];
		}
		//System.out.println("This is raw data: " + rawData);
		//SmartDashboard.putNumberArray("Raw Data", dashboardData);
		destination[0] = ((rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
		destination[1] = ((rawData[2] << 8) | rawData[3]);
		destination[2] = ((rawData[4] << 8) | rawData[5]);

	}

	void init() {
		// wake up device
		writeByte(PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
		Timer.delay(.100); // Wait for all registers to reset

		// get stable time source
		writeByte(PWR_MGMT_1, 0x01); // Auto select clock source to be PLL gyroscope reference if ready else
		Timer.delay(.200);

		// Configure Gyro and Thermometer
		// Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz,
		// respectively;
		// minimum delay time for this setting is 5.9 ms, which means sensor fusion
		// update rates cannot
		// be higher than 1 / 0.0059 = 170 Hz
		// DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
		// With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8
		// kHz, or 1 kHz
		writeByte(CONFIG, 0x03);

		// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
		writeByte(SMPLRT_DIV, 0x04); // Use a 200 Hz rate; a rate consistent with the filter update rate
										// determined inset in CONFIG above

		// Set gyroscope full scale range
		// Range selects FS_SEL and GFS_SEL are 0 - 3, so 2-bit values are left-shifted
		// into positions 4:3
		int c = readByte(GYRO_CONFIG); // get current GYRO_CONFIG register value
		// c = c & ~0xE0; // Clear self-test bits [7:5]
		c = (c & ~0x03); // Clear Fchoice bits [1:0]
		c = (c & ~0x18); // Clear GFS bits [4:3]
		c = c | gScale << 3; // Set full scale range for the gyro
		// c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits
		// 1:0 of GYRO_CONFIG
		writeByte(GYRO_CONFIG, c); // Write new GYRO_CONFIG value to register

		// The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
		// but all these rates are further reduced by a factor of 5 to 200 Hz because of
		// the SMPLRT_DIV setting

		// Configure Interrupts and Bypass Enable
		// Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until
		// interrupt cleared,
		// clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
		// can join the I2C bus and all can be controlled by the Arduino as master
		// writeByte(INT_PIN_CFG, 0x22);
		writeByte(INT_PIN_CFG, 0x12); // INT is 50 microsecond pulse and any read to clear
		writeByte(INT_ENABLE, 0x01); // Enable data ready (bit 0) interrupt
		Timer.delay(.100);

	}

	@Override
	public void calibrate() {

		calibrateMPU9250(gyroBias);
	}

	void calibrateMPU9250(double[] dest) {
		byte data[] = new byte[12]; // data array to hold accelerometer and gyro x, y, z, data
		int ii, packet_count, fifo_count;
		int gyro_bias[] = { 0, 0, 0 };

		// reset device, reset all registers, clear gyro and accelerometer bias
		// registers
		writeByte(PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
		Timer.delay(0.01);

		// get stable time source
		// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
		writeByte(PWR_MGMT_1, 0x01);
		writeByte(PWR_MGMT_2, 0x00);
		Timer.delay(0.02);

		// Configure device for bias calculation
		writeByte(INT_ENABLE, 0x00); // Disable all interrupts
		writeByte(FIFO_EN, 0x00); // Disable FIFO
		writeByte(PWR_MGMT_1, 0x00); // Turn on internal clock source
		writeByte(I2C_MST_CTRL, 0x00); // Disable I2C master
		writeByte(USER_CTRL, 0x00); // Disable FIFO and I2C master modes
		writeByte(USER_CTRL, 0x0C); // Reset FIFO and DMP
		Timer.delay(0.0015);

		// Configure MPU9250 gyro and accelerometer for bias calculation
		writeByte(CONFIG, 0x01); // Set low-pass filter to 188 Hz
		writeByte(SMPLRT_DIV, 0x00); // Set sample rate to 1 kHz
		writeByte(GYRO_CONFIG, 0x00); // Set gyro full-scale to 250 degrees per second, maximum sensitivity

		int gyrosensitivity = 131; // = 131 LSB/degrees/sec

		// Configure FIFO to capture accelerometer and gyro data for bias calculation
		writeByte(USER_CTRL, 0x40); // Enable FIFO
		writeByte(FIFO_EN, 0x78); // Enable gyro and accelerometer sensors for FIFO (max size 512 bytes in
									// MPU-9250)
		Timer.delay(0.04); // accumulate 40 samples in 80 milliseconds = 480 bytes

		// At end of sample accumulation, turn off FIFO sensor read
		writeByte(FIFO_EN, 0x00); // Disable gyro and accelerometer sensors for FIFO
		readBytes(FIFO_COUNTH, 2, data); // read FIFO sample count
		fifo_count = (data[0] << 8) | data[1];
		packet_count = fifo_count / 12;// How many sets of full gyro and accelerometer data for averaging

		for (ii = 0; ii < packet_count; ii++) {
			int gyro_temp[] = { 0, 0, 0 };
			readBytes(FIFO_R_W, 12, data); // read data for averaging

			gyro_temp[0] = ((data[6] << 8) | data[7]);
			gyro_temp[1] = ((data[8] << 8) | data[9]);
			gyro_temp[2] = ((data[10] << 8) | data[11]);

			// Sum individual signed 16-bit biases to get accumulated signed 32-bit biases

			gyro_bias[0] += gyro_temp[0];
			gyro_bias[1] += gyro_temp[1];
			gyro_bias[2] += gyro_temp[2];

		}
		// Normalize sums to get average count biases

		gyro_bias[0] /= packet_count;
		gyro_bias[1] /= packet_count;
		gyro_bias[2] /= packet_count;

		// Construct the gyro biases for push to the hardware gyro bias registers, which
		// are reset to zero upon device startup
		data[0] = (byte) ((-gyro_bias[0] / 4 >> 8) & 0xFF); // Divide by 4 to get 32.9 LSB per deg/s to conform to
															// expected bias input format
		data[1] = (byte) ((-gyro_bias[0] / 4) & 0xFF); // Biases are additive, so change sign on calculated average gyro
														// biases
		data[2] = (byte) ((-gyro_bias[1] / 4 >> 8) & 0xFF);
		data[3] = (byte) ((-gyro_bias[1] / 4) & 0xFF);
		data[4] = (byte) ((-gyro_bias[2] / 4 >> 8) & 0xFF);
		data[5] = (byte) ((-gyro_bias[2] / 4) & 0xFF);

		/// Push gyro biases to hardware registers
		/*
		 * writeByte(XG_OFFSET_H, data[0]); writeByte(XG_OFFSET_L, data[1]);
		 * writeByte(YG_OFFSET_H, data[2]); writeByte(YG_OFFSET_L, data[3]);
		 * writeByte(ZG_OFFSET_H, data[4]); writeByte(ZG_OFFSET_L, data[5]);
		 */
		dest[0] = gyro_bias[0] / gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
		dest[1] = gyro_bias[1] / gyrosensitivity;
		dest[2] = gyro_bias[2] / gyrosensitivity;

	}

	@Override
	public double getAngle() {
		update();
		return yaw;
	}

	@Override
	public double getRate() {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public void reset() {
		
		writeByte(PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
		yaw = 0;
		Timer.delay(0.1);
	}

	public void update() {
		// Multiply timer.getTimestamp * 1000 to get millis
		double currentTimestamp = Timer.getFPGATimestamp() * 1000;
		
		// TODO change to account for drift
		double rotation_threshold = 1;
		SmartDashboard.putBoolean("RUN UPDATE", true);
		readGyroData(gyroData);

		gx = gyroData[0] * gRes;
		gy = gyroData[1] * gRes;
		gz = gyroData[2] * gRes;
		
		double deltaTime = currentTimestamp - currentTime;
		//I have a bad feeling about having it exactly equal to 20
		if (((gz >= rotation_threshold) || (gz <= -rotation_threshold)) && (deltaTime >= 20)) {
			/**
			 * using this website {@link https://playground.arduino.cc/Main/Gyro}
			 */
			gz /= (1000 / deltaTime);
			yaw += gz;
			if (yaw < 0)
				yaw += 360;
			else if (yaw > 359)
				yaw -= 360;
			currentTime = Timer.getFPGATimestamp() * 1000;
		} 
		
		

	}

	public void free() {
		super.free();
		i2c.free();
	}

}
