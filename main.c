/* main.c */

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <sys/time.h>

#include "ekfNavINS.h"

static int running = 0;
double max=32767;

// SFE_UBLOX_GNSS gps;

// uint64_t micros()
// {
//     uint64_t us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::
//                   now().time_since_epoch()).count();
//     return us; 
// }

// interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__ ((unused)) int dummy)
{
	running=0;
	return;
}


double random_double_val(void){
    double random_value;

    // srand ( time ( NULL));
    random_value = (double)rand()/RAND_MAX*20.0-10.0;//float in range -1 to 1
    
    return random_value;
}

// // file pointer
// fstream fout;
// inline void open_file(std::string file_name) {
//     fout.open(file_name, ios::out | ios::app);
//     fout << std::fixed <<"time" << "," << "latitude" << "," << "longitude" << "," 
//     								   << "roll" << "," << "pitch" << "," << "yaw" << "," 
//     								   << "filtered_latitude" << "," << "filtered_longitude" << ","
//     								   << "filtered_roll" << "," << "filtered_pitch" << "," << "filtered_yaw" << std::endl;
// }

// inline void update_file(long int time, double latitude, double longitude,
// 						float roll, float pitch, float yaw, 
// 						double filtered_latitude, double filtered_longitude,
// 						float filtered_roll, float filtered_pitch, float filtered_yaw) {
// 	fout << time << "," << std::setprecision(7) << latitude << "," << std::setprecision(7) << longitude 
// 				 << "," << roll << "," << pitch << "," << yaw
// 				 << "," << std::setprecision(7) << filtered_latitude << "," << std::setprecision(7) << filtered_longitude 
// 				 << "," << filtered_roll << "," << filtered_pitch << "," << filtered_yaw << std::endl;
// }

// inline void close_file() {
// 	fout.close();
// }

int main(int argc, char *argv[])
{    
	struct timeval start, end;

    
	struct r_p_y_pos value;
	// constexpr int FLOAT_MIN = -10;
	// constexpr int FLOAT_MAX = 10;
	printf("111111-----------------------------------------------------------------\n");
	// MPU9560
	// rc_mpu_data_t data; //struct to hold new data
	// Ublox GPS
	// Stream serialComm("/dev/ttyACM0");
    // gps.begin(serialComm);
    // EKF
    // ekfNavINS ekf;

    float ax, ay, az, hx, hy, hz, pitch, roll, yaw;

	// parse arguments
	// opterr = 0;

	// set signal handler so the loop can exit cleanly
	// signal(SIGINT, __signal_handler);
	running = 1;

	// use defaults for now, except also enable magnetometer.
	// rc_mpu_config_t conf = rc_mpu_default_config();
	// conf.i2c_bus = I2C_BUS;
	// conf.enable_magnetometer = 1; // Enable magnetometer
	// conf.show_warnings = 1; // Enable warnings

	// if(rc_mpu_initialize(&data, conf)){
	// 	fprintf(stderr,"rc_mpu_initialize_failed\n");
	// 	return -1;
	// }
	
	// std::random_device rd;
  //   std::default_random_engine eng(rd());
  //   std::uniform_real_distribution<> distr(FLOAT_MIN, FLOAT_MAX);

	// open_file(std::string("./gnss.csv"));
	double random_value;

    srand ( time ( NULL));

	//now just wait, print_data will run
	printf("22222-----------------------------------------------------------------\n");
	while (running) {
		printf("33333-----------------------------------------------------------------\n");
		// read sensor data
		// if(rc_mpu_read_accel(&data)<0){
		// 	printf("read accel data failed\n");
		// }
		// if(rc_mpu_read_gyro(&data)<0){
		// 	printf("read gyro data failed\n");
		// }
		// if(rc_mpu_read_mag(&data)){
		// 	printf("read mag data failed\n");
		// }

		// for (int n = 0; n < 5; ++n) {
    //     	cout << setprecision(10)
    //         	 << distr(eng) << "\n";
    // 	}
		// printf("plot time = %f", distr(eng));
		// printf("plot time2 = %f", distr(eng));

		// update the filter
		if (1) {
			
			ax = random_double_val();
			ay = random_double_val();
			az = random_double_val();
			hx = random_double_val(); 
			hy = random_double_val(); 
			hz = random_double_val();

			
			// auto start = std::chrono::high_resolution_clock::now();

			// std::tie(pitch,roll,yaw) = getPitchRollYaw(ax, ay, az, hx, hy, hz);

			gettimeofday(&start, NULL);

      		value = getPitchRollYaw(ax, ay, az, hx, hy, hz);


			ekf_update(time(NULL) /*,gps.getTimeOfWeek()*/, random_double_val(), random_double_val(), random_double_val(),
				random_double_val(), random_double_val(), random_double_val(),
				random_double_val(), random_double_val(), random_double_val(),
				ax, ay, az, hx, hy, hz);

			gettimeofday(&end, NULL);

			double time_taken = end.tv_sec + end.tv_usec / 1e6 -
								start.tv_sec - start.tv_usec / 1e6; // in seconds

			printf("time program took %f seconds to execute\n", time_taken);

			// auto elapsed = std::chrono::high_resolution_clock::now();

			// auto duration = std::chrono::duration_cast<std::chrono::microseconds>(elapsed - start);

			// cout << "Time taken by function: " << duration.count() << " microseconds" << endl;

			// update_file(time(NULL), 
			// 			gps.getLatitude()*1e-7, gps.getLongitude()*1e-7,
			// 			roll, pitch, yaw,
			// 			ekf.getLatitude_rad()*RAD_TO_DEG, ekf.getLongitude_rad()*RAD_TO_DEG,
			// 			ekf.getRoll_rad(), ekf.getPitch_rad(), ekf.getHeading_rad());

			// printf("------------------------- %ld -------------------------- \n", gps.getTimeOfWeek());
			// printf("Latitude  : %2.7f %2.7f\n", gps.getLatitude()*1e-7, ekf.getLatitude_rad()*RAD_TO_DEG);
			// printf("Longitute : %2.7f %2.7f\n", gps.getLongitude()*1e-7, ekf.getLongitude_rad()*RAD_TO_DEG);
			// printf("Altitude  : %2.3f %2.3f\n", gps.getAltitude()*1e-3, ekf.getAltitude_m());
			// printf("Speed (N) : %2.3f %2.3f\n", gps.getNedNorthVel()*1e-3, ekf.getVelNorth_ms());
			// printf("Speed (E) : %2.3f %2.3f\n", gps.getNedEastVel()*1e-3, ekf.getVelEast_ms());
			// printf("Speed (D) : %2.3f %2.3f\n", gps.getNedDownVel()*1e-3, ekf.getVelDown_ms());
			printf("Roll 	  : %2.3f %2.3f\n", value.roll, getRoll_rad());
			printf("Pitch     : %2.3f %2.3f\n", value.pitch, getPitch_rad());
			printf("Yaw       : %2.3f %2.3f\n", value.yaw, getHeading_rad());
			/*printf("Gyro X    : %f  %f\n", data.gyro[0]*DEG_TO_RAD, ekf.getGyroBiasX_rads());
			printf("Gyro Y    : %f  %f\n", data.gyro[1]*DEG_TO_RAD, ekf.getGyroBiasY_rads());
			printf("Gyro Z    : %f  %f\n", data.gyro[2]*DEG_TO_RAD, ekf.getGyroBiasZ_rads());
			printf("Accel X   : %f  %f\n", data.accel[0], ekf.getAccelBiasX_mss());
			printf("Accel Y   : %f  %f\n", data.accel[1], ekf.getAccelBiasY_mss());
			printf("Accel Z   : %f  %f\n", data.accel[2], ekf.getAccelBiasZ_mss());*/
			printf("-----------------------------------------------------------------\n");
		}
		// sleep(0.1);
	}
	printf("\n");
	// close_file();
	// rc_mpu_power_off();
	return 0;
}