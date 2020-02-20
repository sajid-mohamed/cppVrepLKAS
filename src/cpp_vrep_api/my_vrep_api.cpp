#include <iostream>
#include <cassert>  
#include <opencv2/opencv.hpp>
extern "C" {
    #include "extApi.h"
}
#include "config.hpp"

using namespace std;
using namespace cv;

// private methods
//--- convert image_2_Mat ---//
Mat vrepAPI::vrep_img_2_Mat(){
	// 3 color image (RGB)
	const int b = 3; 
	const int width  = m_resolution[0];
	const int height = m_resolution[1]/2;
	Mat out_Mat(height, width, CV_8UC3);
 	vector<Mat> three_channels;
 	split(out_Mat, three_channels);	
	// Convert to BGR memory storage
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			// Blue channel
			three_channels[0].at<char>(height-y-1,x) = m_image[b * (y * width + x) + 2];
			// Green channel
			three_channels[1].at<char>(height-y-1,x) = m_image[b * (y * width + x) + 1];
			// Red channel
			three_channels[2].at<char>(height-y-1,x) = m_image[b * (y * width + x) + 0];
		}
	}		
	merge(three_channels, out_Mat);
	return out_Mat;

	// split(m_out_Mat[the_iter], m_three_channels[the_iter]);	
	// // Convert to BGR memory storage
	// for (int y = 0; y < height; y++) {
	// 	for (int x = 0; x < width; x++) {
	// 		// Blue channel
	// 		m_three_channels[the_iter][0].at<char>(height-y-1,x) = m_image[b * (y * width + x) + 2];
	// 		// Green channel
	// 		m_three_channels[the_iter][1].at<char>(height-y-1,x) = m_image[b * (y * width + x) + 1];
	// 		// Red channel
	// 		m_three_channels[the_iter][2].at<char>(height-y-1,x) = m_image[b * (y * width + x) + 0];
	// 	}
	// }		
	// merge(m_three_channels[the_iter], m_out_Mat[the_iter]);
	// return m_out_Mat[the_iter];
}

// constructor
vrepAPI::vrepAPI() {
	// ------------ SIM_INIT start ---------------// 
	// [CLIENT SIDE] connect to vrep, use synchronous mode and start simulation
    simxFinish(-1);  // end all running commnication threads [SD]
    m_clientID = simxStart((simxChar*)"127.0.0.1", 19997, true, true, 5000, 5);
    //assert(( "V-REP must be started", m_clientID != -1)); //[SD]
    assert( m_clientID != -1 && "V-REP must be started: ./vrep.sh ../../vrep-scenes/EnterCurveTest.ttt");
    std::string timestamp = get_timestamp();
    cout << timestamp << " Connected to remote API server\n";
    simxSynchronous(m_clientID, true);							 							
    simxStartSimulation(m_clientID, simx_opmode_blocking);		

    // initial configuration
    // Handle V-REP objects, set navigation m_camera
    simxGetObjectHandle(m_clientID, "cam", &m_cam, simx_opmode_blocking);
    simxGetVisionSensorImage(m_clientID, m_cam, m_resolution, &m_image, 0, simx_opmode_streaming);
    // handle vehicle steering joint
    simxGetObjectHandle(m_clientID, "nakedCar_steeringLeft", &m_nakedCar_steeringLeft, simx_opmode_blocking);
    simxGetObjectHandle(m_clientID, "nakedCar_steeringRight", &m_nakedCar_steeringRight, simx_opmode_blocking);
    // handle vehicle longitudinal velocity
    simxGetObjectHandle(m_clientID, "nakedCar_motorLeft", &m_nakedCar_motorLeft, simx_opmode_blocking);
    simxGetObjectHandle(m_clientID, "nakedCar_motorRight", &m_nakedCar_motorRight, simx_opmode_blocking);
    // handle vehicle position
    simxGetObjectHandle(m_clientID, "modAckermannSteeringCar", &m_car, simx_opmode_blocking);
    simxGetObjectHandle(m_clientID, "ResizableFloor_5_25", &m_floor, simx_opmode_blocking);
    simxGetObjectPosition(m_clientID, m_car, m_floor, m_position, simx_opmode_streaming);
    // set initial steering angle to be zero
    simxSetJointTargetPosition(m_clientID, m_nakedCar_steeringLeft,  0, simx_opmode_blocking);
    simxSetJointTargetPosition(m_clientID, m_nakedCar_steeringRight, 0, simx_opmode_blocking);
    simxSetJointPosition(m_clientID, m_nakedCar_steeringLeft,  0, simx_opmode_blocking);
    simxSetJointPosition(m_clientID, m_nakedCar_steeringRight, 0, simx_opmode_blocking);
    // set velocity 
    m_vx  = 2.2; 		 							// vehicle speed = 2.2m/s to simulate ~80km/h in reality
	m_desired_wheel_rot_speed = 2 * m_vx / 0.12681; // desired joint speed in V-REP
    simxSetJointTargetVelocity(m_clientID, m_nakedCar_motorLeft,  m_desired_wheel_rot_speed, simx_opmode_blocking);
    simxSetJointTargetVelocity(m_clientID, m_nakedCar_motorRight, m_desired_wheel_rot_speed, simx_opmode_blocking);
}
// destructor
vrepAPI::~vrepAPI() {
	// ------- finalize ----------//	
	std::string timestamp = get_timestamp();
	cout << "\n" << timestamp << " Success! Simulation finished!\n";
	simxStopSimulation(m_clientID, simx_opmode_blocking);
	simxFinish(-1);		
}

// public methods
void vrepAPI::sim_delay(int time_step) {

	cout << " sim delay: "<< time_step << "\t";	
	// ------------ DELAY  ---------------//
	for (int i = 0; i < time_step; i++){
		simxSynchronousTrigger(m_clientID);
		simxGetPingTime(m_clientID, &m_ping_time);
	}
	//cout << "LastCmdTime: " << simxGetLastCmdTime(m_clientID) << "/9500 ms\n"; 
}

Mat vrepAPI::sim_sense() {

	cout << " sim sense\t";	
	// ------------ SENSE  ---------------// 
	simxGetObjectPosition(m_clientID, m_car, m_floor, m_position, simx_opmode_buffer);
	simxGetVisionSensorImage(m_clientID, m_cam, m_resolution, &m_image, 0, simx_opmode_buffer);
	Mat img_vrep(m_resolution[0], m_resolution[1]/2, CV_8UC3);
	img_vrep = vrep_img_2_Mat();
	return img_vrep;
}

void vrepAPI::sim_actuate(vector<long double> the_steering_angles) {

	cout << " sim actuate\t";	
	// --------- ACTUATE  ------------//
	long double steering_angle_left  = the_steering_angles[0];
	long double steering_angle_right = the_steering_angles[1];
	simxSetJointTargetPosition(m_clientID, m_nakedCar_steeringLeft, steering_angle_left, simx_opmode_blocking);
    simxSetJointTargetPosition(m_clientID, m_nakedCar_steeringRight, steering_angle_right, simx_opmode_blocking);
    simxSetJointPosition(m_clientID, m_nakedCar_steeringLeft,  steering_angle_left, simx_opmode_blocking);
    simxSetJointPosition(m_clientID, m_nakedCar_steeringRight, steering_angle_right, simx_opmode_blocking);
}
