/*
 *  Copyright (c) 2020 sajidmohamed
 *  Eindhoven University of Technology
 *  Eindhoven, The Netherlands
 *
 *  Name            :   cpp_vrep_framework.cpp
 *
 *  Authors         :   Sajid Mohamed (s.mohamed@tue.nl)
 *			Sayandip De (sayandip.de@tue.nl)
 *
 *  Date            :   March 25, 2020
 *
 *  Function        :   main file to run the imacs framework
 *
 *  History         :
 *      25-03-20    :   Initial version.
 *
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <iostream>
//#include <sstream>
#include <opencv2/opencv.hpp>
#include <cmath> 
#include "config.hpp"

int main(int argc, char **argv) {
	using namespace std;
	using namespace cv;
	float simstep = 0.005; // V-rep simulation step time	
	int simulation_time = 15;	
	if (argc < 2) {
		cout << "Minimum one argument needed.\n Usage: ./cpp_vrep_main {int pipeline_version} {float simstep}\n";
		cout << "Eg: ./cpp_vrep_main 1\t (default): ./cpp_vrep_main 1 0.005\n";
		return -1;
	}
	if (argc == 3) {
		simstep = stof(argv[2]);
		cout << "simstep: "<< simstep << endl;
	}
	float wait_time = 3*simstep;
	// ------- simulation parameters ----------//	
	//string analysis = argv[2];
	//string env = argv[3];
	//string result_dir = "/home/sayandipde/Approx_IBC/final_app/results/";
	//string ret_ref = argv[4];
	//string Q_value = argv[5];
	//bool ret_reference = (ret_ref == "True");
	// cout << ret_reference << endl;
	
	// ------- single image wo control -------- //
	// vector<float> period_s {0.040, 0.025, 0.035, 0.040, 0.020, 0.035, 0.025, 0.020};
	// vector<float> tau_s {0.038, 0.0205, 0.0329, 0.0379, 0.0193, 0.0328, 0.0205, 0.0191};
	
	// ------- multiple image wo control -------- //
	// vector<float> period_s {0.040, 0.025, 0.030, 0.040, 0.020, 0.030, 0.020, 0.020};
	// vector<float> tau_s {0.0374, 0.0201, 0.0298, 0.0368, 0.0159, 0.0294, 0.0193, 0.0155};
	
	// ------- multiple image with control -------- //
	vector<float> period_s {0.040, 0.025, 0.035, 0.040, 0.020, 0.030, 0.020, 0.020, 0.040};
	vector<float> tau_s {0.0379, 0.0206, 0.0303, 0.0373, 0.0164, 0.0299, 0.0198, 0.0160, 0.0359};
	
	int pipeline_version = atoi(argv[1]);
	cout << "pipeline_version: v" << pipeline_version << endl;

	// simulation main loop parameters
	Mat img_vrep, img_isp;
	vector<long double> steering_angles(2);
	float time = 0.0f;
	float curr_period = 0.0f;
	long double yL = 0.0L; // lateral deviation
	int it_counter = 0;
	int time_step;
	// ------- init simulation ----------//

	vrepAPI VrepAPI;
    laneDetection Lane_detection;
    lateralController Controller;
    //IBCController Controller;

	// --- delay 2.5 sec to reach desired velocity ---//		
	time_step = wait_time / simstep;
	cout << "\t\t\tinitialising... waiting for " << time_step << " simulation steps\n\t\t\t";
	//time_step = period_s[pipeline_version] / simstep;
	VrepAPI.sim_delay(time_step);
	curr_period = period_s[pipeline_version];
	cout << "\n\t\t\tCompleted initialisation" << endl;
	// ------- simulation main loop -------//
	cout << "control period :" << curr_period << " sec" << endl;
	cout << "control delay :" << tau_s[pipeline_version] << " sec" << endl;
	cout << "simulation step :" << simstep << " sec" << endl;
	
	while(time < simulation_time - curr_period) {
		cout << "\nsimulation_time: " << time << "\t";
		// ------- sensing ----------//	
		img_vrep = VrepAPI.sim_sense();
		
		// ------- laneDetection -----------------//
		yL = Lane_detection.lane_detection_pipeline(img_vrep);
		//cout << "lateral deviation: " << yL << endl;
		cout << "lateral deviation: " << yL;
		
		// ------- control_compute --------------//
		Controller.compute_steering_angles(yL, it_counter, pipeline_version);
		steering_angles = Controller.get_steering_angles();

		// ------- control_next_state  -----------//
		Controller.estimate_next_state(it_counter, pipeline_version);
			
		// ------- sensor-to-actuator delay ----------//
		time_step = ceil(tau_s[pipeline_version] / simstep);
			
		VrepAPI.sim_delay(time_step);  
		time += time_step*simstep;
		// ----- actuating -----//
		// [TODO] add actuation delay here (too small, neglected)
		VrepAPI.sim_actuate(steering_angles); 

		// ------- remaining delay to sampling period ----------//
		time_step = floor((period_s[pipeline_version] - tau_s[pipeline_version])/ simstep);
		VrepAPI.sim_delay(time_step);  
		time += time_step*simstep;

		// ------- handle simulation time -------//
		//time += period_s[pipeline_version];
		
		it_counter++;
	}
	
	cout << "images: " << it_counter << endl; 

	return 0;
}
