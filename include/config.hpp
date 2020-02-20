#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

extern "C" {
    #include "extApi.h"
}

// ------------ defs -------------//
#define SELECT_PERIOD 1          // controller period: 0 -> 0.035, 1 -> 0.065, 2 -> 0.1 
#define DRAW_SLIDING_WINDOWS 1   // select whether to draw the sliding window while tracking
#define VREP_CAM 1               // select when VREP camera frames are used
//#define CARND_CAM 1            // select when CARND camera frames are used
#define RE_DRAW_IMAGE 1          // run re-draw image function

// ------------ class declarations -------------//
class vrepAPI {
private:
    // private membcvers
    // std::vector<std::vector<cv::Mat>> m_three_channels = std::vector<std::vector<cv::Mat>> (400);
    // std::vector<cv::Mat> m_out_Mat = std::vector<cv::Mat> (400, cv::Mat(256,512,CV_8UC3));
    int m_clientID, m_ping_time, m_cam, m_car, m_floor;
    int m_resolution[2];
    simxUChar* m_image;
    int m_nakedCar_steeringLeft, m_nakedCar_steeringRight;
    int m_nakedCar_motorLeft, m_nakedCar_motorRight;
    simxFloat m_position[3];
    double m_vx, m_desired_wheel_rot_speed;  
    // private methods
    cv::Mat vrep_img_2_Mat();
public:
    // constructor
    vrepAPI();
    // destructor
    ~vrepAPI();
    // public methods
    void sim_delay(int time_step);
    cv::Mat sim_sense();
    void sim_actuate(std::vector<long double> the_steering_angles);
};

class laneDetection {
private:
    std::vector<long double> m_yL_container;
	std::vector<long double> m_ref_container;
    // private methods
    void bev_transform(cv::Mat& src, cv::Mat& dst);
    void bev_rev_transform(cv::Mat& src, cv::Mat& dst);
    std::vector<std::vector<cv::Point2f>> get_bev_points();
    std::vector<std::vector<cv::Point>> sliding_window_lane_tracking(cv::Mat& src);
    std::vector<long double> calculate_lateral_deviation(std::vector<cv::Point> left_lane_inds, 
                                            std::vector<cv::Point> right_lane_inds);
    void lane_identification(std::vector<std::vector<cv::Point>> lanes, cv::Mat& src, cv::Mat& img_roi, 
                            cv::Mat& img_warped, cv::Mat& img_lanes_temp, cv::Mat& img_detected_lanes, 
                            cv::Mat& draw_lines, cv::Mat& diff_src_rebev, cv::Mat& img_rev_warped);
public:
    // constructor
    laneDetection();
    // destructor
    ~laneDetection();
    // class public methods
    long double lane_detection_pipeline(cv::Mat src);
    std::vector<long double> get_yL_container();
	std::vector<long double> get_ref_container();
};


class lateralController {
private:
	std::vector<long double> m_steering_angle_left_container;
	std::vector<long double> m_steering_angle_right_container;
    // member variables
    long double m_d, m_l, m_vx, m_LL, m_l_r;
    long double m_z1, m_z2, m_z3, m_z4, m_z5;
    std::vector<long double> m_input = std::vector<long double> (400);
    long double m_desired_steering_angle, m_steering_angle_left, m_steering_angle_right;
    std::vector< Eigen::Matrix<long double, 6, 6> > m_phi_aug    
                                    = std::vector< Eigen::Matrix<long double, 6, 6> > (9);
    std::vector< Eigen::Matrix<long double, 6, 6> > m_T         
                                    = std::vector< Eigen::Matrix<long double, 6, 6> > (9);  
    std::vector< Eigen::Matrix<long double, 1, 5> > m_K2c        
                                    = std::vector< Eigen::Matrix<long double, 1, 5> > (9);
    std::vector< Eigen::Matrix<long double, 6, 1> > m_Gamma_aug  
                                    = std::vector< Eigen::Matrix<long double, 6, 1> > (9);  
public:
    // constructor
    lateralController();
    // destructor
    ~lateralController();
    // class public methods
    void compute_steering_angles(long double the_yL, int the_it_counter, int the_pipe_version);
    std::vector<long double> get_steering_angles();
    void estimate_next_state(int the_it_counter, int the_pipe_version);
	std::vector<long double> get_steering_angle_left_container();
	std::vector<long double> get_steering_angle_right_container();
};

class imageSignalProcessing {
private:    
	void OpenCV_remosaic (cv::Mat *InMat );
public:
    // constructor
    imageSignalProcessing();
    // destructor
    ~imageSignalProcessing();
};


// ------------ function declarations -------------//
template<class Container>
std::ostream& write_containers(	const Container& c1,
								const Container& c2,
								const Container& c3,
								std::ostream& out,
								char delimiter = ',')
{
	out << "lateral_deviation(m)";
	out << delimiter;
    bool write_sep = false;
    for (const auto& e: c1) {
        if (write_sep)
            out << delimiter;
        else
            write_sep = true;
        out << e;
    }
	
	out << "\n";
	out << "steering_angle(left)";
	out << delimiter;
    write_sep = false;
    for (const auto& e: c2) {
        if (write_sep)
            out << delimiter;
        else
            write_sep = true;
        out << e;
    }
	
	out << "\n";
	out << "steering_angle(right)";
	out << delimiter;
    write_sep = false;
    for (const auto& e: c3) {
        if (write_sep)
            out << delimiter;
        else
            write_sep = true;
        out << e;
    }
	
    return out;
}

template<class Container>
std::ostream& write_containers_wref(	const Container& c1,
								const Container& c4,
								const Container& c2,
								const Container& c3,
								std::ostream& out,
								char delimiter = ',')
{
	out << "lateral_deviation(m)";
	out << delimiter;
    bool write_sep = false;
    for (const auto& e: c1) {
        if (write_sep)
            out << delimiter;
        else
            write_sep = true;
        out << e;
    }
	
	out << "\n";
	out << "lane_reference(m)";
	out << delimiter;
    write_sep = false;
    for (const auto& e: c4) {
        if (write_sep)
            out << delimiter;
        else
            write_sep = true;
        out << e;
    }
	
	out << "\n";
	out << "steering_angle(left)";
	out << delimiter;
    write_sep = false;
    for (const auto& e: c2) {
        if (write_sep)
            out << delimiter;
        else
            write_sep = true;
        out << e;
    }
	
	out << "\n";
	out << "steering_angle(right)";
	out << delimiter;
    write_sep = false;
    for (const auto& e: c3) {
        if (write_sep)
            out << delimiter;
        else
            write_sep = true;
        out << e;
    }
	
    return out;
}

// void write_yL_2_file(std::vector<long double> yL_container, int pipeline_version);
std::string get_timestamp();



