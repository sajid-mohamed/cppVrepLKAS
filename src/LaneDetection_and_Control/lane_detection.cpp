#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>
//#include <chrono> 
#include "polyfit.hpp"
#include "config.hpp"

using namespace cv;
using namespace std;
//using namespace std::chrono;

// constructor
laneDetection::laneDetection() {}
// destructor
laneDetection::~laneDetection() {}

/// <summary>
/// "main" function for processing of the input image 
/// goes through the image processing pipeline steps
/// performs the control computations
/// </summary>
/// <param name="src">Image to be processed</param>  
/// <returns>0 when succesful</returns>  
long double laneDetection::lane_detection_pipeline(Mat src_in){
    setUseOptimized(true);
	
	//cout << " lane detection pipeline\n";	
    // scale src
    Mat src;
    copyMakeBorder(src_in, src, 256, 0, 0, 0, BORDER_CONSTANT, Scalar(0)); //Why top=256?? [SD] // Top zeros (black portion at the top)

    // Processing time ##########################################################################
    //auto t_proc_start = high_resolution_clock::now(); 
    // get_warped_image
    Mat img_warped;
    bev_transform(src, img_warped);

    // rgb2gray
    Mat img_gray;
    cvtColor(img_warped, img_gray, COLOR_RGB2GRAY); // Gray-scale conversion [SD]

    // get_white_mask
    Mat mask, img_white_mask;
    #ifdef VREP_CAM
    inRange(img_gray, 200, 255, mask); // All pixels within range set to 255, others set to 0 [SD]
    #endif
    #ifdef CARND_CAM
    inRange(img_gray, 150, 255, mask);
    #endif
    bitwise_and(img_gray, img_gray, img_white_mask, mask); // extracts pixels above threshold=200; other pisels set to 0 [SD]

    // early check on img_white_mask
    Mat img_lanes, img_thresholding;
    double minVal, maxVal;
    minMaxLoc(img_white_mask, &minVal, &maxVal);
	
	// if pipe != 5
    if (maxVal){ // if maxVal > 0 [SD]
        img_lanes = img_white_mask;
    } else {
        // get_threshed_img
        threshold(img_gray, img_thresholding, 0, 255, THRESH_BINARY | THRESH_OTSU);
        img_lanes = img_thresholding;
    }
	
	// if pipe == 5
	// threshold(img_gray, img_thresholding, 0, 255, THRESH_BINARY | THRESH_OTSU);
    // img_lanes = img_thresholding;
    
    // make a temp copy of img_lanes for reverting back to original image 
	#ifdef RE_DRAW_IMAGE				 
    Mat img_lanes_temp = img_lanes.clone();
	#endif
    // perform sliding window lane tracking
    vector<vector<Point>> lanes = sliding_window_lane_tracking(img_lanes); 									  

    #ifdef RE_DRAW_IMAGE
    // lane identification -> reverting back to original image
    Mat img_detected_lanes, draw_lines, diff_src_rebev, img_rev_warped, img_roi;
    lane_identification(lanes, src, img_roi, img_warped, img_lanes_temp, 
                        img_detected_lanes, draw_lines, diff_src_rebev, img_rev_warped);
    #endif

    // calculate lateral deviation
	vector<long double> yL;
	yL = calculate_lateral_deviation(lanes[0], lanes[1]);

    #ifdef RE_DRAW_IMAGE
	string out_string = "/home/sajid/Desktop/cppvreplkas/out_imgs";
    Rect out_roi = Rect(0, 256, 512, 256); // cv::Rect rect(x, y, width, height); 
    Mat cropped = Mat(img_detected_lanes, out_roi);
    imwrite(out_string+"/img_roi.png", img_roi);
    imwrite(out_string+"/img_diff_src_rebev.png", diff_src_rebev);
    imwrite(out_string+"/img_output.png", cropped); 
    imwrite(out_string+"/img_draw_lines.png", draw_lines); 
    imwrite(out_string+"/img_rev_warped.png", img_rev_warped); 
    #endif
    imwrite(out_string+"/img_warped.png", img_warped);
    imwrite(out_string+"/img_gray.png", img_gray);
    if (maxVal){
        imwrite(out_string+"/img_white_mask.png", img_white_mask);
    } else {
        imwrite(out_string+"/img_thresholding.png", img_thresholding);
    }
    imwrite(out_string+"/img_lanes.png", img_lanes_temp);
    return yL[0];
}

/// <summary> .... </summary>
/// <param name="bar"> .... </param>
/// <returns> .... </returns>
vector<vector<Point2f>> laneDetection::get_bev_points(){
    // declare variables
    vector<vector<Point2f>> vertices(2);
    vector<Point2f> src_vertices(4), dst_vertices(4);
    
	//cout << " get bev points\n";	
    #ifdef VREP_CAM
    int warp_offset = 70;
    // assign src points
    src_vertices[1] = Point(233, 280);
    src_vertices[0] = Point(277, 280);
    src_vertices[2] = Point(462, 512);
    src_vertices[3] = Point(50, 512);
    // assign dst points
    dst_vertices[1] = Point(50 + warp_offset, 0);
    dst_vertices[0] = Point(462 - warp_offset, 0);
    dst_vertices[2] = Point(462 - warp_offset, 512);
    dst_vertices[3] = Point(50 + warp_offset, 512);
    #endif

    #ifdef CARND_CAM
    int warp_offset = 70;
    int height = 720, width = 1280;
    // assign src points
    src_vertices[1] = Point(707, 464);
    src_vertices[0] = Point(575, 464);
    src_vertices[2] = Point(258, 682);
    src_vertices[3] = Point(1049, 682);
    // assign dst points
    dst_vertices[1] = Point(width - 350 + warp_offset, 0);
    dst_vertices[0] = Point(250 - warp_offset, 0);
    dst_vertices[2] = Point(250 - warp_offset, height);
    dst_vertices[3] = Point(width - 350 + warp_offset, height);  
    #endif

    // return result 
    vertices[0] = src_vertices;
    vertices[1] = dst_vertices;
    return vertices;   
}

/// <summary> .... </summary>
/// <param name="bar"> .... </param>
/// <returns> .... </returns>
void laneDetection::bev_transform(Mat& src, Mat& dst){

	//cout << " Bev Transform\n";	
    // get transform points 
    vector<vector<Point2f>> vertices = get_bev_points(); // 2D float points [SD] 
    // calculate transform matrix
    Mat M = getPerspectiveTransform(vertices[0], vertices[1]);
    // perform bev 
    warpPerspective(src, dst, M, dst.size(), INTER_LINEAR, BORDER_CONSTANT);
}

/// <summary> .... </summary>
/// <param name="bar"> .... </param>
/// <returns> .... </returns>
void laneDetection::bev_rev_transform(Mat& src, Mat& dst){

	//cout << " Bev Rev Transform\n";	
    // get transform points 
    vector<vector<Point2f>> vertices = get_bev_points();
    // calculate rev transform matrix
    Mat M = getPerspectiveTransform(vertices[1], vertices[0]);
    // perform bev
    warpPerspective(dst, src, M, dst.size(), INTER_LINEAR, BORDER_CONSTANT);
}

/// <summary> .... </summary>
/// <param name="bar"> .... </param>
/// <returns> .... </returns>
vector<vector<Point>> laneDetection::sliding_window_lane_tracking(Mat& src){

	//cout << " Sliding window lane tracking\n";	
    // sliding window polyfit 
    Size shape = src.size();
    // get bottom half of BEV image
    Mat img_bottom_half = src(Range(shape.height/2, shape.height), Range(0, shape.width));
    // get histogram of bottom half
    Mat histogram;
    reduce(img_bottom_half, histogram, 0, REDUCE_SUM, CV_32S);
    // midpoint of histogram
    int midpoint = shape.width/2;
    // get left and right halves of histogram
    Point min_loc, max_loc;
    double minVal, maxVal; 
    Size histo = histogram.size();
    // left half of histogram
    Mat hist_half = histogram(Range(0, histo.height), Range(0, histo.width/2));
    minMaxLoc(hist_half, &minVal, &maxVal, &min_loc, &max_loc);
    int leftx_base = max_loc.x;
    // right half of histogram
    hist_half = histogram(Range(0, histo.height), Range(histo.width/2, histo.width));
    minMaxLoc(hist_half, &minVal, &maxVal, &min_loc, &max_loc);
    int rightx_base = max_loc.x + midpoint;

    // Get x and y positions of all nonzero pixels in the image
    Mat nonzero;
    findNonZero(src, nonzero);
    
    // Current positions to be updated for each window
    int leftx_current = leftx_base;
    int rightx_current = rightx_base;
    // width of the windows
    int margin   = 65;
    // minimum number of pixels needed to recenter window
    unsigned int minpix   = 400;
    // number of sliding windows
    int nwindows = 8;
    // Set height of sliding windows
    int window_height = shape.height/nwindows;   
    // declare rectangle parameters
    int win_y_low, win_y_high, win_xleft_low, win_xleft_high, win_xright_low, win_xright_high;
    // declare variables for mean calculation
    int total_good_left, counter_good_left, total_good_right, counter_good_right;
    // Create vectors for points of left box and right box
    vector<Point> point_1_left(nwindows), point_1_right(nwindows);
    vector<Point> point_2_left(nwindows), point_2_right(nwindows);
    // Create vectors to receive left and right lane pixel indices per window
    vector<int> good_left_inds;
    vector<int> good_right_inds;
    good_left_inds.reserve(nonzero.total());
    good_right_inds.reserve(nonzero.total());
    // Create empty vectors to accumulate left and right lane pixel Points
    vector<Point> left_lane_inds, right_lane_inds;

    // draw sliding windows and track the two lanes
    for (int window = 0; window < nwindows; ++window){
        // set rectangle parameters
        win_y_low       = shape.height - (window + 1) * window_height;
        win_y_high      = shape.height - window * window_height;
        win_xleft_low   = leftx_current - margin;
        win_xleft_high  = leftx_current + margin;
        win_xright_low  = rightx_current - margin;
        win_xright_high = rightx_current + margin;

        #ifdef DRAW_SLIDING_WINDOWS
        // top left corner
        point_1_left[window]  = Point(win_xleft_low, win_y_high);
        point_1_right[window] = Point(win_xright_low, win_y_high);
        // bottom right corner
        point_2_left[window]  = Point(win_xleft_high, win_y_low);
        point_2_right[window] = Point(win_xright_high, win_y_low);
        // draw rectangles 
        rectangle(src, point_1_left[window], point_2_left[window], Scalar(255, 255, 255));
        rectangle(src, point_1_right[window], point_2_right[window], Scalar(255, 255, 255));
        #endif

        // initialize variables for mean calculation
        total_good_left    = 0;
        counter_good_left  = 0;
        total_good_right   = 0;
        counter_good_right = 0;

        // Identify the nonzero pixels in x and y within the 2 rectangles
        for (int i = nonzero.total(); i > 0; --i) {
            // left rectangle
            if ((nonzero.at<Point>(i).y >= win_y_low) && (nonzero.at<Point>(i).y < win_y_high) && 
                (nonzero.at<Point>(i).x >= win_xleft_low) && (nonzero.at<Point>(i).x < win_xleft_high)){
                    // // Append candidate point.x to the current window vector
                    // good_left_inds[i] = nonzero.at<Point>(i).x;
                    good_left_inds.push_back(nonzero.at<Point>(i).x);
                    // mean calculation
                    total_good_left += nonzero.at<Point>(i).x;
                    counter_good_left++;
                    // Append candidate point to the accumulation vector
                    left_lane_inds.push_back(nonzero.at<Point>(i));
                }
            // right rectangle
            if ((nonzero.at<Point>(i).y >= win_y_low) && (nonzero.at<Point>(i).y < win_y_high) && 
                (nonzero.at<Point>(i).x >= win_xright_low) && (nonzero.at<Point>(i).x < win_xright_high)){
                    // // Append candidate point.x to the current window vector
                    // good_right_inds[i] = nonzero.at<Point>(i).x;
                    good_right_inds.push_back(nonzero.at<Point>(i).x);
                    // mean calculation
                    total_good_right += nonzero.at<Point>(i).x;
                    counter_good_right++;
                    // Append candidate point to the accumulation vector
                    right_lane_inds.push_back(nonzero.at<Point>(i));
                }
        }
        // If you found > minpix pixels, recenter next window on their mean position
        if ((good_left_inds.size() > minpix) && counter_good_left ){
            leftx_current  = total_good_left / counter_good_left;
        }
        if ((good_right_inds.size() > minpix) && counter_good_right ){
            rightx_current = total_good_right / counter_good_right;
        }
    }   

    // return identified points for both lanes
    vector<vector<Point>> lanes(2);
    lanes[0] = left_lane_inds;
    lanes[1] = right_lane_inds;
    return lanes;
}

/// <summary> .... </summary>
/// <param name="bar"> .... </param>
/// <returns> .... </returns>
vector<long double> laneDetection::calculate_lateral_deviation(vector<Point> left_lane_inds, vector<Point> right_lane_inds){

	//cout << " Calculate lateral deviation\n";	
    //  Extract left and right line pixel positions
    vector<long double> leftx(left_lane_inds.size()), lefty(left_lane_inds.size());
    vector<long double> rightx(right_lane_inds.size()), righty(right_lane_inds.size());
    for (unsigned int i = 0; i < left_lane_inds.size(); ++i){
        leftx[i] = left_lane_inds[i].x;
        lefty[i] = left_lane_inds[i].y;
    }
    for (unsigned int i = 0; i < right_lane_inds.size(); ++i){
        rightx[i] = right_lane_inds[i].x;
        righty[i] = right_lane_inds[i].y;
    }   
    
    // Fit a second order polynomial to left and right lane positions
    vector<long double> left_fit = mathalgo::polyfit( lefty, leftx, 2 );
    vector<long double> right_fit = mathalgo::polyfit( righty, rightx, 2 );
    
    // calculate lateral deviation yL at look-ahead distance LL = 0.55 (5.5 meters): Use y=483.0 [SD]
    vector<long double> vec1{483.0L}, vec2{512.0L}; // changed vec2(512.0L) [SD]
    vector<long double> yL_leftx = mathalgo::polyval(left_fit, vec1); 
    vector<long double> yL_rightx = mathalgo::polyval(right_fit, vec1);  
    vector<long double> scale_leftx = mathalgo::polyval(left_fit, vec2); 
    vector<long double> scale_rightx = mathalgo::polyval(right_fit, vec2); 
    long double scale = 0.325 / (scale_rightx[0] - scale_leftx[0]); // 0.325m (V-REP lane width) and 3.25m (Actual lane-width), scale=10:1
	
	vector<long double>	yL(2);
    yL[0] = (256 - (yL_leftx[0] + yL_rightx[0]) / 2) * scale; // 256 is used as car camera is centered at the middle of the image at lowest point (y=512.0)
	yL[1] = ((yL_leftx[0] + yL_rightx[0]) / 2) * scale;	
    //cout << " Computed lateral deviation: "<< yL[0] <<"\n";
    return yL;
}

/// <summary> .... </summary>
/// <param name="bar"> .... </param>
/// <returns> .... </returns>
void laneDetection::lane_identification(vector<vector<Point>> lanes, Mat& src, Mat& img_roi, 
										Mat& img_warped, Mat& img_lanes_temp, Mat& img_detected_lanes, 
										Mat& draw_lines, Mat& diff_src_rebev, Mat& img_rev_warped) {
    
	//cout << " Lane identification\n";	
    // reverting back to original image
    draw_lines = img_warped.clone();

    // draw circes iso lines;
    for (unsigned int i = 0; i < lanes[0].size(); ++i){
        circle(draw_lines, lanes[0][i], 1, Scalar(0, 0, 255), 1, 8); //Scalar (b, g, r) [SD]
    }
    for (unsigned int i = 0; i < lanes[1].size(); ++i){
        circle(draw_lines, lanes[1][i], 1, Scalar(0, 0, 255), 1, 8);
    }

    // get reverse bev on bev with drawed detected lines 
    bev_rev_transform(img_rev_warped, draw_lines);
    // reverse bev transform on bev of src image 
    bev_rev_transform(img_roi, img_warped);
    // substract: diff_src_rebev = src - img_roi 
    absdiff(src, img_roi, diff_src_rebev);
    // combine: diff_src_rebev + img_rev_warped
    bitwise_or(diff_src_rebev, img_rev_warped, img_detected_lanes);    
}

/// <summary> .... </summary>
/// <param name="bar"> .... </param>
/// <returns> .... </returns>
vector<long double> laneDetection::get_yL_container(){
    return m_yL_container;
}

vector<long double> laneDetection::get_ref_container(){
    return m_ref_container;
}
