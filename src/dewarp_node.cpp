#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc/imgproc.hpp>

cv::Mat K, D;
double u, v;
double new_u, new_v;
cv::Mat map1, map2;

bool initialized;

ros::Publisher img_pub;

cv::Point2d dist(const cv::Point2d & in, const cv::Mat & D, const cv::Mat & K){
	double k1 = D.at<double>(0);
	double k2 = D.at<double>(1);
	double k3 = D.at<double>(4);
	double p1 = D.at<double>(2);
	double p2 = D.at<double>(3);
	
	double fx = K.at<double>(0);
	double fy = K.at<double>(4);
	double cx = K.at<double>(2);
	double cy = K.at<double>(5);
	
	cv::Point2d out;
	
	double x = (in.x - cx) / fx;
	double y = (in.y - cy) / fy;
	
	
	std::cout << K << "\n";
	printf("%f \n", K.at<double>(2));
	
	double r2 = pow(x, 2.0) + pow(y, 2.0);
	double r4 = r2 * r2;
	double r6 = r2 * r2 * r2;
		
	double X_radial = x * k1 * r2 + x * k2 * r4 + x * k3 * r6;
	double Y_radial = y * k1 * r2 + y * k2 * r4 + y * k3 * r6;
		
	double X_tangential = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
	double Y_tangential = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
		
	out.x = fx * (x + X_radial + X_tangential) + cx;
	out.y = fy * (y + Y_radial + Y_tangential) + cy;
	
	return out;
}

void cam_info_CB(const sensor_msgs::CameraInfo::ConstPtr & i){

		ROS_INFO("GOT INFO\n");
	if(
		u != i->width			||
		v != i->height			||
		
		D.at<double>(0) != i->D[0]	||
		D.at<double>(1) != i->D[1]	||
		D.at<double>(2) != i->D[2]	||
		D.at<double>(3) != i->D[3]	||
		D.at<double>(4) != i->D[4]	||
		
		K.at<double>(0) != i->K[0]	||
		K.at<double>(2) != i->K[2]	||
		K.at<double>(4) != i->K[4]	||
		K.at<double>(5) != i->K[5]
	){
	
		ROS_INFO("INIT BEGINNING\n");
		
		u = i->width;
		v = i->height;
		K = (cv::Mat1d(3, 3) << i->K[0], 0, i->K[2], 0, i->K[4], i->K[5], 0, 0, 1);
		D = (cv::Mat1d(1, 5) << i->D[0], i->D[1], i->D[3], i->D[4], i->D[2]);
	
		double u_centralized = u / 2.0;
		double v_centralized = v / 2.0;
		cv::Point2d tl = dist(cv::Point2d(-u_centralized,  v_centralized), D, K);
		cv::Point2d tr = dist(cv::Point2d( u_centralized,  v_centralized), D, K);
		cv::Point2d bl = dist(cv::Point2d(-u_centralized, -v_centralized), D, K);
		cv::Point2d br = dist(cv::Point2d( u_centralized, -v_centralized), D, K);
		
		double new_u = std::max(tr.x - tl.x, br.x - bl.x);
		double new_v = std::max(tl.y - bl.y, tr.y - br.y);
		
		printf("NEW DIMENSIONS %f %f\n", new_u, new_v);
		
		cv::initUndistortRectifyMap(K, D, cv::Mat(), K,  cv::Size(new_u, new_v), CV_32FC1,  map1,  map2);
	
		initialized = true;
	}
}

void image_CB(const sensor_msgs::Image::ConstPtr & i){
	if(initialized){
		cv::Mat im_original;
		cv::Mat im_rectified;
		cv_bridge::CvImagePtr cv_ptr;
		try {
			cv_ptr = cv_bridge::toCvCopy(i);
			im_original = cv_ptr->image;
		} catch (cv_bridge::Exception &e) {
			ROS_ERROR("Could not convert from encoding to 'bgr8'.");
			return;
		}
		
		cv::remap(im_original, im_rectified, map1, map2, cv::INTER_LINEAR);
		
		cv_ptr->image = im_rectified;
		sensor_msgs::Image i_out = *(cv_ptr->toImageMsg());
		img_pub.publish(i_out);
	}
}

int main(int argc, char * * argv){
	initialized = false;
	ros::init(argc, argv, "dewarp_node");
	ros::NodeHandle nh;
	
	img_pub = nh.advertise<sensor_msgs::Image>("/camera/image_rect_color", false);
	ros::Subscriber s1 = nh.subscribe("/camera/camera_info", 1, cam_info_CB);
	ros::Subscriber s2 = nh.subscribe("/camera/image_raw_color", 1, image_CB);
	
	ROS_INFO("GOING INTO SPIN");
	
	ros::spin();
	
	return 0;
}
