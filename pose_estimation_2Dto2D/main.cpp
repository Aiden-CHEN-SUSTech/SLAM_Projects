#include <iostream>
#include <opencv4/opencv2/opencv.hpp>

using namespace std;
using namespace cv;

void find_feature_matches(Mat& img_1, Mat& img_2, 
													vector<KeyPoint>& keypoints_1, 
													vector<KeyPoint>& keypoints_2, 
													vector<DMatch>& matches)
{
	Ptr<ORB> orb = ORB::create();;
	Mat descriptors_1, descriptors_2;
	
	//Step1 : Detect the FAST keyPoint
	orb -> detect(img_1,keypoints_1);
	orb -> detect(img_2,keypoints_2);
	//Step2 : Compute the BRIEF descriptors_1
	orb -> compute(img_1,keypoints_1,descriptors_1);
	orb -> compute(img_2,keypoints_2,descriptors_2);

	//Step3 : Extracte feature, using Hamming distance
	vector<DMatch> match;
	BFMatcher matcher(NORM_HAMMING);
	matcher.match(descriptors_1,descriptors_2,match);
		
	//Step4 : Filter the matched point
	double min_dist = 10000,max_dist = 0;
		
	for(int i = 0;i < descriptors_1.rows;i ++)
	{
		double dist = match[i].distance;
		if(dist < min_dist)
			min_dist = dist;
		if(dist > max_dist)
			max_dist = dist;
	}
		
	for(int i = 0;i < descriptors_1.rows;i ++)
	{
		if(match[i].distance <= max(2 * min_dist,30.0))
			matches.push_back(match[i]);
	}
}

void pose_estimation_2d2d(
	vector<KeyPoint> keypoints_1,
	vector<KeyPoint> keypoints_2,
	vector<DMatch> matches,
	Mat& R,Mat& t)
{
	Mat K = (Mat_<double> (3,3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
	
	vector<Point2f> points1;
	vector<Point2f> points2;
	
	for(int i = 0; i < (int) matches.size(); i++)
	{
		points1.push_back(keypoints_1[matches[i].queryIdx].pt);
		points2.push_back(keypoints_2[matches[i].queryIdx].pt);
	}

	Mat fundamental_matrix;
	fundamental_matrix = findFundamentalMat(points1,points2, FM_RANSAC);
	cout<<"Fundamental Matrix is "<<endl<<fundamental_matrix<<endl;
	
	Point2d principle_point(325.1, 249.7);//Optical Center of the camera
	int focal_length = 521;//Focal Length of camera
	Mat essential_matrix;
	essential_matrix = findEssentialMat(points1,points2,focal_length,principle_point);
	cout<<"Essential Matrix is: "<<endl<<essential_matrix<<endl;
	
	Mat homography_matrix;
	homography_matrix = findHomography(points1,points2,RANSAC,3,noArray(),2000,0.9);
	cout<<"Homography Matrix is : "<<endl<<homography_matrix<<endl;
	
	recoverPose(essential_matrix,points1,points2,R,t,focal_length,principle_point);
	cout<<"R is: "<<endl<<R<<endl;
	cout<<"t is : "<<endl<<t<<endl;
}

int main(int argc, char **argv) {
// 	if(argc != 3)
// 	{
// 		cout<<"Please input image1 and image2!"<<endl;
// 		return 1;
// 	}
	
	Mat img_1 = imread("1.png",1);
	Mat img_2 = imread("2.png",1);
	Mat img_output;
	vector<KeyPoint> keypoints_1,keypoints_2;
	vector<DMatch> matches;
	find_feature_matches(img_1,img_2,keypoints_1,keypoints_2,matches);
	cout<<"Total has "<<matches.size()<<" matched points!"<<endl;
	cout<<"Hello"<<endl;
	cout<<"Hello"<<endl;
	cout<<"Hello"<<endl;
	Mat R,t;
	pose_estimation_2d2d(keypoints_1,keypoints_2,matches,R,t);
	
  return 0;
}
