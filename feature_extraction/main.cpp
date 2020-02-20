#include <iostream>
 #include <opencv4/opencv2/opencv.hpp>

 using namespace std;
 using namespace cv;
 
int main(int argc, char **argv) {
    if(argc != 3)
		{
			cout<<"Please input image1 and image2!"<<endl;
			return 1;
		}
		
		Mat img_1 = imread(argv[1] , 1);
		Mat img_2 = imread(argv[2] , 1);
		
		vector<KeyPoint> keypoints_1,keypoints_2;
		Mat descriptors_1,descriptors_2;
		Ptr<ORB> orb = ORB::create();
		//Step1 : Detect the FAST keyPoint
		orb -> detect(img_1,keypoints_1);
		orb -> detect(img_2,keypoints_2);
		//Step2 : Compute the BRIEF descriptors_1
		orb -> compute(img_1,keypoints_1,descriptors_1);
		orb -> compute(img_2,keypoints_2,descriptors_2);
		
		Mat outImg_1;
		drawKeypoints(img_1,keypoints_1,outImg_1,Scalar::all(-1),DrawMatchesFlags::DEFAULT);
		imshow("ORB Feature Point",outImg_1);
		
		//Step3 : Extracte feature, using Hamming distance
		vector<DMatch> matches;
		BFMatcher matcher(NORM_HAMMING);
		matcher.match(descriptors_1,descriptors_2,matches);
		
		//Step4 : Filter the matched point
		double min_dist = 10000,max_dist = 0;
		
		for(int i = 0;i < descriptors_1.rows;i ++)
		{
			double dist = matches[i].distance;
			if(dist < min_dist)
				min_dist = dist;
			if(dist > max_dist)
				max_dist = dist;
		}
		
		printf("-- Max dist : %f \n",max_dist);
		printf("-- Min dist : %f \n",min_dist);
		
		vector<DMatch> good_matches;
		for(int i = 0;i < descriptors_1.rows;i ++)
		{
			if(matches[i].distance <= max(2 * min_dist,30.0))
				good_matches.push_back(matches[i]);
		}
		
		//Step5 : Output
		Mat img_match;
		Mat img_goodmatch;
		drawMatches(img_1,keypoints_1,img_2,keypoints_2,matches,img_match);
		drawMatches(img_1,keypoints_1,img_2,keypoints_2,good_matches,img_goodmatch);
		imshow("All Feature Extraction",img_match);
		imshow("Optimized Feature Extraction",img_goodmatch);
		waitKey(0);
		
    return 0;
}
