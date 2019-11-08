/* //! include copyright

 */


#include <iostream>
#include "opencv2/opencv_modules.hpp"
#include <stdio.h>
#include <string>
#include <fstream>
#include <algorithm>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

# include "opencv2/core/core.hpp"
# include "opencv2/features2d/features2d.hpp"
# include "opencv2/highgui/highgui.hpp"
# include "opencv2/nonfree/features2d.hpp"

#include <opengv/relative_pose/methods.hpp>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>
//#include "kalman/kalman.hpp"
#include "kalman/KalmanFilterScale.h"
#include "3dcov/cbshot.h"

#include <vector>
#include <Eigen/Dense>
#include <pointmatcher/PointMatcher.h>


using namespace std;
using namespace cv;
using namespace opengv;



void readme();

void trafoFloat7Matrix4f(float trafoFloat[7], Eigen::Matrix4f & trafoMat) {
	trafoMat.setIdentity(4, 4);

	Eigen::Quaternion<float> quat;
	quat.x() = trafoFloat[0];
	quat.y() = trafoFloat[1];
	quat.z() = trafoFloat[2];
	quat.w() = trafoFloat[3];

	float sc = quat.norm();
	quat.normalize();

	trafoMat.block(0, 0, 3, 3) = quat.toRotationMatrix();
	trafoMat.block(0, 3, 3, 1) << trafoFloat[4], trafoFloat[5], trafoFloat[6];
}
void calc3DPt(Point2f pt2D, float fx, float fy, float cx, float cy, Eigen::Vector3f & pt3D) {
	pt3D << pt2D.x * 1 / fx + (-cx / fx), pt2D.y * 1 / fy + (-cy / fy), 1.0;
	float inv_depth = -1;

	pt3D = pt3D * inv_depth;
}
void invTrafo(Eigen::Matrix4f trafo, Eigen::Matrix4f & trafoInv) {
	// **matrix.block(i,j,p,q)**    ===>   `Block of size (p,q), starting at (i,j)`
	//
	// | a b c x |
	// | d e f y |
	// | g h i z |
	// | 0 0 0 1 |

	trafoInv.block(0, 0, 3, 3) = trafo.block(0, 0, 3, 3).transpose();
	trafoInv.block(0, 3, 3, 1) = -trafo.block(0, 0, 3, 3).transpose() * trafo.block(0, 3, 3, 1); // to replace x, y ,z
}
void transform3DPt(float simTrafo[7], Eigen::Vector3f pt3D, Eigen::Vector3f & pt3DTransformed) {
	Eigen::Matrix4f trafoTmp(Eigen::Matrix4f::Identity(4, 4));
	Eigen::Vector4f pt3DTmp;

	trafoFloat7Matrix4f(simTrafo, trafoTmp);

	pt3DTmp << pt3D(0), pt3D(1), pt3D(2), 1.0;
	pt3DTmp = trafoTmp * pt3DTmp;
	pt3DTmp = pt3DTmp / pt3DTmp(3);

	pt3DTransformed(0) = pt3DTmp(0);
	pt3DTransformed(1) = pt3DTmp(1);
	pt3DTransformed(2) = pt3DTmp(2);
}

void transform3DPt(Eigen::Matrix4f simTrafo, Eigen::Vector3f pt3D, Eigen::Vector3f & pt3DTransformed) {
	Eigen::Vector4f pt3DTmp;

	pt3DTmp << pt3D(0), pt3D(1), pt3D(2), 1.0;
	pt3DTmp = simTrafo * pt3DTmp;
	pt3DTmp = pt3DTmp / pt3DTmp(3);

	pt3DTransformed(0) = pt3DTmp(0);
	pt3DTransformed(1) = pt3DTmp(1);
	pt3DTransformed(2) = pt3DTmp(2);
}
void print4x4Matrix (const Eigen::Matrix4d & matrix) {
	printf ("Rotation matrix :\n");
	printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
	printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
	printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
	printf ("Translation vector :\n");
	printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}
void pcl2dp(const pcl::PointCloud<pcl::PointXYZ>& pc,
			PointMatcher<float>::DataPoints& dp)
{
	if(pc.size() == 0)
	{
		return;
	}

	dp.features.resize(4, pc.points.size());

	for(uint32_t i=0; i<pc.points.size(); i++)
	{
		dp.features(0,i) = pc.points[i].x;
		dp.features(1,i) = pc.points[i].y;
		dp.features(2,i) = pc.points[i].z;
		dp.features(3,i) = 1.0;
	}
}


/**
 * @function main
 * @brief Main function
 */
int main( int argc, char** argv )
{
	std::ofstream file("Results/transformation.txt");
	std::cout << "Have " << argc << " arguments:" << std::endl;
    for (int i = 0; i < argc; ++i) {
        std::cout << argv[i] << std::endl;
    }
  if( argc != 5 )
  { readme(); return -1; }

  Mat img_1 = imread( argv[1], CV_LOAD_IMAGE_GRAYSCALE );
  Mat img_2 = imread( argv[2], CV_LOAD_IMAGE_GRAYSCALE );


  if( !img_1.data || !img_2.data )
  { printf(" --(!) Error reading images \n"); return -1; }

  std::string pcd_path[2];
  pcd_path[0] = std::string(argv[3]);
  pcd_path[1] = std::string(argv[4]);

  //std::string pcd_out (argv[5]);


    // Two Input Point Clouds
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source (new pcl::PointCloud<pcl::PointXYZ>);
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target (new pcl::PointCloud<pcl::PointXYZ>);
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed (new pcl::PointCloud<pcl::PointXYZ>);
   pcl::PointCloud<pcl::PointXYZ> cloud_filtered_source;
   pcl::PointCloud<pcl::PointXYZ> cloud_filtered_target;

   PointMatcher<float>::DataPoints dp_source, dp_target;

    /************************************/
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_path[0], *cloud_source) == -1) //* load the file
    {
       PCL_ERROR ("Couldn't read file pcd file \n");
        return (-1);
    }

    if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_path[1], *cloud_target) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file pcd file \n");
        return (-1);
    }
    /**************************************/


     std::cout << "Status : Loaded both the point clouds and started to compute SHOT descriptors" << std::endl;

	/**
	 * Save the I/P Point Clouds
	 */
    pcl::io::savePCDFileASCII("Results/cloud_source.pcd", *cloud_source);
	pcl::io::savePCDFileASCII("Results/cloud_target.pcd", *cloud_target);
	/**
	 * for source point cloud
	 */
	pcl::PointXYZ minPt, maxPt;
	
	pcl::getMinMax3D (*cloud_source, minPt, maxPt);
	float source_filter_volume = (maxPt.x-minPt.x)*(maxPt.y-minPt.y)*(maxPt.z-minPt.z);
	cout << "source_filter_volume" << source_filter_volume << endl;
	
	pcl::getMinMax3D (*cloud_target, minPt, maxPt);
	float target_filter_volume = (maxPt.x-minPt.x)*(maxPt.y-minPt.y)*(maxPt.z-minPt.z);
     cout << "target_filter_volume" << target_filter_volume << endl;
	
 if(max(source_filter_volume,target_filter_volume) / min(source_filter_volume,target_filter_volume) > 3 ){
	    //-- Step 1: Detect the keypoints using SURF Detector
  cout << "Have Scale Difference" << endl;
  Ptr<FeatureDetector> detector = new SiftFeatureDetector(0, 4, 0.04, 10, 1.0);
  std::vector<KeyPoint> keypoints_1, keypoints_2;
  detector->detect( img_1, keypoints_1 );
  detector->detect( img_2, keypoints_2 );
  
  Ptr<DescriptorExtractor> extractor = new SiftDescriptorExtractor();
  //-- Step 2: Calculate descriptors (feature vectors)
  // FlannBasedMatcher matcher;
  Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("FlannBased");

 // Ptr<BOWImgDescriptorExtractor> bide = new BOWImgDescriptorExtractor(extractor, matcher);
  
  Mat descriptors_1, descriptors_2;
  
  extractor->compute(img_1, keypoints_1, descriptors_1 );
  extractor->compute(img_2, keypoints_2, descriptors_2 );

  //-- Step 3: Matching descriptor vectors using FLANN matcher
  std::vector< DMatch > matches;
  matcher->match( descriptors_1, descriptors_2, matches );
 
  double max_dist = 0; 
  double min_dist = 100;


  //-- Quick calculation of max and min distances between keypoints
  for( int i = 0; i < descriptors_1.rows; i++ )
  { 
	  double dist = matches[i].distance;
    if( dist < min_dist ) min_dist = dist;
    if( dist > max_dist ) max_dist = dist;
  }
 //cout << "192" << endl;
  printf("-- Max dist : %f \n", max_dist );
  printf("-- Min dist : %f \n", min_dist );

  std::vector< DMatch > good_matches;

  for( int i = 0; i < descriptors_1.rows; i++ )
  { if( matches[i].distance <= max(4*min_dist, 10.0) )
    { good_matches.push_back( matches[i]); }
  }

  //-- Draw only "good" matches
  Mat img_matches;
  drawMatches( img_1, keypoints_1, img_2, keypoints_2,
               good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

  //-- Show detected matches
  imshow( "Good Matches", img_matches );
  imwrite("Results/refined_matches.jpg", img_matches);
  imwrite("Results/source_kf.jpg", img_1);
  imwrite("Results/target_kf.jpg", img_2);
  for( int i = 0; i < (int)good_matches.size(); i++ )
  { printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx ); }

    bearingVector_t bearingVectorQuery, bearingVectorTest;
	bearingVectors_t bearingVectorsQuery;
	bearingVectors_t bearingVectorsTest;


	vector<Point2f> matchedPtsQuery, matchedPtsTest;
	for (int i = 0; i < good_matches.size(); ++i)
	{
		Point2f matchedPtQuery = keypoints_1[good_matches[i].queryIdx].pt;
		Point2f matchedPtTest = keypoints_2[good_matches[i].trainIdx].pt;

		matchedPtsQuery.push_back(matchedPtQuery);
		matchedPtsTest.push_back(matchedPtTest);

		bearingVectorQuery[0] = matchedPtQuery.x / sqrt(matchedPtQuery.x * matchedPtQuery.x + matchedPtQuery.y * matchedPtQuery.y + 1.0);
		bearingVectorQuery[1] = matchedPtQuery.y / sqrt(matchedPtQuery.x * matchedPtQuery.x + matchedPtQuery.y * matchedPtQuery.y + 1.0);
		bearingVectorQuery[2] = 1.0; //sqrt(1.0 - matchedPtQuery.x*matchedPtQuery.x - matchedPtQuery.y*matchedPtQuery.y);
		bearingVectorQuery.normalize();

		bearingVectorTest[0] = matchedPtTest.x / sqrt(matchedPtTest.x * matchedPtTest.x + matchedPtTest.y * matchedPtTest.y + 1.0);
		bearingVectorTest[1] = matchedPtTest.y / sqrt(matchedPtTest.x * matchedPtTest.x + matchedPtTest.y * matchedPtTest.y + 1.0);
		//sqrt(1.0 - matchedPtTest.x*matchedPtTest.x - matchedPtTest.y*matchedPtTest.y)
		bearingVectorTest[2] = 1.0; //sqrt(1.0 - matchedPtTest.x*matchedPtTest.x - matchedPtTest.y*matchedPtTest.y);
		bearingVectorTest.normalize();

		bearingVectorsQuery.push_back(bearingVectorQuery);
		bearingVectorsTest.push_back(bearingVectorTest);
		
	}
	// create the central relative adapter
	relative_pose::CentralRelativeAdapter adapter(bearingVectorsQuery, bearingVectorsTest);
	// create a RANSAC object
	sac::Ransac<sac_problems::relative_pose::CentralRelativePoseSacProblem> ransac;
	//  create a CentralRelativePoseSacProblem
	//  (set algorithm to STEWENIUS, NISTER, SEVENPT, or EIGHTPT)
	std::shared_ptr<sac_problems::relative_pose::CentralRelativePoseSacProblem>
		relposeproblem_ptr(
			new sac_problems::relative_pose::CentralRelativePoseSacProblem(
				adapter,
				sac_problems::relative_pose::CentralRelativePoseSacProblem::NISTER)); // nine point

	// run ransac
	ransac.sac_model_ = relposeproblem_ptr;
	ransac.threshold_ = 2 * (1.0 - cos(atan(sqrt(2.0) * 0.5 / 467.87)));
	ransac.max_iterations_ = 250;

	ransac.computeModel();

	//  get the result
	transformation_t pCamTestCamQueryOpenGV = ransac.model_coefficients_;

	Eigen::Matrix4f iG(Eigen::Matrix4f::Identity(4, 4));
 
	Eigen::Matrix4d iGdouble(Eigen::Matrix4d::Identity(4, 4));
	iGdouble.block(0, 0, 3, 4) = pCamTestCamQueryOpenGV;
	iG = iGdouble.cast<float>();
	
	// Will be used in Kalman Filter

	
	if (file.is_open())
	{
		file << "Here is the iG:\n" << iG.cast<double>() << '\n';
	}

	Eigen::Matrix4d m_matrix = iG.cast<double>();
	printf ("Rotation matrix :\n");
	printf ("    | %6.3f %6.3f %6.3f | \n", m_matrix (0, 0), m_matrix (0, 1), m_matrix (0, 2));
	printf ("R = | %6.3f %6.3f %6.3f | \n", m_matrix (1, 0), m_matrix (1, 1), m_matrix (1, 2));
	printf ("    | %6.3f %6.3f %6.3f | \n", m_matrix (2, 0), m_matrix (2, 1), m_matrix (2, 2));
	printf ("Translation vector :\n");
	printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", m_matrix (0, 3), m_matrix (1, 3), m_matrix (2, 3));
 
  	/**
	 * To calculate the scale
	 *
	 */

  float KF_fx = 467.87;
  float KF_fy = 514.317;
  float KF_cx = 271.828;
  float KF_cy = 246.906;
  KalmanFilterScale filter;
  KalmanFilterScale::Vector x(1);
	x(1) = 1.0;
	static const float _P0[] = {10.0};
	KalmanFilterScale::Matrix P0(1, 1, _P0);
  filter.init(x, P0);




	Eigen::Vector3f matched3DPtQuery(0.0, 0.0, 0.0), matched3DPtTest(0.0, 0.0, 0.0);
	
	cout << "good matches size" << good_matches.size() << endl;
	for (int i = 0; i < good_matches.size(); ++i)
	{
		//cout << "main_in" << i << endl;
		Point2f matched2DPtQuery = keypoints_1[good_matches[i].queryIdx].pt;
		Point2f matched2DPtTest = keypoints_2[good_matches[i].trainIdx].pt;

		calc3DPt(matched2DPtQuery, KF_fx, KF_fy, KF_cx, KF_cy, matched3DPtQuery);
	//	cout << "matched3DPtQuery " << matched3DPtQuery << endl;
		

		calc3DPt(matched2DPtTest, KF_fx, KF_fy, KF_cx, KF_cy, matched3DPtTest);
			
		//	cout << "matched3DPtTest " << matched3DPtTest << endl;
		Eigen::Matrix4f iGTrafoInv(Eigen::Matrix4f::Identity(4, 4));
		invTrafo(iG, iGTrafoInv);
		transform3DPt(iGTrafoInv, matched3DPtQuery, matched3DPtQuery);

		//	cout << "matched3DPtQuery " << matched3DPtQuery << endl;
		KalmanFilterScale::Vector u(0);
		KalmanFilterScale::Vector z(3);
		
		for (int ii = 0; ii < 3; ++ii)
		{

			float measurement = matched3DPtTest(ii) / matched3DPtQuery(ii);
			z(ii+1) = measurement;
			
		
		}
		filter.step(u, z);
 
	} 

	cout << "scale estimate:	" << filter.getX()(1) << endl;

	Eigen::Matrix4f scale(Eigen::Matrix4f::Identity(4, 4));

	scale.block(0, 0, 3, 3) = 1 / filter.getX()(1) * Eigen::Matrix3f::Identity(3, 3);
  
    cout << "Scale Matrix" << endl;
    print4x4Matrix(scale.cast<double>()); 

	if (file.is_open())
	{
		file << "Here is the Scale Matrix:\n" << scale.cast<double>() << '\n';
	}
     pcl::transformPointCloud(*cloud_source, *cloud_source, scale);
	 pcl::io::savePCDFileASCII("Results/cloud_source_scale.pcd", *cloud_source);
}

	/**
	 * for source point cloud
	 */
	
	pcl::getMinMax3D (*cloud_source, minPt, maxPt);
	cout << "Max x: " << maxPt.x << endl;
	cout << "Max y: " << maxPt.y << endl;
	cout << "Max z: " << maxPt.z << endl;
	cout << "Min x: " << minPt.x << endl;
	cout << "Min y: " << minPt.y << endl;
	cout << "Min z: " << minPt.z << endl;
		
	
    pcl::PassThrough<pcl::PointXYZ> pass;
  	pass.setInputCloud (cloud_source);
  	pass.setFilterFieldName ("y");
	
	float min_filter = minPt.y;
    float max_filter = maxPt.y-minPt.y;
	max_filter = maxPt.y - max_filter/3;
	cout << "height_cloud" << endl;
  pass.setFilterLimits (min_filter,max_filter);
   pass.setFilterLimitsNegative (true);
  	pass.filter (cloud_filtered_source);
	

/**
 * for target point cloud
 */
	pcl::getMinMax3D (*cloud_target, minPt, maxPt);
	cout << "Max x: " << maxPt.x << endl;
	cout << "Max y: " << maxPt.y << endl;
	cout << "Max z: " << maxPt.z << endl;
	cout << "Min x: " << minPt.x << endl;
	cout << "Min y: " << minPt.y << endl;
	cout << "Min z: " << minPt.z << endl;
		
	pass.setInputCloud (cloud_target);
  	pass.setFilterFieldName ("y");

	min_filter = minPt.y;
    max_filter = maxPt.y-minPt.y;
	max_filter = maxPt.y - max_filter/3;

	cout << "height_cloud" << endl;
  pass.setFilterLimits (min_filter,max_filter);
   pass.setFilterLimitsNegative (true);
  	pass.filter (cloud_filtered_target);
	
	
	
	 PointMatcher<float>::ICP icp;

	 std::ifstream f("../icp.yaml");
	 if(f.good())
	 {
	 	icp.loadFromYaml(f);
	 	f.close();
	 }
	 else
	 {
	 	icp.setDefault();
	 	std::cout << "I: icp setDefault " << std::endl;
	 }
	
	pcl2dp(*cloud_source, dp_source);
	pcl2dp(*cloud_target, dp_target);
	
	
	PointMatcher<float>::TransformationParameters T = icp(dp_source,dp_target);
	
	 Eigen::Matrix4f tmp = T;
	 Eigen::Affine3d tf(tmp.cast<double>());
	
	std::cout <<  "Final transformation :\n" << tmp.cast<double>() << std::endl;
   	Eigen::MatrixXd icp_cov(6, 6);
   	Eigen::Matrix4f final_transformation(Eigen::Matrix4f::Identity(4, 4));
   	cbshot::calculate_cov(*cloud_source, *cloud_target, tmp , final_transformation, icp_cov);

   	if (file.is_open())
 	{
 		file << "Here is the final_transformation:\n" <<  final_transformation << '\n';
		file << "Here is the Covarience:\n" <<  icp_cov << '\n'; 
 	}
	
 	pcl::transformPointCloud(*cloud_source, *cloud_source, tf);
   	pcl::io::savePCDFileASCII("Results/cloud_s_transformed.pcd", *cloud_source);

   return 0;
}

/**
 * @function readme
 */
void readme()
{ printf(" Usage: ./main <img1> <img2> <pointcloud2> <pointcloud1> \n"); }
