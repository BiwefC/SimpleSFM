#include "SLAMBase.hpp"

int main(int argc, char** argv)
{
	Frame frame1,frame2;
	Result_of_PnP result;
	Eigen::Isometry3d T;

	frame1.rgb = cv::imread("../data/rgb1.png");
	frame1.depth = cv::imread("../data/depth1.png", -1);
	frame2.rgb = cv::imread("../data/rgb2.png");
	frame2.depth = cv::imread("../data/depth2.png");

	Camera_Intrinsic_Parameters camera;
	camera.scale = 1000;
	camera.cx = 325.5;
	camera.cy = 253.5;
	camera.fx = 518.0;
	camera.fy = 519.0;
	frame1.ComputeFeatAndDesp();
	frame2.ComputeFeatAndDesp();

	PointCloud::Ptr last_pc = Image2PointCloud(frame1.rgb, frame1.depth, camera);

	result = MatchAndRansac(frame1, frame2, camera);    
	T = RvecTvec2Mat(result.rvec, result.tvec);
	PointCloud::Ptr new_pc = UpdatePointCloud(last_pc, frame2, T, camera);
	

	pcl::io::savePCDFile("../data/result.pcd", *new_pc);
  	cout<<"Final result saved."<<endl;
	
	pcl::visualization::CloudViewer viewer( "viewer" );
	viewer.showCloud(new_pc);
	 while( !viewer.wasStopped() )
	      {
              }

	return 0;

}