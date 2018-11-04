# include "SLAMBase.hpp"

PointCloud::Ptr image2PointCloud( cv::Mat& rgb, cv::Mat& depth, CAMERA_INTRINSIC_PARAMETERS& camera )
{
    PointCloud::Ptr cloud ( new PointCloud );

    for (int m = 0; m < depth.rows; m+=2)
        for (int n=0; n < depth.cols; n+=2)
        {
            // 获取深度图中(m,n)处的值
            ushort d = depth.ptr<ushort>(m)[n];
            // d 可能没有值，若如此，跳过此点
            if (d == 0)
                continue;
            // d 存在值，则向点云增加一个点
            PointT p;

            // 计算这个点的空间坐标
            p.z = double(d) / camera.scale;
            p.x = (n - camera.cx) * p.z / camera.fx;
            p.y = (m - camera.cy) * p.z / camera.fy;

            // 从rgb图像中获取它的颜色
            // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
            p.b = rgb.ptr<uchar>(m)[n*3];
            p.g = rgb.ptr<uchar>(m)[n*3+1];
            p.r = rgb.ptr<uchar>(m)[n*3+2];

            // 把p加入到点云中
            cloud->points.push_back( p );
        }
    // 设置并保存点云
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;

    return cloud;
}

cv::Point3f point2dTo3d( cv::Point3f& point, CAMERA_INTRINSIC_PARAMETERS& camera )
{
    cv::Point3f p; // 3D 点
    p.z = double( point.z ) / camera.scale;
    p.x = ( point.x - camera.cx) * p.z / camera.fx;
    p.y = ( point.y - camera.cy) * p.z / camera.fy;
    return p;
}

void CompuFeaAndDesp(FRAME& frame)
{
	cv::Ptr<cv::FeatureDetector> fea_detec;
	cv::Ptr<cv::DescriptorExtractor> fea_desp;

	fea_detec = cv::ORB::create();
	fea_desp  = cv::ORB::create();

	fea_detec->detect( frame.rgb, frame.fea );
	fea_desp->compute( frame.rgb, frame.fea, frame.desp );


	return;
}


RESULT_OF_PNP MatchAndRansac(FRAME& frame1, FRAME& frame2, CAMERA_INTRINSIC_PARAMETERS& Camera)
{
	//read data from data floder
	cv::Mat pic1_rgb = frame1.rgb;
	cv::Mat pic2_rgb = frame2.rgb;
	cv::Mat pic1_depth = frame1.depth;
	cv::Mat pic2_depth = frame2.depth;
  cv::Mat pic1_desp = frame1.desp;
  cv::Mat pic2_desp = frame2.desp;

	vector< cv::KeyPoint > fea1 = frame1.fea;
	vector< cv::KeyPoint > fea2 = frame2.fea;

	//output the size of feature point
	std::cout<<"Key points of two images: "<<fea1.size()<<", "<<fea2.size()<<std::endl;

	//match all feature points
	vector< cv::DMatch > matches;
	cv::BFMatcher matcher;
	matcher.match(pic1_desp, pic2_desp, matches);
	std::cout<<"Find total "<<matches.size()<<" matches."<<std::endl;

	//output match
	cv::Mat imgMatch;
	cv::drawMatches( pic1_rgb, fea1, pic2_rgb, fea2, matches, imgMatch);
	cv::imshow( "matches", imgMatch);
	cv::imwrite( "./data/matches.png", imgMatch);
	cv::waitKey(0);

	//Selete feature point match
	//rule:delete points that the distance longer than forth min distance
    // first step : find the min distance
  vector< cv::DMatch > goodMatch;
  double min_dis = 10000000;

	for(size_t i = 0; i < matches.size(); i++)
	{
		if(matches[i].distance < min_dis) min_dis = matches[i].distance;
	}

	std::cout<<"min_dis = "<<min_dis<<std::endl;

	//second:slecte the good feature points
	for(size_t i = 0; i < matches.size(); i++)
	{
		if (matches[i].distance < 10*min_dis) goodMatch.push_back(matches[i]);
	}

	//output goodMatch
	std::cout<<"goodMatch = "<<goodMatch.size()<<std::endl;

	cv::drawMatches( pic1_rgb, fea1, pic2_rgb, fea2, goodMatch, imgMatch);
	cv::imshow( "good_matches", imgMatch);
	cv::imwrite( "./data/good_matches.png", imgMatch);
	cv::waitKey(0);

	//the next part: use the RANSAC to optimize

	//inital
	vector<cv::Point3f> pic_obj;
	vector<cv::Point2f> pic_img;

	//get the depth of pic1
	for(size_t i = 0; i < goodMatch.size(); i++)
	{
		cv::Point2f p = fea1[goodMatch[i].queryIdx].pt;
		ushort d = pic1_depth.ptr<ushort>(int(p.y))[int(p.x)];
		if(d == 0) continue;
		pic_img.push_back( cv::Point2f(fea2[goodMatch[i].trainIdx].pt));

		//(u,v,d) to (x,y,z)
		cv::Point3f pt (p.x, p.y, d);
		cv::Point3f pd = point2dTo3d(pt, Camera);
		pic_obj.push_back(pd);

	}

	double Camera_matrix[3][3] = {{Camera.fx, 0, Camera.cx},{0, Camera.fy, Camera.cy},{0, 0, 1}};

	//build the Camera matrix
	cv::Mat cameraMatrix(3, 3, CV_64F, Camera_matrix);
	cv::Mat rvec, tvec, inlPoint;

	cv::solvePnPRansac(pic_obj, pic_img, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 8, 0.99, inlPoint);

	std::cout<<"inlPoint: "<<inlPoint.rows<<std::endl;
	std::cout<<"R="<<rvec<<std::endl;
	std::cout<<"t="<<tvec<<std::endl;

	vector<cv::DMatch> matchShow;
    for (size_t i = 0; i < inlPoint.rows; i++)
    {
        matchShow.push_back(goodMatch[inlPoint.ptr<int>(i)[0]]);
    }

	//out inlpoint
    cv::drawMatches( pic1_rgb, fea1, pic2_rgb, fea2, matchShow, imgMatch);
    cv::imshow( "inlPoint matches", imgMatch);
    cv::imwrite( "./data/inlPoint.png", imgMatch);
    cv::waitKey(0);

  RESULT_OF_PNP result;
	result.rvec = rvec;
	result.tvec = tvec;
	result.inlPoint = inlPoint.rows;
	return result;
}