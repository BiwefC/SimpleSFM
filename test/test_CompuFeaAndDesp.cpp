#include <iostream>

#include "SLAMBase.hpp"

int main(int argc, char** argv)
{
    // while(1);
    FRAME frame;

    frame.rgb = cv::imread("../data/rgb.png");
    frame.depth = cv::imread("../data/depth.png", -1);


    CompuFeaAndDesp(frame);
    cv::Mat imgOutput;
    cv::drawKeypoints(frame.rgb, frame.fea, imgOutput, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::imshow( "keypoints", imgOutput);
    cv::imwrite( "./data/keypoints.png", imgOutput);
    cv::waitKey(0);
    return 0;
}