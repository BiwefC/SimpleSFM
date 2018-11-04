#include <iostream>

#include "SLAMBase.hpp"

int main(int argc, char** argv)
{
    Frame frame(1, "../data");

    frame.ComputeFeatAndDesp();
    cv::Mat imgOutput;
    cv::drawKeypoints(frame.rgb, frame.feat, imgOutput, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::imshow( "keypoints", imgOutput);
    cv::imwrite( "./data/keypoints.png", imgOutput);
    cv::waitKey(0);
    return 0;
}