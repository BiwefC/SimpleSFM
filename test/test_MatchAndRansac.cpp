int main(int argc, char** argv)
{
    // while(1);
    FRAME frame1,frame2;
    RESULT_OF_PNP result;

    frame1.rgb = cv::imread("../data/rgb1.png");
    frame1.depth = cv::imread("../data/depth1.png", -1);
    frame2.rgb = cv::imread("../data/rgb2.png");
    frame2.depth = cv::imread("../data/depth2.png");

    CAMERA_INTRINSIC_PARAMETERS camera;
    camera.scale = 1000;
    camera.cx = 325.5;
    camera.cy = 253.5;
    camera.fx = 518.0;
    camera.fy = 519.0;
    CompuFeaAndDesp(FRAME& frame1);
    CompuFeaAndDesp(FRAME& frame2);

    result = RESULT_OF_PNP MatchAndRansac(FRAME& frame1, FRAME& frame2, CAMERA_INTRINSIC_PARAMETERS& Camera);

    std::cout<<"inlPoint: "<<result.inlPoint<<std::endl;
    std::cout<<"R="<<result.rvec<<std::endl;
    std::cout<<"t="<<result.tvec<<std::endl;
    
    return 0;
}