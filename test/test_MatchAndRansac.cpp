#include "SLAMBase.hpp"

int main(int argc, char** argv)
{
    // while(1);
    Frame frame1(255, "../../data");
    Frame frame2(259, "../../data");
    Result_of_PnP result;

    Camera_Intrinsic_Parameters camera;
    camera.scale = 1000;
    camera.cx = 325.5;
    camera.cy = 253.5;
    camera.fx = 518.0;
    camera.fy = 519.0;
    frame1.ComputeFeatAndDesp();
    frame2.ComputeFeatAndDesp();

    result = MatchAndRansac(frame1, frame2, camera);

    std::cout<<"inlPoint: "<<result.inlPoint<<std::endl;
    std::cout<<"R = "<<result.rvec<<std::endl;
    std::cout<<"t = "<<result.tvec<<std::endl;

    return 0;
}