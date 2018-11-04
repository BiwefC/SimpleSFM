#include "SLAMBase.hpp"

int main(int argc, char** argv)
{
    Frame frame1,frame2;
    Result_of_PnP result;
    Eigen::Isometry3d T;

    Frame frame1(1, "../data");
    Frame frame2(2, "../data");

    Camera_Intrinsic_Parameters camera;
    camera.scale = 1000;
    camera.cx = 325.5;
    camera.cy = 253.5;
    camera.fx = 518.0;
    camera.fy = 519.0;
    frame1.ComputeFeatAndDesp();
    frame2.ComputeFeatAndDesp();

    result = MatchAndRansac(frame1, frame2, camera);
    std::cout<< result.rvec<< std::endl;
    std::cout<< result.tvec<< std::endl;
    
    T = RvecTvec2Mat(result.rvec, result.tvec);

    // std::cout<< T<<std::endl;

}