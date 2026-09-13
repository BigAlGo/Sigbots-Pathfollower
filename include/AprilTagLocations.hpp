#pragma once
#include "Pose.hpp"

struct AprilTagLocations
{
    inline static double offsetAmount = 1.745;
    inline static double pi = std::numbers::pi;

    inline static Pose upOffset =    Pose(0,  offsetAmount,  pi / 2);
    inline static Pose downOffset =  Pose(0, -offsetAmount, -pi / 2 );
    inline static Pose rightOffset = Pose( offsetAmount, 0,  0);
    inline static Pose leftOffset =  Pose(-offsetAmount, 0,  pi);

    inline static Pose tag0 = Pose(  0,   0);
    inline static Pose tag1 = Pose(-48,  24);
    inline static Pose tag2 = Pose(-48, -24);
    inline static Pose tag3 = Pose(-24, -48);
    inline static Pose tag4 = Pose( 24, -48);

    // measured camera relative (depth, lateral)
    inline static Pose cameraOffset = Pose(-0/*set me*/, 0); 
    
};
