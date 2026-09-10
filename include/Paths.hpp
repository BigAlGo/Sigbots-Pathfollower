#pragma once
#include "Pose.hpp"

struct Paths {
    inline static std::vector<Pose> toRightWallZig = {
        Pose(-68.0,-48.6),
        Pose(3.6,10.0),
        Pose(-19.2,-76.1),
        Pose(80,-101.3),
        Pose(60.5,-13.3)
    };

    inline static std::vector<Pose> toLeftWallZag = {
        Pose(-8.8,3),
        Pose(83.7,-94.0),
        Pose(47.7,-159.3),
        Pose(-96.0,96.0),
        Pose(3.0,-66.3),
        Pose(-62.7,-62.4)
    };
};
