#pragma once
#include "Pose.hpp"

struct Paths {
    inline static std::vector<Pose> toRightWallZig = {
        Pose(-63.5,-48.6),
        Pose(13.8,4.0),
        Pose(-46.5,-67.2),
        Pose(74,-104.3),
        Pose(60.2,-10)
    };

    inline static std::vector<Pose> toLeftWallZag = {
        Pose(60.2,-10),
        Pose(-18.0,0),
        Pose(91.8,-93.1),
        Pose(24,-48),
        Pose(39.6,-157.7),
        Pose(-96.0,96.0),
        Pose(3.0,-66.3),
        Pose(-62.7,-62.4)
    };
};
