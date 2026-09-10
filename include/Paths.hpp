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

    inline static std::vector<Pose> toBottomWallZigy = {
        Pose(60.2,-10),
        Pose(-6.3,2.7),
        Pose(78,-72),
        Pose(26,-64)
    };

    inline static std::vector<Pose> toLeftWallZag = {
        Pose(26,-64),
        Pose(-10.5,-56.3),
        Pose(-52.6,21.8),
        Pose(-36.4,-68.8),
        Pose(-62.7,-62.4)
    };
};
