#pragma once
#include "api.h"
#include "Pose.hpp"
#include "AprilTagLocations.hpp"
#include <atomic>
#include <algorithm> 
#include "pros/serial.hpp"
#include <cstring>
#include "DriveLocalizerConstants.hpp"

struct VisionTag {
    int32_t id;
    float x;
    float y;
    float heading;
};

class AprilTagManager {
public:
    AprilTagManager(DriveLocalizerConstants constants = DriveLocalizerConstants());

    Pose getRobotPose(Pose estRobotPose);

    void jetBrainsComs();
    std::vector<Pose> getTagPosesCamRelitive() { return {Pose(), Pose(), Pose()}; };
private:

    std::vector<VisionTag> tagsList;
    std::vector<VisionTag> prevTagsList;
    pros::Mutex tags_mutex; 
    std::vector<uint8_t> serialBuffer;
    pros::Serial my_serial;

    DriveLocalizerConstants constants;
    bool onceBuffer = false;
    bool prevTagsListStored = false;
};
