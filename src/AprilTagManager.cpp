#include "AprilTagManager.hpp"

struct TagData {
    int32_t id;
    float x;
    float y;
    float heading;
};


AprilTagManager::AprilTagManager(DriveLocalizerConstants constants) 
    : my_serial(1, 115200) {
    this->constants = constants;
}

/**
 * correct format for serial port is 
 * 0x00
 * 0x01
 */

void AprilTagManager::jetBrainsComs() {
    if (!onceBuffer) {
        serialBuffer.clear();
        onceBuffer = true;
    }

    while (my_serial.get_read_avail() > 0) {
        serialBuffer.push_back(my_serial.read_byte());
    }

    // makes sure that it starts at 0x00
    while (!serialBuffer.empty() && serialBuffer.front() != 0x00) {
        serialBuffer.erase(serialBuffer.begin());
    }

    while (serialBuffer.size() >= 2) {
    
        // Scan sequentially to find the frame's true ending delimiter (0x02)
        int endPacketIndex = -1;
        for (size_t i = 1; i < serialBuffer.size(); ++i) {
            if (serialBuffer[i] == 0x02) {
                endPacketIndex = i;
                break;
            }
        }

        // If the 0x02 packet terminator hasn't finished transferring yet, stop and wait
        if (endPacketIndex == -1) {
            break; 
        }

        tagsList.clear();
        int serialIndex = 1;
        bool packetCorrupted = false;

        while (serialIndex < endPacketIndex) {
            // Read block header byte
            uint8_t blockHeader = serialBuffer[serialIndex];
            serialIndex++;

            if (blockHeader == 0x01) {
                // make sure there are 16 payload bytes available before the 0x02 flag
                if (serialIndex + 16 > endPacketIndex) {
                    packetCorrupted = true;
                    break;
                }

                uint8_t idBytes[4];
                uint8_t xBytes[4];
                uint8_t yBytes[4];
                uint8_t headingBytes[4];

                // Safely fill extraction buffers relative to the current position
                for (int j = 0; j < 4; ++j, ++serialIndex) idBytes[j] = serialBuffer[serialIndex];
                for (int j = 0; j < 4; ++j, ++serialIndex) xBytes[j] = serialBuffer[serialIndex];
                for (int j = 0; j < 4; ++j, ++serialIndex) yBytes[j] = serialBuffer[serialIndex];
                for (int j = 0; j < 4; ++j, ++serialIndex) headingBytes[j] = serialBuffer[serialIndex];

                int32_t tagId;
                float tagX, tagY, tagHeading;

                std::memcpy(&tagId,      idBytes,       4); 
                std::memcpy(&tagX,       xBytes,        4); 
                std::memcpy(&tagY,       yBytes,        4); 
                std::memcpy(&tagHeading, headingBytes,  4); 

                tagsList.push_back({tagId, tagX, tagY, tagHeading});
            } else {
                packetCorrupted = true;
                break;
            }
        }

        if (!packetCorrupted) {
            prevTagsList = tagsList;
            prevTagsListStored = true;
        } else if (prevTagsListStored) {
            tagsList = prevTagsList; // Revert to previous good frame data on error
        }

        serialBuffer.erase(serialBuffer.begin(), serialBuffer.begin() + endPacketIndex + 1);
    }
}

// returns estRobot if no tags are valid
Pose AprilTagManager::getRobotPose(Pose estRobot) {

    jetBrainsComs();

    std::vector<Pose> possibleRobotPoses;

    for (VisionTag tag : tagsList) {
        // roatation aroun field
        for (int i = -1; i < 2; i += 2) {
            // rotation around each goal 
            for (int j = 0; j < 4; ++j) {
                
                double Vx = (double)((tag.x + AprilTagLocations::cameraOffset.x) * i);
                double Vy = (double)((tag.y + AprilTagLocations::cameraOffset.y) * i);
                double Vh = (double)(std::numbers::pi / 2) - tag.heading;


                Pose goalRotOffset;
                switch (j) {
                    case 0:
                        goalRotOffset = AprilTagLocations::upOffset;
                        break;
                    case 1:
                        goalRotOffset = AprilTagLocations::downOffset;
                        break;
                    case 2:
                        goalRotOffset = AprilTagLocations::leftOffset;
                        break;
                    case 3:
                        goalRotOffset = AprilTagLocations::rightOffset;
                        break;
                }

                Vh += goalRotOffset.heading; 

                double Rh = Vh;

                double Rx = std::abs((std::sqrt(std::pow(Vx, 2) + std::pow(Vy, 2))) * std::sin((std::numbers::pi / 2) - (std::numbers::pi - Vh - std::atan2(Vy, Vx))));

                double Ry = (std::sqrt(std::pow(Vx, 2) + std::pow(Vy, 2))) * std::cos((std::numbers::pi / 2) - (std::numbers::pi - Vh - std::atan2(Vy, Vx)));

                //this math is stoiped but is has to be here
                double VxSign = 1;
                if (Vx != 0.0) {
                    VxSign = std::abs(Vx) / Vx;
                }

                Pose runnerPose = Pose(Rx, Ry * VxSign, Rh);

                Pose goalNumOffset;
                switch (tag.id) {
                    case 0:
                        goalNumOffset = AprilTagLocations::tag0;
                        break;
                    case 1:
                        goalNumOffset = AprilTagLocations::tag1;
                        break;
                    case 2:
                        goalNumOffset = AprilTagLocations::tag2;
                        break;
                    case 3:
                        goalNumOffset = AprilTagLocations::tag3;
                        break;
                    case 4:
                        goalNumOffset = AprilTagLocations::tag4;
                        break;
                    default:
                        //Not a valid ID
                        continue;
                }

                


                Pose robotPose = Pose(runnerPose.x + goalRotOffset.x + goalNumOffset.x, 
                                      runnerPose.y + goalRotOffset.y + goalNumOffset.y, 
                                      runnerPose.heading + (std::numbers::pi * 0)); // to account for first add of pi/2
                possibleRobotPoses.push_back(robotPose);
            }
        }
    }

    std::vector<Pose> likelyRobotPoses;
    for (Pose possible : possibleRobotPoses) {
        if (possible.distFrom(estRobot) < constants.maxEstimatedDistance) {
            likelyRobotPoses.push_back(possible);
        }
    }

    if (likelyRobotPoses.empty()) {
        return estRobot;
    }

    double accumulateX = 0.0;
    double accumulateY = 0.0;
    double accumulateSin = 0.0;
    double accumulateCos = 0.0;
    for (Pose likely : likelyRobotPoses) {
        accumulateX += likely.x;
        accumulateY += likely.y;
        accumulateSin += std::sin(likely.heading);
        accumulateCos += std::cos(likely.heading);
    }

    double num = likelyRobotPoses.size();

    Pose cameraRobotPose = Pose(accumulateX / num, 
                                accumulateY / num, 
                                std::atan2(accumulateSin, accumulateCos)); // calculates ratio so doesnt care about accumulation

    return cameraRobotPose;
}