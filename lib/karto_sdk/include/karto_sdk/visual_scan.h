#ifndef VISUAL_CONSTRAINT_SCAN_H
#define VISUAL_CONSTRAINT_SCAN_H

#include "karto_sdk/Karto.h"

namespace karto {

class VisualConstraintScan : public SensorData {
public:
  VisualConstraintScan(const Name& rSensorName, const Pose2& pose)
    : SensorData(rSensorName), visualPose(pose) {}
  
  virtual ~VisualConstraintScan() {}

  void SetVisualPose(const Pose2& pose) { visualPose = pose; }
  Pose2 GetVisualPose() const { return visualPose; }

private:
  Pose2 visualPose;  // Visual pose estimate from camera-based loop closure
  
};

}  // namespace karto

#endif // VISUAL_CONSTRAINT_SCAN_H