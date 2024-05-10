#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Pose3.h>

namespace gtsam {

    class State {
  private:
    Pose3 pose_;       // Rotation from global to Camera, Position of camera/robot in global
    double stateId_;   // State ID in the graph

  public:
    // Default Constructor
    State() : pose_(Pose3()), stateId_(-1) {}
    
    // Copy Constructor
    State(const State& state) {
      this->pose_     = state.pose_;
      this->stateId_ = state.stateId_;
    }
   
    // Constructor
    State(const Pose3& pose, const double stateID = -1) {
      this->pose_     = pose;
      this->stateId_  = stateID;
    }

    // Set pose
    void set_pose(const Pose3& pose) {
      this->pose_ = pose;
    }

    // Return pose as Pose3
    const Pose3& pose() const {
      return pose_;
    }

    // Return rotation as Quaternion
    Quaternion q() const {
      return pose().rotation().toQuaternion();
    }

    // Return translation as Vector3
    Vector3 p() const {
      return pose().translation();
    }

    double stateId() const {
      return stateId_;
    }

    /// How this node gets printed in the ostream
    GTSAM_EXPORT
    friend std::ostream &operator<<(std::ostream &os, const State& state) {
        os << "[STATE]: q = " << std::fixed << state.q().x() << ", " << std::fixed << state.q().y() << ", "
                              << std::fixed << state.q().z() << ", " << std::fixed << state.q().w() << " | ";
//        os << "x, y, theta = "  << std::fixed << state.p()(0) << ", "  << std::fixed << state.p()(1)  << ", " << std::fixed << state.yaw()  << " | ";
        os << "p = "  << std::fixed << state.p()(0)  << ", "  << std::fixed << state.p()(1)  << ", " << std::fixed << state.p()(2)  << " | ";
        return os;
    }

    /// Print function for this node
    void print(const std::string& s = "") const {
      std::cout << s << *this << std::endl;
    }

}; // State class

} // namespace gtsam
