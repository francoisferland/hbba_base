#ifndef IW_TOOLS_COMMON_HPP
#define IW_TOOLS_COMMON_HPP

/// \file common.hpp A set of common functions for generating HBBA desires.

#include <hbba_msgs/AddDesires.h>
#include <hbba_msgs/RemoveDesires.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <string>
#include <sstream>

namespace iw_tools
{
    /// \brief Generate a pose facing a landmark at a specific distance.
    ///
    /// The distance is calculated along the Z axis of the landmark.
    ///
    /// \param goal     The landmark pose message.
    /// \param dist     The distance along Z.
    /// \param goal_t   The resulting goal pose message.
    inline void facingPose(
        const geometry_msgs::PoseStamped& goal,
        const double dist,
        geometry_msgs::PoseStamped& goal_t)
    {
        tf::Stamped<tf::Pose> p;
        tf::poseStampedMsgToTF(goal, p);
        tf::Vector3 z(0, 0, dist);
        z = p.getBasis() * z;
        p.setOrigin(p.getOrigin() + z);

        // Also need to rotate the target pose so that the robot
        // is aligned along Z-:
        double yaw = -atan2(-z.y(), -z.x());
        p.setRotation(tf::createQuaternionFromYaw(yaw));

        // NOTE: Assumes a ground-aligned frame.
        tf::poseStampedTFToMsg(p, goal_t);
    }

    /// \brief Generate a pose facing a landmark at a specific distance on the
    ///        XY plane.
    ///
    /// The pose is generated along the line segment connecting poses A and B,
    /// A being the robot.
    /// The orientations are ignored, but the goal pose will be generated so
    /// that the X axis point toward the goal.
    /// The goal will be generated in the same frame as pose B.
    ///
    /// If the distance between A and B is too short for the wanted distance,
    /// the goal will be to stay in place, but still face the goal.
    ///
    /// \param tf       A TF listener/transformer object.
    /// \param pose_a   The current robot pose.
    /// \param pose_b   The goal pose.
    /// \param dist     The desired distance from the goal (floored at 0).
    /// \param goal_t   The resulting goal pose message.
    inline void facingPoseXY(
        tf::Transformer&                    tf,
        const geometry_msgs::PoseStamped&   pose_a,
        const geometry_msgs::PoseStamped&   pose_b,
        double                              dist,
        geometry_msgs::PoseStamped&         goal_t)
    {
        if (dist < 0.0) {
            dist = 0.0;
        }

        tf::Stamped<tf::Pose> t_a, t_b;
        tf::poseStampedMsgToTF(pose_a, t_a);
        tf::poseStampedMsgToTF(pose_b, t_b);

        // Make sure Pose A is in the same frame as B and the output.
        tf.transformPose(pose_b.header.frame_id, t_a, t_a);

        const tf::Point pos_a = t_a.getOrigin();
        const tf::Point pos_b = t_b.getOrigin();

        const tf::Vector3 ab = pos_b - pos_a;
        // Saturate the desired distance to stay in place.
        double ab_l = ab.length();
        if (ab_l < dist) {
            dist = ab_l;
        }

        // Generate the pose, using the vector between A and B to generate the
        // Yaw angle around Z on the XY plane.
        // This should orient the goal's X toward B.
        tf::Point goal_p = pos_b - dist * ab.normalized();
        tf::Quaternion goal_q = 
            tf::createQuaternionFromYaw(atan2(ab.y(), ab.x()));

        // Prepare the output message
        goal_t.header.frame_id = pose_b.header.frame_id;
        goal_t.header.stamp    = pose_b.header.stamp;
        tf::pointTFToMsg(goal_p,      goal_t.pose.position);
        tf::quaternionTFToMsg(goal_q, goal_t.pose.orientation);
    }

    /// \brief Converts a (x,y,theta) tuple into a navigation goal string.
    ///
    /// See pubNavGoal plugin for HBBA's script_engine for details.
    inline void poseToNavGoal(const std::string&    frame_id,
                              const double          x,
                              const double          y,
                              const double          t,
                              std::string&          out)
    {
        std::stringstream ss;
        ss << "{frame_id: '" << frame_id << "', ";
        ss << "x: " << x << ", ";
        ss << "y: " << y << ", ";
        ss << "t: " << t << "}";
        out = ss.str();
    }

    /// \brief Converts a (x,y,theta) tuple into a navigation goal string.
    ///
    /// See pubNavGoal plugin for HBBA's script_engine for details.
    inline void poseToNavGoal(const std::string&    frame_id,
                              const tf::Point&      pt,
                              std::string&          out)
    {
        poseToNavGoal(frame_id, pt.x(), pt.y(), pt.z(), out);
    }

    /// \brief Converts a pose stamped message into a navigation goal string.
    ///
    /// See pubNavGoal plugin for HBBA's script_engine for details.
    inline void poseStampedToNavGoal(
        const geometry_msgs::PoseStamped& msg, 
        std::string& out)
    {
        tf::Point pt(msg.pose.position.x,
                     msg.pose.position.y,
                     tf::getYaw(msg.pose.orientation));

        poseToNavGoal(msg.header.frame_id, pt, out);
    }

    /// \brief Converts a standard string into a dialog string.
    ///
    /// See pubString plugin for HBBA's script_engine for details.
    inline void stringToDialog(
        const std::string& data, 
        std::string& out)
    {
        std::stringstream ss;
        ss << "{msg: '" << data << "'}";
        out = ss.str();
    }

    /// \brief Converts a String message into a dialog string.
    ///
    /// See pubString plugin for HBBA's script_engine for details.
    inline void stringToDialog(
        const std_msgs::String& msg, 
        std::string& out)
    {
        stringToDialog(msg.data, out);
    }

    /// \brief Converts a tf::StampedTransform to a geometry_msgs::PoseStamped.
    ///
    /// The orientation is directly transfered, and the output's position
    /// corresponds to the input target frame's origin.
    inline void stampedTransformTFToPoseMsg(const tf::StampedTransform& in,
                                            geometry_msgs::PoseStamped& out)
    {
        out.header.frame_id = in.frame_id_;
        out.header.stamp    = in.stamp_;
        tf::pointTFToMsg(     in.getOrigin(),   out.pose.position);
        tf::quaternionTFToMsg(in.getRotation(), out.pose.orientation);
    }

    /// \brief Return the latest known transform between two frames.
    ///
    /// Catches TF errors, output them with ROS_ERROR and return false.
    ///
    /// \param tf     The transformer to use.
    /// \param source The source (reference, or origin) frame.
    /// \param target The target frame.
    /// \param out    Output transform.
    inline bool latestTransform(tf::Transformer&      tf,
                                const std::string&    source,
                                const std::string&    target,
                                tf::StampedTransform& out) 
    {
        try {
            tf.lookupTransform(source, target, ros::Time(), out);
        } catch (tf::TransformException e) {
            ROS_ERROR("Could not obtain transform between %s and %s, error: %s",
                      source.c_str(),
                      target.c_str(),
                      e.what());
            return false;
        }

        return true;
    }

    /// \brief Return the latest known transform between two frames.
    /// 
    /// Same as the other overload, but with a built-in conversion to a
    /// PoseStamped message type.
    inline bool latestTransform(tf::Transformer&            tf,
                                const std::string&          source,
                                const std::string&          target,
                                geometry_msgs::PoseStamped& out) 
    {
        tf::StampedTransform tmp;
        if (latestTransform(tf, source, target, tmp)) {
            stampedTransformTFToPoseMsg(tmp, out);
            return true;
        } else {
            return false;
        }
    }

    /// \brief Call the add_desire service for a single desire id.
    ///
    /// If a service client isn't provided, will use the ros::service::call(...)
    /// interface, which is less efficient if you plan on calling this often
    /// (costs an extra ROS master round trip for discovery).
    inline void addDesire(const hbba_msgs::Desire& desire,
                          ros::ServiceClient*      scl = NULL)
    {
        hbba_msgs::AddDesires::Request   req;
        hbba_msgs::AddDesires::Response  res;

        req.desires.push_back(desire);

        if (scl != NULL) {
            scl->call(req, res);
        } else {
            ros::service::call("add_desires", req, res);
        }
    }

    /// \brief Call the remove_desire service for a single desire id.
    ///
    /// Same mechanism as addDesire, see other function for details.
    inline void removeDesire(const std::string&    id,
                             ros::ServiceClient*   scl = NULL)
    {
        hbba_msgs::RemoveDesires::Request   req;
        hbba_msgs::RemoveDesires::Response  res;

        req.ids.push_back(id);

        if (scl != NULL) {
            scl->call(req, res);
        } else {
            ros::service::call("remove_desires", req, res);
        }
    }
}

#endif

