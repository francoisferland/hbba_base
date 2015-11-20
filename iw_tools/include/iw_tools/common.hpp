#ifndef IW_TOOLS_COMMON_HPP
#define IW_TOOLS_COMMON_HPP

/// \file common.hpp A set of common functions for generating HBBA desires.

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

    /// \brief Converts a PoseStamped message into a navigation goal string.
    ///
    /// See pubNavGoal plugin for HBBA's script_engine for details.
    inline void poseStampedToNavGoal(
        const geometry_msgs::PoseStamped& msg, 
        std::string& out)
    {
        std::stringstream ss;
        ss << "{frame_id: '" << msg.header.frame_id << "', ";
        ss << "x: " << msg.pose.position.x << ", ";
        ss << "y: " << msg.pose.position.y << ", ";
        ss << "t: " << tf::getYaw(msg.pose.orientation) << "}";
        out = ss.str();
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
}

#endif

