#include <iw_tools/common.hpp>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <ros/ros.h>

using namespace iw_tools;

namespace 
{
    class FacePoseXYNode
    {
    private:
        double                  dist_;

        tf::TransformListener   tf_;
        ros::Subscriber         sub_pose_;
        ros::Publisher          pub_pose_;
    public:
        FacePoseXYNode(ros::NodeHandle& n, const ros::NodeHandle& np)
        {
            np.param("dist", dist_, 1.0);

            sub_pose_ = n.subscribe("pose_in",
                                    10,
                                    &FacePoseXYNode::subPose,
                                    this);
            pub_pose_ = n.advertise<geometry_msgs::PoseStamped>("pose_out", 10);
        }

        void subPose(const geometry_msgs::PoseStamped& pose)
        {
            geometry_msgs::PoseStamped pose_in;

            pose_in.header = pose.header;
            pose_in.pose.position.x = 0.0;
            pose_in.pose.position.y = 0.0;
            pose_in.pose.position.z = 0.0;
            pose_in.pose.orientation.x = 0.0;
            pose_in.pose.orientation.y = 0.0;
            pose_in.pose.orientation.z = 0.0;
            pose_in.pose.orientation.w = 1.0;

            geometry_msgs::PoseStamped pose_out;

            facingPoseXY(tf_, pose_in, pose, dist_, pose_out);
            pub_pose_.publish(pose_out);
        }
    };
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "face_pose_xy");

    ros::NodeHandle n, np("~");
    FacePoseXYNode node(n, np);

    ros::spin();
}

