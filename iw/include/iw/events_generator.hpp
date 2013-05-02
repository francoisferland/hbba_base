#ifndef EVENTS_GENERATOR_HPP
#define EVENTS_GENERATOR_HPP

#include "exploitation_matcher.hpp"
#include <hbba_msgs/DesiresSet.h>
#include <hbba_msgs/Intention.h>
#include <hbba_msgs/Event.h>
#include <hbba_msgs/CreateExploitationMatcher.h>
#include <std_msgs/String.h>
#include <ros/ros.h>
#include <map>

namespace iw
{
    /// \brief A module that generates messages from events happening in the
    /// Intention Workspace.
    ///
    /// Events are associated with desires.
    /// Desires are monitored for:
    ///
    ///  - When they appear or disappear from the active desires set;
    ///  - When they appear or disappear from the selected intention;
    ///  - When their exploitation starts or stops.
    ///
    /// Parameters:
    ///  - exp_timeout: Time used for exploitation deactivation detection.
    ///    If a desire has not been exploited for this duration, an 
    ///    EXP_OFF event is generated.
    ///    EXP_OFF events are generated only if there has been a previous
    ///    EXP_ON event.
    ///    Default: 0.5 s.
    ///
    /// Input topics:
    ///  - desires_set: The current active desires in the IW.
    ///  - intention: The active strategies selected by the solver.
    ///  - exploitation_match: Exploited desires as detected by
    ///    exploitation_matcher(s).
    ///
    /// Output topics:
    ///  - events: Messages representing a single event (hbba_msgs/Event).
    ///
    /// See also ExploitationMatcher for other input/output topics, and
    /// CreateExploitationMatcher's interface for sub-namespace details.
    class EventsGenerator
    {
    public:
        /// \brief Constructor.
        ///
        /// \param n Node handle to use for topics and services.
        /// \param np Node handle to use for parameters.
        EventsGenerator(ros::NodeHandle& n, ros::NodeHandle& np);

        ~EventsGenerator();

    private:
        void desiresCB(const hbba_msgs::DesiresSet::ConstPtr& msg);
        void intentionCB(const hbba_msgs::Intention::ConstPtr& msg);
        bool cemCB(
            hbba_msgs::CreateExploitationMatcher::Request& req,
            hbba_msgs::CreateExploitationMatcher::Response& res);

        void exploitationCB(const std::string& id);

        void detectExpOff();
        void event(const std::string& id, const unsigned char type);

        ros::Subscriber sub_desires_;
        ros::Subscriber sub_intention_;
        ros::ServiceServer srv_cem_;
        ros::Publisher pub_events_;

        ros::Duration exp_timeout_;

        enum Flags
        {
            FLAG_NONE   = 0,
            FLAG_INT    = 1,
            FLAG_EXP    = 2
        };
        struct DesireData
        {
            int flags;
            ros::Time last_exp_;
        };
        typedef std::map<std::string, DesireData> DesMap;
        DesMap map_;

        std::list<ExploitationMatcher*> matchers_;

    };

}

#endif

