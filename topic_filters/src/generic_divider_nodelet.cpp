#include "generic_divider.hpp"
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace topic_filters
{
    class GenericDivider: public nodelet::Nodelet
    {
    private:
        boost::scoped_ptr<GenericDividerNode> node_;

    public:
        GenericDivider() {}
        virtual void onInit()
        {
            node_.reset(new GenericDividerNode(getNodeHandle(), 
                                               getPrivateNodeHandle(),
                                               getMyArgv()));
        }

    };

}

PLUGINLIB_EXPORT_CLASS(topic_filters::GenericDivider, nodelet::Nodelet)

