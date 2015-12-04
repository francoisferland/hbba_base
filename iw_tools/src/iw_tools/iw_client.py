# IWClient class for easy interaction with the Intention Workspace
# Expects a running ROS environment (rospy.init(...))

import json
import rospy
import numbers

from hbba_msgs.msg import Desire, Event
from hbba_msgs.srv import AddDesires, RemoveDesires

def generate_id(prefix="iwc_des"):
    if (not hasattr(generate_id, "counter")):
        generate_id.counter = 0
    generate_id.counter += 1

    """ Generate a desire id with an increasing number """
    return (prefix + "_" + str(generate_id.counter))

class IWClient:
    def __init__(self):
        srv_add = "/hbba/add_desires"
        srv_rem = "/hbba/remove_desires"
        try:
            rospy.wait_for_service(srv_add, 5.0)
            rospy.wait_for_service(srv_rem, 5.0)
        except Exception as e:
            raise Exception("Could not reach IW services: " + str(e))

        self.scl_add = rospy.ServiceProxy(srv_add, AddDesires)
        self.scl_rem = rospy.ServiceProxy(srv_rem, RemoveDesires)

        self.sub_evt = rospy.Subscriber("/hbba/events", Event, self.event_cb)

        self.auto_remove = []

    def add_raw_desire(self, d):
        """ Add a single desire (no changes applieD)"""
        self.scl_add([d])

    def add_desire(self,
                   d_type, 
                   intensity = 1.0, 
                   utility = 1.0,
                   params = {},
                   security = False,
                   d_id="",
                   auto_rem=False): 
        """ Add a desire from the given arguments (with some defaults, except
        for type (d_type). Returns the generated id.

            d_type:    Type of desire (mandatory), string
            intensity: Desire intensity (default: 1.0), int or float
            utility:   Desire utility (default: 1.0), int or float
            params:    Dictionary of desire parameters (see type for details)
            security:  If it's security-related (always fulfilled, default: 
                       False), bool
            d_id:      Desire ID (default: auto-generated0, string
            auto_rem:  Automatic removal of the desire after accomplishment
                       (ACC_ON event, default: False), bool 
        
        """

        assert(isinstance(d_type,    str)),          "Desire type (d_type) is not a string"
        assert(isinstance(d_id,      str)),          "Desire id (d_id) is not a string"
        assert(isinstance(intensity, numbers.Real)), "Desire intensity is not a float"
        assert(isinstance(utility,   numbers.Real)), "Desire utility is not a float"
        assert(isinstance(params,    dict)),         "Desire params is not a dict"
        assert(isinstance(security,  bool)),         "Desire security is not a bool"

        if (d_id == ""):
            d_id = generate_id("iwc_" + d_type)

        d = Desire()
        d.id        = d_id
        d.type      = d_type
        d.intensity = intensity
        d.utility   = utility
        d.params    = json.dumps(params)
        d.security  = security

        self.add_raw_desire(d)

        if (auto_rem):
            self.auto_remove.append(d_id)

        return d_id

    def rem_desire(self, desire_id):
        """ Remove a desire based on its ID. """
        assert(type(desire_id) is str), "Desire id is not a string"

        self.scl_rem([desire_id])

        if (desire_id in self.auto_remove):
            self.auto_remove.remove(desire_id)

    def event_cb(self, msg):
        """ Callback for HBBA events, used to detect fulfilled desires """

        if (msg.type == Event.ACC_ON) and (msg.desire_type == "GoTo"):
            # Check if it's in the auto-delete list
            # If so, delete.
            if msg.desire in self.auto_remove:
                self.rem_desire(msg.desire) # Also remove from list.


if __name__ == "__main__":
    # Simple test script to make sure a link with the IW is available.

    rospy.init_node("test_iw_client")

    print("Testing access to IW services...")
    try:
        iwc = IWClient()
    except Exception as e:
        print("Something went wrong: " + str(e))
        exit(-1)

    print("IW services ok.")
    


