# IWClient class for easy interaction with the Intention Workspace
# Expects a running ROS environment (rospy.init(...))

import rospy

from hbba_msgs.srv import AddDesires, RemoveDesires

class IWClient:
    def __init__(self):
        srv_add = "hbba/add_desires"
        srv_rem = "hbba/rem_desires"
        try:
            rospy.wait_for_service(srv_add, 5.0)
            rospy.wait_for_service(srv_del, 5.0)
        except:
            raise Exception("Could not reach IW services - timeout")

        self.scl_add = rospy.ServiceProxy(srv_add,    AddDesires)
        self.scl_rem = rospy.ServiceProxy(srv_rem, RemoveDesires)

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
    


