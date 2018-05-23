import rospy, ipfsapi
import base58
from std_msgs.msg import String
from std_srvs.srv import Empty
from chemistry_services.srv import *
from robonomics_liability.msg import Liability

class PublishToBlockchain:

    got_file = False
    path_to_file = ''

    def __init__(self):
        rospy.init_node('publish_to_blockchain_node')
        self.pub = rospy.Publisher("result", String, queue_size=10)
        self.ipfs = ipfsapi.connect("localhost", 5001)
        self.got_file = False

        def cb(req):
            rospy.loginfo("publish_to_blockchain service was called with file " + req.pathtofile)
            self.path_to_file = req.pathtofile
            self.got_file = True
            return PublishToBCResponse()
        rospy.Service('file_for_publishing', PublishToBC, cb)

        def get_file_cb(req):
            while not hasattr(self, 'res'):
                rospy.sleep(1)

            response = PublishToBCResponse()
            response.result = self.res
            response.address = self.address
            del self.res
            self.got_file = False
            return response
        rospy.Service('get_result', PublishToBC, get_file_cb)

        def callback(data):
            rospy.loginfo("I got the task")

            while not self.got_file:
                rospy.sleep(1)

            hash = self.ipfs.add(self.path_to_file)
            self.res = hash['Hash']
            rospy.loginfo("Result is " + self.res)
            self.pub.publish(self.res)

            rospy.wait_for_service("liability/finish")
            fin = rospy.ServiceProxy("liability/finish", Empty)
            rospy.loginfo("finishing...")
            fin()
        rospy.Subscriber("/task", String, callback)

        def save_address(data):
            rospy.loginfo('New liability: {}'.format(data.address))
            self.address = data.address
        rospy.Subscriber("liability/incoming", Liability, save_address)

    def spin(self):
        rospy.spin()

