import rospy, ipfsapi
from std_msgs.msg import String
from std_srvs.srv import Empty
from chemistry_services.srv import *
from robonomics_liability.msg import Liability
from robonomics_lighthouse.msg import Result

class PublishToBlockchain:

    liability_finished = False
    path_to_file = ''

    def __init__(self):
        rospy.init_node('publish_to_blockchain_node')
        self.pub = rospy.Publisher("result", String, queue_size=10)
        self.ipfs = ipfsapi.connect("localhost", 5001)
        self.signing_result = rospy.Publisher('lighthouse/infochan/signing/result', Result, queue_size=10) 

        def cb(req):
            rospy.loginfo("publish_to_blockchain service was called with file " + req.pathtofile)
            self.publishToBlockchain(req.pathtofile)
            response = PublishToBlockchainResponse()
            response.result = self.res
            response.address = self.address
            return response
        rospy.Service('publish_to_bc', PublishToBC, cb)

        def callback(data):
            rospy.loginfo("I got the task")
            hash = self.ipfs.add(self.path_to_file)
            self.res = hash['Hash']
            rospy.loginfo("Result is " + self.res)
            self.pub.publish(self.res)

            r = Result()
            r.liability = self.address
            r.result = bytes(self.res, "UTF-8")
            rospy.loginfo("Sending the result")
            self.signing_result.publish(r)
            self.liability_finished = True
        rospy.Subscriber("/task", String, callback)

        def save_address(data):
            rospy.loginfo('New liability: {}'.format(data.address))
            self.address = data.address
        rospy.Subscriber("liability/incoming", Liability, save_address)

    def spin(self):
        rospy.spin()

    def publishToBlockchain(self, pathtofile):
        rospy.loginfo("instide publishToBlockchain('" + pathtofile + "')")
        self.liability_finished = False
        self.path_to_file = pathtofile
        rospy.wait_for_service('make_bid')
        m = rospy.ServiceProxy('make_bid', Empty)
        rospy.loginfo("Let's call make_bid service")
        m()
        
        while not self.liability_finished:
            rospy.sleep(1)

