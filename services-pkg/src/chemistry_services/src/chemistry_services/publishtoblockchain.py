import rospy, ipfsapi
from std_msgs.msg import String
from std_srvs.srv import Empty
from chemistry_services.srv import PublishToBlockchainRequest, PublishToBlockchainResponse
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
            rospy.loginfo("publish_to_blockchain service was called")
            self.publishToBlockchain(req.pathtofile)
            response = PublishToBlockchainResponse()
            response.result = self.res
            response.address = self.address
            return response
        rospy.Service('publish_to_bc', PublishToBlockchainRequest, cb)

        def callback(data):
            rospy.loginfo("I got the task")
            hash = self.ipfs.add(self.path_to_file)
            self.res = hash['Hash']
            rospy.loginfo("Result is " + self.res)
            self.pub.publish(self.res)

            r = Result()
            r.liability = self.address
            r.result = bytes(self.res, "UTF-8")
            self.signing_result.publish(r)
            self.liability_finished = True
        rospy.Subscriber("/task", String, callback)

        def save_address(data):
            rospy.loginfo('New liability: {}'.format(msg))
            self.addrees = data.address
        rospy.Subscriber("liability/incoming", Liability, save_address)

    def spin(self):
        rospy.spin()

    def publishToBlockchain(self, pathtofile):
        self.liability_finished = False
        self.path_to_file = pathtofile
        rospy.wait_for_service('/make_ask_bid/make_bid')
        m = rospy.ServiceProxy('/make_ask_bid/make_bid', Empty)
        m()
        
        while not self.liability_finished:
            rospy.sleep(1)

