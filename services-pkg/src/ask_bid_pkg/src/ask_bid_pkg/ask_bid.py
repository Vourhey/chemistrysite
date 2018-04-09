import rospy
import time
from robonomics_liability.msg import Liability
from robonomics_market.srv import AsksGenerator, BidsGenerator
from robonomics_market.msg import Ask, Bid
from ask_bid_pkg.srv import *
from std_srvs.srv import Empty
from std_msgs.msg import String

class AskBid:
    finished = False

    def __init__(self):
        rospy.init_node("ask_bid_node")
        self.signing_bid = rospy.Publisher('/market/signing/bid', Bid, queue_size=128)
        self.signing_ask = rospy.Publisher('/market/signing/ask', Ask, queue_size=128)
        self.pub = rospy.Publisher("sensor_data_hash", String, queue_size=10)

        def save_objective(data):
            self.objective = data.objective
            self.address = data.address
            rospy.loginfo("got new objective " + self.objective)
        rospy.Subscriber("/liability/incoming", Liability, save_objective)

        def task(data):
            service_name = '/liability/' + self.objective + '/finish'

            try:
                liability_service = rospy.ServiceProxy(service_name, Empty)
            except rospy.ServiceException as e:
                print ("Service call failed: ")

            rospy.loginfo("about to publish ipfs hash to sensor_data_hash topic")
            self.pub.publish(self.ipfsHash)
            resp1 = liability_service()
            self.finished = True
        rospy.Subscriber("/task", String, task)

        def publishToBlockchain(req):
            rospy.loginfo("publishToBlockchain was called")
            try:
                self.ipfsHash = req.ipfsHash
                self.makebid()
                time.sleep(3)
                self.makeask()
                
                while not finished:
                    time.sleep(1)
                
                rospy.loginfo("exiting service")
                return PublishToBlockchainResponse(self.address)
            except Exception as e:
                return PublishToBlockchainResponse("Error: {}".format(e))
        rospy.Service('publishToBlockchain', PublishToBlockchain, publishToBlockchain)

    def spin(self):
        rospy.spin()

    def makebid(self):
        rospy.loginfo("let's make a bid")
        msg = Bid()
        msg.model = 'QmWboFP8XeBtFMbNYK3Ne8Z3gKFBSR5iQzkKgeNgQz3dZ4'
        msg.fee   = 5
        msg.cost  = 10
        msg.count = 1
        rospy.loginfo("about to sign bid")
        self.signing_bid.publish(msg)

    def makeask(self):
        rospy.loginfo("let's make an ask")
        msg = Ask()
        msg.model = 'QmWboFP8XeBtFMbNYK3Ne8Z3gKFBSR5iQzkKgeNgQz3dZ4'
        msg.objective = 'QmcFA8HCYofYT4jb8J3samFkUdcAXjqMVLfy63FoMRUKux' # should publish to /task topic
        msg.fee   = 5
        msg.cost  = 10
        msg.count = 1
        rospy.loginfo("about to sign ask")
        self.signing_ask.publish(msg)

