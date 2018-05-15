import rospy
from robonomics_lighthouse.msg import Ask, Bid
from std_srvs.srv import Empty, EmptyResponse
from web3 import Web3, HTTPProvider

class MakeAskBid:

    model = 'QmWboFP8XeBtFMbNYK3Ne8Z3gKFBSR5iQzkKgeNgQz3dZ4'
    token = '0x0Ef7fCB816fd725819e071eFB48F7EACb85c1A6A'    # kovan
    cost  = 1
    count = 1

    def __init__(self):
        rospy.init_node("make_ask_bid_node")

        self.web3 = Web3(HTTPProvider("http://127.0.0.1:8545"))
        self.signing_bid = rospy.Publisher('lighthouse/infochan/signing/bid', Bid, queue_size=128)

        def callback(m):
            rospy.loginfo("about to make a bid")
            
            block = self.web3.eth.getBlock('latest')
            deadline = block.number + 10000 # should be enough for a day

            msg = Bid()
            msg.model = self.model
            msg.token = self.token
            msg.cost = self.cost
            msg.count = self.count
            msg.lighthouseFee = 0
            msg.deadline = deadline
            self.signing_bid.publish(msg)
        rospy.Subscriber('lighthouse/infochan/incoming/ask', Ask, callback)

    def spin(self):
        rospy.spin()

