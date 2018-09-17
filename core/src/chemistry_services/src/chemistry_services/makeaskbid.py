import rospy
from robonomics_lighthouse.msg import Ask, Bid
from std_srvs.srv import Empty, EmptyResponse
from web3 import Web3, HTTPProvider

class MakeAskBid:

    model = 'QmWboFP8XeBtFMbCeMiStRY3gKFBSR5iQzkKgeNgQz3dZ5'
    # token = '0xdEA33F21294399c675C8F1D6C4a1F39b0719BCBf'    # kovan
    cost  = 1
    count = 1

    def __init__(self):
        rospy.init_node("make_ask_bid_node")

        self.web3 = Web3(HTTPProvider("http://127.0.0.1:8545"))
        self.signing_bid = rospy.Publisher('liability/infochan/signing/bid', Bid, queue_size=128)

        def callback(m):
            rospy.loginfo("about to make a bid")

            if m.model != self.model:
                return
            
            block = self.web3.eth.getBlock('latest')
            deadline = block.number + 10000 # should be enough for a day

            msg = Bid()
            msg.model = self.model
            msg.objective   = m.objective
            msg.token = m.token
            msg.cost = self.cost
            msg.lighthouseFee = 0
            msg.deadline = deadline
            rospy.loginfo("Publishing")
            self.signing_bid.publish(msg)
        rospy.Subscriber('liability/infochan/incoming/ask', Ask, callback)

    def spin(self):
        rospy.spin()

