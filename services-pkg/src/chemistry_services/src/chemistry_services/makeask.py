import rospy
from robonomics_lighthouse.msg import Ask
from std_srvs.srv import Empty, EmptyResponse

class MakeAsk:

    def __init__(self):
        rospy.init_node("make_ask_node")
        self.signing_ask = rospy.Publisher('/lighthouse/infochan/signing/ask', Ask, queue_size=128)

        def callmakeask(req):
            self.makeask()
            return EmptyResponse()
        rospy.Service('publishToBlockchain', Empty, callmakeask)

    def spin(self):
        rospy.spin()

    def makeask(self):
        rospy.loginfo("let's make an ask")
        msg = Ask()
        msg.model = 'QmWboFP8XeBtFMbNYK3Ne8Z3gKFBSR5iQzkKgeNgQz3dZ4'
        msg.objective = 'QmUo3vvSXZPQaQWjb3cH3qQo1hc8vAUqNnqbdVABbSLb6r' # should publish to /task topic
        msg.token = '0x2b3ce4c151f7c9662fdd12e5b9c7b39b0d61e7f2'    # kovan
        msg.cost  = 10
        msg.count = 1
        msg.validator = '0x0000000000000000000000000000000000000000'
        msg.deadline = 8000000
        rospy.loginfo("about to sign ask")
        self.signing_ask.publish(msg)

