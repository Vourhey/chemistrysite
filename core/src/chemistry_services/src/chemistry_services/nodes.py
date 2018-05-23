from . import makeaskbid
from . import publishtoblockchain

def makeask_node():
    makeaskbid.MakeAskBid().spin()

def publish_node():
    publishtoblockchain.PublishToBlockchain().spin()


