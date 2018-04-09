frontend-pkg - the site with upload and get info pages
services-pkg - makeask, makebid and publishnode

requirements
--------

* parity --chain kovan --unlock 0x... --password pass 
* account must have XRT and ETH
* ipfs daemon --enable-pubsub-experiment
* ipfs pubsub sub aira_market
* 0 1 * * * . $HOME/chemistry-quality-control/services-pkg/install/setup.bash; rosservice call publishToBlockchain
