frontend-pkg - the site with upload and get info pages
services-pkg - makeask, makebid and publishnode

requirements
--------

* libs: ipfsapi, pyqrcode, pypng
* parity --chain kovan --unlock 0x... --password pass 
* account must have XRT and ETH
* ipfs daemon --enable-pubsub-experimentt
* crontab: * */3 * * * . $HOME/chemistry-quality-control/services-pkg/install/setup.bash; rosservice call /make_ask
* approve for spender and miner

