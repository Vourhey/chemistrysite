# A Quality Control System for NanoDoctor.pro
## Core
Contains core services. To run:
```
cd core
catkin_make install
source install/setup.bash
roslaunch chemistry_services services.launch
```
### Requirements:
* [robonomics_comm](https://github.com/airalab/robonomics_comm)
* parity --chain kovan --unlock 0x... --password pass
* ipfs daemon --enable-pubsub-experiment
* roslaunch robonomics_lighthouse lighthouse.launch
* roslaunch robonomics_liability liability.launch

## Web

```
cd web
nix-shell
virtualenv env
source env/bin/activate
python manage.py runserver
```

### Requirements

* Django
* virtualenv
* libs: ipfsapi, pyqrcode, pypng
* account must have XRT and ETH
* approve for spender and miner

