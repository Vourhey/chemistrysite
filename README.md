# A Quality Control System for Blockator.ru 

The agent issues a digital passport for a batch of production

The Dapp is [here](https://dapp.airalab.org/#/blockator/)

For the list of topics see [robonomics/model_blockator.json](robonomics/model_blockator.json)

## Build

```
nix build -f release.nix
```

## Launch

```
roslaunch blockator_agent agent.launch  login:=LOGIN email_from:=[FROM] email_password:=PASSWORD \
    pinata_api_key:=[API_KEY] pinata_secret_api_key:=[SECRET_API_KEY] \
    sentry:=[SENTRY]
```

