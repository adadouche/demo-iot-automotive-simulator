# breeze


## Description


## Installation
PyQt5
```
pip3 install --user pyqt5  
sudo apt-get install python3-pyqt5  
sudo apt-get install pyqt5-dev-tools
sudo apt-get install qttools5-dev-tools
```

FleetWise BOTO3 SDK

https://docs.aws.amazon.com/iot-fleetwise/latest/developerguide/preview-sdk-cli.html

## Usage

1. Create campaigns first using the CLI or console.
2. Update the `config.json` file as per campaigns you created in step 1.
```
jq --arg var "$AWS_REGION" '.region = $var' setup_config.json > temp.json && mv temp.json setup_config.json
```
3. Launch application. This will load the config.json and display the campaign names accordingly.

`python3 breeze.py`


