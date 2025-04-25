# socketcan-setup

A systemd service to bring up the SocketCAN `can0` channel automatically when the
USB to CAN adapter is plugged in.

For installation it is best to clone the repo first on the Carla EC2 and then perform this commands:

Clone repo:
```bash
git clone https://github.com/aws4embeddedlinux/demo-iot-automotive-simulator.git
```

Install:
```bash
cd demo-iot-automotive-simulator/socket-can-setup
sudo cp setup-socketcan.sh /usr/bin
sudo cp setup-socketcan.service /lib/systemd/system
sudo systemctl start setup-socketcan
sudo systemctl enable setup-socketcan
```

Check the status:
```bash
sudo journalctl -fu setup-socketcan
```