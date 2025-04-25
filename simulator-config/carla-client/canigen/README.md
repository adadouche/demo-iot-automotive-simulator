canigen (CAN Interactive Generator)
====================================

- Generates SocketCAN messages interactively according to a DBC file
- Can mock an ECU's OBD server responding to OBD PID requests
- Let's you interactively set CAN signal values and OBD PIDs
- Has autocompletion

## Prerequisites
For OBD support the [can-isotp kernel]
kernel module must be installed and Python 3.7+ is required. [pyenv](https://github.com/pyenv/pyenv)
is recommended for managing multiple Python versions.

    pip3 install cantools prompt_toolkit python-can can-isotp

## Quick Start

    python3 canigen.py -i vcan0 -d chevy.dbc -o obd_config_chevy.json -v vals_chevy_default.json 

- Set a signal value:

       set sig Engine_Torque 123

- Set an OBD PID value:

       set pid ENGINE_SPEED 1000

- Set an OBD DTC as failed:

       set dtc 0123 1

- Save values to a JSON file:

       save vals_chevy_default.json

- Load values from a JSON file:

       load vals_chevy_default.json

## Generate a `canplayer` file and play it back on the target
1. Generate the log, note interface is `can0`:

       python3 canigen.py -i can0 -d chevy.dbc -f dump.log

2. Playback the log on the S32G, note interface was configured as `can0` above:

       canplayer -I dump.log
