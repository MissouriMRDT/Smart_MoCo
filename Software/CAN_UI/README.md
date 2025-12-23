# Smart Motor Controller CAN UI

## Operation

This software was tested with the [UCAN v1.0 USB to CAN adapter board](https://www.fysetc.com/products/fysetc-ucan-board) on Windows 11 with Python 3.12.10. The [gs-usb](https://python-can.readthedocs.io/en/4.0.0/interfaces/gs_usb.html) Python library is used to interface with the UCAN module. [Zadig](https://zadig.akeo.ie/) was needed to force the use of the the correct USB drivers for pyusb as instructed [here](https://python-can.readthedocs.io/en/stable/interfaces/gs_usb.html#supported-platform). Within Zadig, select the UCAN device and install libusbK. Before attempting to run any commands, use the ping button to check motor controller connectivity. Unfortunately, UCAN must be unplugged and plugged in again before each run of this program.

## Running

### Windows

1) `py -m venv venv`
2) `venv\Scripts\python -m pip install -r requirements.txt`
3) `venv\Scripts\python main.py`

### Linux

1) `python3 -m venv venv`
2) `venv/bin/pip install -r requirements.txt`
3) `venv/bin/python main.py`
