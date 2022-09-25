
## list serial ports

`docker run --rm -ti --device=/dev/ttyUSB0:/dev/ttyUSB0 192.168.178.30:5000/python3-serial python -m serial.tools.list_ports -v`

## run miniterm

`docker run --rm -ti --device=/dev/ttyUSB0:/dev/ttyUSB0 192.168.178.30:5000/python3-serial python -m serial.tools.miniterm`

## run custom script

`docker run --rm -ti --device=/dev/ttyUSB0:/dev/ttyUSB0 -v /home/henry/dev:/host/dev 192.168.178.30:5000/python3-serial python /host/dev/hichi-bridge.py`

## run in background

`docker run -d --device=/dev/ttyUSB0:/dev/ttyUSB0 -v /home/henry/dev:/host/dev 192.168.178.30:5000/python3-serial python /host/dev/hichi-bridge.py`