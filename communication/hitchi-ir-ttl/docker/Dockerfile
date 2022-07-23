FROM python:3.10.5-alpine3.16

# see https://techdirectarchive.com/2021/09/19/how-to-create-and-deploy-a-local-registry-server-with-docker-image/
# docker run --rm -ti -p 5000:5000 registry:2
# docker build -t localhost:5000/python3-serial .
# docker push localhost:5000/python3-serial
# docker pull 192.168.178.30:5000/python3-serial
# docker run -d --device=/dev/ttyUSB0:/dev/ttyUSB0 -v /home/henry/dev:/host/dev 192.168.178.30:5000/python3-serial python /host/dev/hichi-bridge.py

# make sure we use the latest versions
RUN apk upgrade --update \
#
### tools
    && apk add \
            bash \
            wget \
            curl \
            tar \
            nano \
#
### python libraries
    && python3 -m pip install --upgrade pip \
    && python3 -m pip install -U pyserial paho-mqtt psycopg[binary] \
#
### clean up files
#    && apk del .build-deps .build-deps-edge .build-deps-testing .build-deps-community \
    && rm -rf /tmp/* /var/cache/apk/* /usr/src/* \