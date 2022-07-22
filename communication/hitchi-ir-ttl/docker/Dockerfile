FROM python:3.10.5-alpine3.16

# make sure we use the latest versions
RUN apk upgrade --update \
#
### useful tools
    && apk add --virtual .helper \
            bash \
            wget \
            curl \
            tar \
            nano \
#
### python libraries
    && python3 -m pip install --upgrade pip \
    && python3 -m pip install -U pyserial paho-mqtt numpy psycopg[binary] \
#
### clean up files
#    && apk del .build-deps .build-deps-edge .build-deps-testing .build-deps-community \
    && rm -rf /tmp/* /var/cache/apk/* /usr/src/* \