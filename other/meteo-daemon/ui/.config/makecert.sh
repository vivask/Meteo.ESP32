#!/bin/bash

#Create the CA
certstrap init --common-name "MeteoCA"

#Create the certificates for the client and servers
certstrap request-cert --domain  "client"
certstrap request-cert --domain  "192.168.1.9"

#Sign the certificates for the client and servers
certstrap sign client --CA MeteoCA
certstrap sign 192.168.1.9 --CA MeteoCA  
