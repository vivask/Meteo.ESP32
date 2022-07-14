#!/bin/bash

sudo groupadd meteo

sudo useradd -g meteo -G meteo -s /sbin/nologin meteo

mkdir /var/log/meteo-daemon
mkdir /var/lib/meteo-daemon

chown -R meteo:meteo /var/log/meteo-daemon
chown -R meteo:meteo /var/lib/meteo-daemon