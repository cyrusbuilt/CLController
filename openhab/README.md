# OpenHAB Components

This directory contains components for integrating with OpenHAB2.

## Instructions for integrating with OpenHAB2

The first thing you will need to do if you haven't already, is install an MQTT broker. For the purposes of this guide, I will assume you will be using [Mosquitto](https://mosquitto.org/) and will be installing it on the same machine as OpenHAB2.

To install mosquitto, follow the download and install instructions [here](https://mosquitto.org/download/). But assuming you are installing on an Ubuntu or Ubuntu-based system just run:

```bash
> sudo apt-add-repository ppa:mosquitto-dev/mosquitto-ppa
> sudo apt-get update
> sudo apt install mosquitto mosquitto-clients
```

NOTE: This document is a work in progress. Will be updated soon.