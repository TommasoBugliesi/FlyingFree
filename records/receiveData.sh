#!/bin/bash

YEAR=$(echo $(date +%Y))
MONTH=$(echo $(date +%m))
DAY=$(echo $(date +%d))

FOLDER=${YEAR}"-"${MONTH}"-"${DAY}
FULL_PATH=/media/tommaso-bugliesi/Volume/Coding/PlatformIO/Projects/FlightController-ESP32/records/$FOLDER
mkdir $FULL_PATH

scp tommaso-bugliesi@192.168.0.40:/home/tommaso-bugliesi/Documents/DataLogging/records/$FOLDER/* $FULL_PATH
