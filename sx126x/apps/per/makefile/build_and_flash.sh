#!/bin/bash
set -e
echo "Cleaning..."
make clean_all > trace.txt
echo "Building transmitter and receiver images"
make >> trace.txt
echo "Flashing Receiver on NODE_L476RG"
cp out/per_receiver.bin /media/$USER/NODE_L476RG
echo "Flashing Transmitter on NODE_L476RG1"
cp out/per_transmitter.bin /media/$USER/NODE_L476RG1