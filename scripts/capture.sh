#!/bin/bash

# Copy this script into a folder and execute

# ----------------------------------------------------------------------

# Verify if the argument is a valid name
if [[ ! ${1} ]]; then
    echo "Please inform a valid name as argument."
    return;
fi

# Set depth capture method
if [[ ! ${2} || ! ( ${2} == "1" || ${2} == "2" ) ]]; then
    echo "Please inform a valid kinect version (1 or 2)."
    return;
fi

# ----------------------------------------------------------------------

# Set directory where are the necessary executable for this pipeline
EXE_DIR=/usr/local/LowCost3DReconstruction

# ----------------------------------------------------------------------

# Set model name
OBJECT_NAME=${1}

# Set capture step
CAPTURE_STEP=1

# Set sr size
SR_SIZE=16

# Capture method
KINECT_VERSION=""
if [[ ${2} == "2" ]]; then
    KINECT_VERSION="_kv2"
fi

# ----------------------------------------------------------------------

# Step 1: Capture
${EXE_DIR}/depth_capture${KINECT_VERSION} --capture_name ${OBJECT_NAME} --capture_step ${CAPTURE_STEP} --sr_size ${SR_SIZE}

# Set number of captures
NUM_OF_CAPTURES=${?}
