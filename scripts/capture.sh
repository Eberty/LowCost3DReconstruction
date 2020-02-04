#!/bin/bash

# Copy this script into a folder and execute

# ----------------------------------------------------------------------

# Verify if the argument is a valid name
if [[ ! ${1} ]]; then
	echo "Please inform a valid name as argument."
	return;
fi

# ----------------------------------------------------------------------

# Set directory where are the necessary executable for this pipeline
EXE_DIR=/usr/local/msc-research

# ----------------------------------------------------------------------

# Set artefact name
ARTEFACT_NAME=${1}

# Set capture step
CAPTURE_STEP=1

# Set sr size
SR_SIZE=16

# ----------------------------------------------------------------------

# Step 1: Capture
${EXE_DIR}/depth_capture --capture_name ${ARTEFACT_NAME} --capture_step ${CAPTURE_STEP} --sr_size ${SR_SIZE}

# Set number of captures
NUM_OF_CAPTURES=${?}
