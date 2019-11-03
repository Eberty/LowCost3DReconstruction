#!/bin/bash

# Copy this script into a folder and execute

# ----------------------------------------------------------------------

# Verify if the argument is a valid name
if [[ ! "$1" ]]; then
	echo "Please inform a valid name as argument."
	return;
fi

# ----------------------------------------------------------------------

# Set artefact name
ARTEFACT_NAME=$1

# Set Super4PCS command
Super4PCS_BIN=/home/eberty/Super4PCS/build/install/bin/Super4PCS

# Set directory where are the necessary executable for this pipeline
EXE_DIR=/home/eberty/Dropbox/DocumentosMestrado/Codigos/msc-research/bin

# ----------------------------------------------------------------------

# Set capture step
CAPTURE_STEP=1

# ----------------------------------------------------------------------

# Step 1: Capture
$EXE_DIR/depth_capture --capture_name ${ARTEFACT_NAME} --capture_step ${CAPTURE_STEP} --sr_size 16

# Set number of captures
NUM_OF_CAPTURES=$?

# ----------------------------------------------------------------------

# Step 2: Super resolution
$EXE_DIR/super_resolution --capture_name ${ARTEFACT_NAME} --capture_step ${CAPTURE_STEP} --num_captures ${NUM_OF_CAPTURES} --sr_size 16

# ----------------------------------------------------------------------

# Step 3: Fix normals [optional]
for i in $(seq 0 "$(( 10#${NUM_OF_CAPTURES} - 1 ))"); do
	$EXE_DIR/normal_estimation --neighbors 50 --input "$ARTEFACT_NAME"_srmesh_"$(( 10#$i * 10#${CAPTURE_STEP} ))".ply --output "$i".ply
	meshlabserver -i "$i".ply -o "$i".ply -s meshlab_script_normal.mlx -om vc vn
done

# ----------------------------------------------------------------------

# Step 4: Coarse Alignment
for i in $(seq 1 "$(( 10#${NUM_OF_CAPTURES} - 1 ))"); do
	$Super4PCS_BIN -i "$(( 10#$i - 01 ))".ply "$i".ply -r "$i".ply -t 10
done

# ----------------------------------------------------------------------

# Step 5: Fine alignment - TODO
# LC_ALL=C meshlab
# For now use meshlab ICP (Please save as ${ARTEFACT_NAME}.ply)
 
# ----------------------------------------------------------------------

# Step 6: Remove unnecessary points that no belong to object [optional]
$EXE_DIR/outlier_removal --input ${ARTEFACT_NAME}.ply --output ${ARTEFACT_NAME}.ply --neighbors 50 --dev_mult 1.0

# ----------------------------------------------------------------------

# LC_ALL=C meshlab ${ARTEFACT_NAME}.ply
