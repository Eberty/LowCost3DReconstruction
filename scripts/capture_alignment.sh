#!/bin/bash

# Copy this script into a folder and execute

# ----------------------------------------------------------------------

# Verify if the argument is a valid name
if [[ ! ${1} ]]; then
	echo "Please inform a valid name as argument."
	return;
fi

# ----------------------------------------------------------------------

# Set artefact name
ARTEFACT_NAME=${1}

# Set Super4PCS command
Super4PCS_BIN=/usr/local/bin/Super4PCS

# Set directory where are the necessary executable for this pipeline
EXE_DIR=/home/eberty/Dropbox/DocumentosMestrado/Codigos/msc-research/bin

# Set directory with meshlab scripts
MESHLAB_SCRIPTS_DIR=/home/eberty/Dropbox/DocumentosMestrado/Codigos/msc-research/scripts

# ----------------------------------------------------------------------

# Set capture step
CAPTURE_STEP=1

# ----------------------------------------------------------------------

# Step 1: Capture
${EXE_DIR}/depth_capture --capture_name ${ARTEFACT_NAME} --capture_step ${CAPTURE_STEP} --sr_size 16

# Set number of captures
NUM_OF_CAPTURES=${?}

# ----------------------------------------------------------------------

# Step 2: Super resolution
${EXE_DIR}/super_resolution --capture_name ${ARTEFACT_NAME} --capture_step ${CAPTURE_STEP} --num_captures ${NUM_OF_CAPTURES} --sr_size 16
${EXE_DIR}/super_resolution --capture_name ${ARTEFACT_NAME} --top --sr_size 16
${EXE_DIR}/super_resolution --capture_name ${ARTEFACT_NAME} --bottom --sr_size 16

# ----------------------------------------------------------------------

# Step 3: Fix normals
for i in $(seq 0 "$(( 10#${NUM_OF_CAPTURES} - 1 ))"); do
	meshlab.meshlabserver -i ${ARTEFACT_NAME}_srmesh_"$(( 10#${i} * 10#${CAPTURE_STEP} ))".ply -o ${i}.ply -m vc vn -s ${MESHLAB_SCRIPTS_DIR}/normal_estimation.mlx
done

meshlab.meshlabserver -i ${ARTEFACT_NAME}_srmesh_top.ply -o top.ply -m vc vn -s ${MESHLAB_SCRIPTS_DIR}/normal_estimation.mlx
meshlab.meshlabserver -i ${ARTEFACT_NAME}_srmesh_bottom.ply -o bottom.ply -m vc vn -s ${MESHLAB_SCRIPTS_DIR}/normal_estimation.mlx

# ----------------------------------------------------------------------

# Step 4: Coarse Alignment
for i in $(seq 1 "$(( 10#${NUM_OF_CAPTURES} - 1 ))"); do
 	${Super4PCS_BIN} -i "$(( 10#${i} - 01 ))".ply ${i}.ply -r tmp.ply -t 10 -m tmp.txt
 	sed -i '1,2d' tmp.txt
 	${EXE_DIR}/transform -i ${i}.ply -o ${i}.ply -t tmp.txt
 	meshlab.meshlabserver -i ${i}.ply -o ${i}.ply -m vc vn
done

rm tmp.ply tmp.txt

# ----------------------------------------------------------------------

# Step 5: Fine alignment - TODO
# For now use meshlab ICP: Please flatten layers and save as ${ARTEFACT_NAME}.ply
meshlab 0.ply

# Top and bottom alignment

# Coarse Alignment
${EXE_DIR}/pair_align -i top.ply -t ${ARTEFACT_NAME}.ply -o top.ply --roll -90 --pitch 0 --yaw 90 --elevation 50
${EXE_DIR}/pair_align -i bottom.ply -t ${ARTEFACT_NAME}.ply -o bottom.ply--roll 90 --pitch 0 --yaw 90 --elevation -50

# Fine alignment - TODO
# For now use meshlab ICP: Please flatten layers and save as ${ARTEFACT_NAME}.ply
# meshlab ${ARTEFACT_NAME}.ply
 
# ----------------------------------------------------------------------

# Step 6: Remove outliers and scale kinect ply result file
cp ${ARTEFACT_NAME}.ply ${ARTEFACT_NAME}_backup.ply
${EXE_DIR}/outlier_removal --input ${ARTEFACT_NAME}.ply --output ${ARTEFACT_NAME}.ply --neighbors 50 --dev_mult 1.0
${EXE_DIR}/scale --input ${ARTEFACT_NAME}.ply --output ${ARTEFACT_NAME}.ply --scale 0.01
meshlab.meshlabserver -i ${ARTEFACT_NAME}.ply -o ${ARTEFACT_NAME}.ply -m vc vn -s ${MESHLAB_SCRIPTS_DIR}/normal_normalize.mlx

# ----------------------------------------------------------------------

# meshlab ${ARTEFACT_NAME}.ply
