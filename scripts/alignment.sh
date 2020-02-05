#!/bin/bash

# Copy this script into a folder and execute

# ----------------------------------------------------------------------

# Verify if the first argument is a valid name
if [[ ! ${1}  || ! -f ${1}_mesh_0.ply ]]; then
	echo "Please inform a valid name as argument."
	return;
fi

# Verify if the second argument is a valid number
if [[ ! ${2} || ! ${2} =~ ^[+-]?[0-9]+$ ]]; then
	echo "Please inform a valid number of captures as argument."
	return;
fi

# ----------------------------------------------------------------------

# Set Super4PCS command
Super4PCS_BIN=/usr/local/bin/Super4PCS

# Set directory where are the necessary executable for this pipeline
EXE_DIR=/usr/local/msc-research

# Set meshlabserver command
MESHLABSERVER="LC_ALL=C meshlab.meshlabserver"

# Set directory with meshlab scripts
MESHLAB_SCRIPTS_DIR=/usr/local/msc-research
cp ${MESHLAB_SCRIPTS_DIR}/*.mlx ${PWD}

# ----------------------------------------------------------------------

# Set artefact name
ARTEFACT_NAME=${1}

# Set number of captures
NUM_OF_CAPTURES=${2}

# Set capture step
CAPTURE_STEP=1

# Set sr size
SR_SIZE=16

# ----------------------------------------------------------------------

# Step 1: Super resolution
${EXE_DIR}/super_resolution --capture_name ${ARTEFACT_NAME} --capture_step ${CAPTURE_STEP} --num_captures ${NUM_OF_CAPTURES} --sr_size ${SR_SIZE}
${EXE_DIR}/super_resolution --capture_name ${ARTEFACT_NAME} --top --sr_size ${SR_SIZE}
${EXE_DIR}/super_resolution --capture_name ${ARTEFACT_NAME} --bottom --sr_size ${SR_SIZE}

# ----------------------------------------------------------------------

# Step 2: Fix normals
for i in $(seq 0 "$(( 10#${NUM_OF_CAPTURES} - 1 ))"); do
	eval ${MESHLABSERVER} -i ${ARTEFACT_NAME}_srmesh_"$(( 10#${i} * 10#${CAPTURE_STEP} ))".ply -o ${i}.ply -m vc vn -s normal_estimation.mlx 2> /dev/null
done

eval ${MESHLABSERVER} -i ${ARTEFACT_NAME}_srmesh_top.ply -o top.ply -m vc vn -s normal_estimation.mlx 2> /dev/null
eval ${MESHLABSERVER} -i ${ARTEFACT_NAME}_srmesh_bottom.ply -o bottom.ply -m vc vn -s normal_estimation.mlx 2> /dev/null

# ----------------------------------------------------------------------

# Step 3: Model alignment

# Coarse Alignment
for i in $(seq 1 "$(( 10#${NUM_OF_CAPTURES} - 1 ))"); do
 	${Super4PCS_BIN} -i "$(( 10#${i} - 01 ))".ply ${i}.ply -r tmp.ply -t 10 -m tmp.txt
 	sed -i '1,2d' tmp.txt
 	${EXE_DIR}/transform -i ${i}.ply -o ${i}.ply -t tmp.txt
	eval ${MESHLABSERVER} -i ${i}.ply -o ${i}.ply -m vc vn 2> /dev/null
done

rm tmp.ply tmp.txt

# Fine alignment - TODO
# For now use meshlab: open all number.ply, apply ICP align, flatten layers and save as ${ARTEFACT_NAME}.ply
LC_ALL=C meshlab 0.ply 2> /dev/null

# Remove outliers
cp ${ARTEFACT_NAME}.ply ${ARTEFACT_NAME}_backup.ply
${EXE_DIR}/outlier_removal --input ${ARTEFACT_NAME}.ply --output ${ARTEFACT_NAME}.ply --neighbors 50 --dev_mult 2.0
eval ${MESHLABSERVER} -i ${ARTEFACT_NAME}.ply -o ${ARTEFACT_NAME}.ply -m vc vn -s normal_normalize.mlx 2> /dev/null

# ----------------------------------------------------------------------

# Step 4: Top and bottom alignment

# Coarse Alignment
${EXE_DIR}/pair_align -i top.ply -t ${ARTEFACT_NAME}.ply -o top.ply --roll -90 --pitch 0 --yaw 90 --elevation 50
${EXE_DIR}/pair_align -i bottom.ply -t ${ARTEFACT_NAME}.ply -o bottom.ply --roll 90 --pitch 0 --yaw 90 --elevation -50

# Fine alignment - TODO
# For now use meshlab: open top.ply and bottom.ply, apply ICP align, flatten layers and save as ${ARTEFACT_NAME}.ply
LC_ALL=C meshlab ${ARTEFACT_NAME}.ply 2> /dev/null
 
# ----------------------------------------------------------------------

# Step 5: Scale kinect ply result file
${EXE_DIR}/scale --input ${ARTEFACT_NAME}.ply --output ${ARTEFACT_NAME}.ply --scale 0.01
eval ${MESHLABSERVER} -i ${ARTEFACT_NAME}.ply -o ${ARTEFACT_NAME}.ply -m vc vn -s normal_normalize.mlx

# ----------------------------------------------------------------------

rm ${PWD}/*.mlx

# LC_ALL=C meshlab ${ARTEFACT_NAME}.ply &> /dev/null
