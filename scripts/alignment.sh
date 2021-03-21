#!/bin/bash

# Copy this script into a folder and execute

# ----------------------------------------------------------------------

# Print help message
for ARG in "${@:1}"; do
  if [[ ${ARG} == "-h" || ${ARG} == "--help" ]]; then
    echo "Usage: "
    echo "    source ${BASH_SOURCE} <object_name> <num_of_captures> [sr <kinect_version=1|2>]"
    return;
  fi
done

# ----------------------------------------------------------------------

# Verify if the first argument is a valid name
if [[ ! ${1}  || ! -f ${1}_mesh_0.ply ]]; then
  echo "Please inform a valid name as first argument."
  return;
fi

# Verify if the second argument is a valid number
if [[ ! ${2} || ! ${2} =~ ^[+-]?[0-9]+$ ]]; then
  echo "Please inform a valid number of captures as second argument."
  return;
fi

# Set SR method
SR=""
KINECT_VERSION=""

if [[ ${3} && "${3,,}" == "sr" ]]; then
  SR="sr"

  # Set depth capture method
  if [[ ! ${4} || ! ( ${4} == "1" || ${4} == "2" ) ]]; then
    echo "Please inform a valid kinect version (1 or 2) indicating the origin of depth images."
    return;
  fi

  if [[ ${4} == "2" ]]; then
    KINECT_VERSION="_kv2"
  fi

  echo "Using SR mesh"
else
  echo "Using mesh"
fi

# ----------------------------------------------------------------------

# Set LowCost3DReconstruction directory
LOW_COST_3D_RECONSTRUCTION_DIR=/usr/local/LowCost3DReconstruction

# Set Super4PCS command
Super4PCS_BIN=/usr/local/bin/Super4PCS

# Set meshlab commands
MESHLAB="LC_ALL=C ${LOW_COST_3D_RECONSTRUCTION_DIR}/MeshLab2020.12-linux.AppImage"
MESHLABSERVER="LC_ALL=C ${LOW_COST_3D_RECONSTRUCTION_DIR}/MeshLabServer2020.12-linux.AppImage"

# Set model name
OBJECT_NAME=${1}

# Set number of captures
NUM_OF_CAPTURES=${2}

# Set capture step
CAPTURE_STEP=1

# Set sr size
SR_SIZE=16

# ----------------------------------------------------------------------

# Super resolution
if [[ ${SR} == "sr" ]]; then
  ${LOW_COST_3D_RECONSTRUCTION_DIR}/super_resolution${KINECT_VERSION} --capture_name ${OBJECT_NAME} --capture_step ${CAPTURE_STEP} --num_captures ${NUM_OF_CAPTURES} --sr_size ${SR_SIZE}
  ${LOW_COST_3D_RECONSTRUCTION_DIR}/super_resolution${KINECT_VERSION} --capture_name ${OBJECT_NAME} --top --sr_size ${SR_SIZE}
  ${LOW_COST_3D_RECONSTRUCTION_DIR}/super_resolution${KINECT_VERSION} --capture_name ${OBJECT_NAME} --bottom --sr_size ${SR_SIZE}
fi

# Scale Kinect ply files
for i in $(seq 0 "$(( 10#${NUM_OF_CAPTURES} - 1 ))"); do
  ${LOW_COST_3D_RECONSTRUCTION_DIR}/scale -i ${OBJECT_NAME}_${SR}mesh_"$(( 10#${i} * 10#${CAPTURE_STEP} ))".ply -o ${i}.ply --scale 0.01
done

${LOW_COST_3D_RECONSTRUCTION_DIR}/scale -i ${OBJECT_NAME}_${SR}mesh_top.ply -o top.ply --scale 0.01
${LOW_COST_3D_RECONSTRUCTION_DIR}/scale -i ${OBJECT_NAME}_${SR}mesh_bottom.ply -o bottom.ply --scale 0.01

# Fix normals
for i in $(seq 0 "$(( 10#${NUM_OF_CAPTURES} - 1 ))"); do
  eval ${MESHLABSERVER} -i ${i}.ply -o ${i}.ply -m vc vn -s ${LOW_COST_3D_RECONSTRUCTION_DIR}/normal_estimation.mlx 2> /dev/null
done

eval ${MESHLABSERVER} -i top.ply -o top.ply -m vc vn -s ${LOW_COST_3D_RECONSTRUCTION_DIR}/normal_estimation.mlx 2> /dev/null
eval ${MESHLABSERVER} -i bottom.ply -o bottom.ply -m vc vn -s ${LOW_COST_3D_RECONSTRUCTION_DIR}/normal_estimation.mlx 2> /dev/null

# ----------------------------------------------------------------------

# Coarse alignment
for i in $(seq 1 "$(( 10#${NUM_OF_CAPTURES} - 1 ))"); do
  ${Super4PCS_BIN} -i "$(( 10#${i} - 01 ))".ply ${i}.ply -r tmp.ply -t 15 -m tmp.txt -o 0.6 -d 0.03 -n 700
  sed -i '1,2d' tmp.txt
  for j in $(seq ${i} "$(( 10#${NUM_OF_CAPTURES} - 1 ))"); do
    ${LOW_COST_3D_RECONSTRUCTION_DIR}/transform -i ${j}.ply -o ${j}.ply -t tmp.txt
    eval ${MESHLABSERVER} -i ${j}.ply -o ${j}.ply -m vc vn &> /dev/null
  done
done

rm tmp.ply tmp.txt

# Fine alignment - TODO
echo "----- Fine alignment -----"
echo "For now use meshlab: Apply ICP align for all [number].ply, flatten layers and save as ${OBJECT_NAME}.ply"
FILES=$(ls +([0-9]).ply | sort -n)
eval ${MESHLAB} ${FILES} &> /dev/null

# Remove outliers
cp ${OBJECT_NAME}.ply ${OBJECT_NAME}_backup.ply
${LOW_COST_3D_RECONSTRUCTION_DIR}/outlier_removal -i ${OBJECT_NAME}.ply -o ${OBJECT_NAME}.ply --neighbors 50 --dev_mult 3.0
eval ${MESHLABSERVER} -i ${OBJECT_NAME}.ply -o ${OBJECT_NAME}.ply -m vc vn -s ${LOW_COST_3D_RECONSTRUCTION_DIR}/normal_normalize.mlx 2> /dev/null

# Coarse alignment (Top and bottom)
${LOW_COST_3D_RECONSTRUCTION_DIR}/centroid_align -i top.ply -t ${OBJECT_NAME}.ply -o top.ply --roll -90 --pitch 0 --yaw 90 --elevation 50
${LOW_COST_3D_RECONSTRUCTION_DIR}/centroid_align -i bottom.ply -t ${OBJECT_NAME}.ply -o bottom.ply --roll 90 --pitch 0 --yaw 90 --elevation -50

# Fine alignment (Top and bottom) - TODO
echo "----- Fine alignment -----"
echo "For now use meshlab: Apply ICP align ussing all ply files, flatten layers and save as ${OBJECT_NAME}.ply"
eval ${MESHLAB} ${OBJECT_NAME}.ply top.ply bottom.ply &> /dev/null
