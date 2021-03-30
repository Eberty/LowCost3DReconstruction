#!/bin/bash

# Copy this script into a folder with an 'images' (SfM) subfolder

# ----------------------------------------------------------------------

# Print help message
for ARG in "${@:1}"; do
  if [[ ${ARG} == "-h" || ${ARG} == "--help" ]]; then
    echo "Usage: "
    echo "    source ${BASH_SOURCE} <object_name.ply>"
    return;
  fi
done

# ----------------------------------------------------------------------

# Set sfm directory
SFM_DIR=${PWD}/images

# Verify directory existence
if [[ ! -d ${SFM_DIR} || ! "$(ls -A ${SFM_DIR})" ]]; then
  echo "Directory ${SFM_DIR} not found or empty."
  return;
fi

# Verify if the argument is a valid file
if [[ ! -f ${1} || ${1: -4} != ".ply" ]]; then
  echo "Please inform a valid mesh file as argument."
  return;
fi

# ----------------------------------------------------------------------

# Set LowCost3DReconstruction directory
LOW_COST_3D_RECONSTRUCTION_DIR=/usr/local/LowCost3DReconstruction

# Set openMVS directory
OPENMVS_DIR=/usr/local/bin/OpenMVS

# Set Super4PCS command
Super4PCS_BIN=/usr/local/bin/Super4PCS

# Set meshlab commands
MESHLAB="LC_ALL=C ${LOW_COST_3D_RECONSTRUCTION_DIR}/MeshLab2020.12-linux.AppImage"
MESHLABSERVER="LC_ALL=C ${LOW_COST_3D_RECONSTRUCTION_DIR}/MeshLabServer2020.12-linux.AppImage"

# Set model name (remove extention)
OBJECT_NAME=${1%.*}

# Mesh empty colors
ORANGE=16744231
WHITE=16777215
BLACK=0

# ----------------------------------------------------------------------

# Get main alignment transformation
${Super4PCS_BIN} -i ${SFM_DIR}/model_dense_outlier_removal.ply ${OBJECT_NAME}.ply -r tmp.ply -t 15 -m tmp.txt -o 0.6 -d 0.03 -n 700
sed -i '1,2d' tmp.txt
${LOW_COST_3D_RECONSTRUCTION_DIR}/transform -i ${OBJECT_NAME}.ply -o ${OBJECT_NAME}_transformed.ply -t tmp.txt
${LOW_COST_3D_RECONSTRUCTION_DIR}/cloud_downsampling -i ${OBJECT_NAME}_transformed.ply -o ${OBJECT_NAME}_transformed.ply --leaf_size 0.01
eval ${MESHLABSERVER} -i ${OBJECT_NAME}_transformed.ply -o ${OBJECT_NAME}_transformed.ply -m vc vn &> /dev/null

# Also transform individual files
FILES="$(ls +([0-9]).ply | sort -n) top.ply bottom.ply"
for FILE in ${FILES}; do
  ${LOW_COST_3D_RECONSTRUCTION_DIR}/transform -i ${FILE} -o ${FILE} -t tmp.txt
  eval ${MESHLABSERVER} -i ${FILE} -o ${FILE} -m vc vn &> /dev/null
done

rm tmp.ply tmp.txt

# Fine alignment - TODO
echo "----- Fine alignment -----"
echo "Opening meshlab with SFM model and ${OBJECT_NAME}_transformed.ply."
echo "Please, apply ICP align, fix matrix of tranformed file and save as ${OBJECT_NAME}_transformed.ply"
eval ${MESHLAB} ${SFM_DIR}/model_dense_outlier_removal.ply ${OBJECT_NAME}_transformed.ply &> /dev/null

# ----------------------------------------------------------------------

# Create 3D model from depth sensor data
eval ${MESHLABSERVER} -i ${OBJECT_NAME}_transformed.ply -o ${SFM_DIR}/kinect_poisson.ply -m vc vq -s ${LOW_COST_3D_RECONSTRUCTION_DIR}/mesh_reconstruction.mlx 2> /dev/null
${OPENMVS_DIR}/ReconstructMesh --input-file ${SFM_DIR}/model.mvs --output-file ${SFM_DIR}/kinect_mesh.mvs --mesh-file ${SFM_DIR}/kinect_poisson.ply --smooth 0 --working-folder ${SFM_DIR}
${OPENMVS_DIR}/TextureMesh --input-file ${SFM_DIR}/kinect_mesh.mvs --patch-packing-heuristic 0 --cost-smoothness-ratio 1 --empty-color ${BLACK} --export-type ply --close-holes 50 --resolution-level 1 --working-folder ${SFM_DIR}

# ----------------------------------------------------------------------

# Create 3D model from SFM + MVS output
eval ${MESHLABSERVER} -i ${SFM_DIR}/model_dense_outlier_removal.ply -o ${SFM_DIR}/sfm_poisson.ply -m vc vq -s ${LOW_COST_3D_RECONSTRUCTION_DIR}/mesh_reconstruction.mlx 2> /dev/null
${OPENMVS_DIR}/ReconstructMesh --input-file ${SFM_DIR}/model.mvs --output-file ${SFM_DIR}/sfm_mesh.mvs --mesh-file ${SFM_DIR}/sfm_poisson.ply --smooth 0 --working-folder ${SFM_DIR}
${OPENMVS_DIR}/TextureMesh --input-file ${SFM_DIR}/sfm_mesh.mvs --patch-packing-heuristic 0 --cost-smoothness-ratio 1 --empty-color ${BLACK} --export-type ply --close-holes 50 --resolution-level 1 --working-folder ${SFM_DIR}

# ----------------------------------------------------------------------

# Create 3D model from hybrid point cloud
${LOW_COST_3D_RECONSTRUCTION_DIR}/accumulate_clouds --all -i ${OBJECT_NAME}_transformed.ply -t ${SFM_DIR}/model_dense_outlier_removal.ply -o ${SFM_DIR}/hybrid_model.ply
eval ${MESHLABSERVER} -i ${SFM_DIR}/hybrid_model.ply -o ${SFM_DIR}/hybrid_poisson.ply -m vc vq -s ${LOW_COST_3D_RECONSTRUCTION_DIR}/mesh_reconstruction.mlx 2> /dev/null
${OPENMVS_DIR}/ReconstructMesh --input-file ${SFM_DIR}/model.mvs --output-file ${SFM_DIR}/hybrid_mesh.mvs --mesh-file ${SFM_DIR}/hybrid_poisson.ply --smooth 0 --working-folder ${SFM_DIR}
${OPENMVS_DIR}/TextureMesh --input-file ${SFM_DIR}/hybrid_mesh.mvs --patch-packing-heuristic 0 --cost-smoothness-ratio 1 --empty-color ${BLACK} --export-type ply --close-holes 50 --resolution-level 1 --working-folder ${SFM_DIR}

rm ${SFM_DIR}/*.log

