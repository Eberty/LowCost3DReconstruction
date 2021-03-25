#!/bin/bash

# Copy this script into a folder with an 'images' (SfM) subfolder

# ----------------------------------------------------------------------

# Print help message
for ARG in "${@:1}"; do
  if [[ ${ARG} == "-h" || ${ARG} == "--help" ]]; then
    echo "Usage: "
    echo "    source ${BASH_SOURCE} <object_name.ply> [dense [hybrid]]"
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

# Set mesh file name (remove extention)
FILE_NAME=${1%.*}

# ----------------------------------------------------------------------

# Alignment -> Accurately register two point clouds created from different image spectrums
if [[ ${2} && "${2,,}" == "dense" ]]; then
  cp ${SFM_DIR}/model_dense_outlier_removal.ply sfm_model.ply
else
  cp ${SFM_DIR}/model_outlier_removal.ply sfm_model.ply
fi

# Get main transformation
${Super4PCS_BIN} -i sfm_model.ply ${FILE_NAME}.ply -r tmp.ply -t 15 -m tmp.txt -o 0.6 -d 0.03 -n 700
sed -i '1,2d' tmp.txt
${LOW_COST_3D_RECONSTRUCTION_DIR}/transform -i ${FILE_NAME}.ply -o ${FILE_NAME}_transformed.ply -t tmp.txt
eval ${MESHLABSERVER} -i ${FILE_NAME}_transformed.ply -o ${FILE_NAME}_transformed.ply -m vc vn &> /dev/null

# Also transform individual files
FILES="$(ls +([0-9]).ply | sort -n) top.ply bottom.ply"
for FILE in ${FILES}; do
  ${LOW_COST_3D_RECONSTRUCTION_DIR}/transform -i ${FILE} -o ${FILE} -t tmp.txt
  eval ${MESHLABSERVER} -i ${FILE} -o ${FILE} -m vc vn &> /dev/null
done

rm tmp.ply tmp.txt

# Fine alignment - TODO
echo "----- Fine alignment -----"
echo "Opening meshlab with SFM model and ${FILE_NAME}_transformed.ply."
echo "Align using 4-point based for rigid transformation, apply ICP align. Fix matrix of tranformed mesh and save ${FILE_NAME}_transformed.ply"
eval ${MESHLAB} sfm_model.ply ${FILE_NAME}_transformed.ply &> /dev/null

# ----------------------------------------------------------------------

# Create hybrid mesh
MESH=${FILE_NAME}_transformed.ply
MODEL_NAME=model

if [[ ( ${2} && "${2,,}" == "dense" ) && ( ${3} && "${3,,}" == "hybrid" ) ]]; then
  echo "----- Create hybrid mesh -----"
  cp sfm_model.ply tmp.ply

  ${LOW_COST_3D_RECONSTRUCTION_DIR}/cloud_downsampling -i tmp.ply -o tmp_2.ply -s 0.05
  ${LOW_COST_3D_RECONSTRUCTION_DIR}/accumulate_clouds -i ${MESH} -t tmp_2.ply -o tmp_3.ply -r 0.05 -c 0 -d 0 -n

  mv tmp_3_negative.ply sfm_complement.ply
  ${LOW_COST_3D_RECONSTRUCTION_DIR}/accumulate_clouds -i sfm_complement.ply -t tmp.ply -o hybrid_model.ply -a

  MESH=hybrid_model.ply
  MODEL_NAME=hybrid

  rm tmp.ply tmp_2.ply tmp_3.ply
fi

# ----------------------------------------------------------------------

# Reconstruction
eval ${MESHLABSERVER} -i ${MESH} -o mesh_file.ply -m vc vq -s ${LOW_COST_3D_RECONSTRUCTION_DIR}/mesh_reconstruction.mlx 2> /dev/null

# Use the new model for texturization
cp mesh_file.ply ${SFM_DIR}/mesh_file.ply
${OPENMVS_DIR}/ReconstructMesh --input-file ${SFM_DIR}/model.mvs --output-file ${SFM_DIR}/${MODEL_NAME}_mesh.mvs --mesh-file mesh_file.ply --smooth 0 --working-folder ${SFM_DIR}

# Mesh texturing for computing a sharp and accurate texture to color the mesh
ORANGE=16744231
YELLOW=16776960
WHITE=16777215
BLACK=0
${OPENMVS_DIR}/TextureMesh --input-file ${SFM_DIR}/${MODEL_NAME}_mesh.mvs --patch-packing-heuristic 0 --cost-smoothness-ratio 1 --empty-color ${BLACK} --export-type ply --close-holes 50 --resolution-level 1 --working-folder ${SFM_DIR}

rm ${SFM_DIR}/*.log

# ----------------------------------------------------------------------

# Copy result to current dir
cp ${SFM_DIR}/${MODEL_NAME}_mesh_texture.png ${PWD}
cp ${SFM_DIR}/${MODEL_NAME}_mesh_texture.ply ${PWD}
