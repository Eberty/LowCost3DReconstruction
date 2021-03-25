#!/bin/bash

# Copy this script into a folder with an 'images' (SfM) subfolder

# ----------------------------------------------------------------------

# Print help message
for ARG in "${@:1}"; do
  if [[ ${ARG} == "-h" || ${ARG} == "--help" ]]; then
    echo "Usage: "
    echo "    source ${BASH_SOURCE} <meshlab_bundler.out> <raster_image_files>"
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

# Verify if the first argument is a valid name
if [[ ! -f ${1} || ${1: -4} != ".out" ]]; then
  echo "Please inform a valid '.out' file as argument."
  return;
fi

# Verify if the others arguments are valid images
if [[ ! "${@:2}" ]]; then
  echo "Please inform valid images."
  return;
else
  for IMG in "${@:2}"; do
    if [[ ! -f ${IMG} || ! ( ${IMG: -4} == ".png" || ${IMG: -4} == ".jpg" ) ]]; then
      echo "Please inform a valid images as argument (error on parsing ${IMG})"
      return;
    fi
  done
fi

# ----------------------------------------------------------------------

# Set LowCost3DReconstruction directory
LOW_COST_3D_RECONSTRUCTION_DIR=/usr/local/LowCost3DReconstruction

# Set openMVS directory
OPENMVS_DIR=/usr/local/bin/OpenMVS

# Set meshlab commands
MESHLABSERVER="LC_ALL=C ${LOW_COST_3D_RECONSTRUCTION_DIR}/MeshLabServer2020.12-linux.AppImage"

# Set mesh file name
FILE_NAME=${1}

# Working directory
cp ${FILE_NAME} ${SFM_DIR}/${FILE_NAME}
for IMG in "${@:2}"; do
  cp ${IMG} ${SFM_DIR}/
done

# ----------------------------------------------------------------------

# Convert colmap project into a Bundler project
colmap model_converter --input_path ${SFM_DIR}/sparse/0 --output_path ${SFM_DIR}/bundler --output_type Bundler
mv ${SFM_DIR}/bundler.bundle.out ${SFM_DIR}/bundle.out
mv ${SFM_DIR}/bundler.list.txt ${SFM_DIR}/bundle-list.txt

COUNTER=0
while IFS= read -r line
do
  sed -i "$[${COUNTER} +1]s|.*|${SFM_DIR}/undistorted_images/$(printf '%05d' ${COUNTER}).png|" ${SFM_DIR}/bundle-list.txt
  COUNTER=$[${COUNTER} +1]
done < "${SFM_DIR}/bundle-list.txt"

# ----------------------------------------------------------------------

# Merge bundle files
${LOW_COST_3D_RECONSTRUCTION_DIR}/bundle_merge -i ${@:2} -m ${SFM_DIR}/${FILE_NAME} -b ${SFM_DIR}/bundle.out -l ${SFM_DIR}/bundle-list.txt -p merged
rm ${SFM_DIR}/bundle.out
rm ${SFM_DIR}/bundle-list.txt

# ----------------------------------------------------------------------

# Use the new file for texturization
${OPENMVS_DIR}/InterfaceVisualSFM --input-file ${SFM_DIR}/merged.bundle.out --output-file ${SFM_DIR}/final.mvs --working-folder ${SFM_DIR}
${OPENMVS_DIR}/ReconstructMesh --input-file ${SFM_DIR}/final.mvs --mesh-file ${SFM_DIR}/mesh_file.ply --smooth 0 --working-folder ${SFM_DIR}

# Mesh texturing for computing a sharp and accurate texture to color the mesh
ORANGE=16744231
YELLOW=16776960
WHITE=16777215
BLACK=0
${OPENMVS_DIR}/TextureMesh --input-file ${SFM_DIR}/final_mesh.mvs --patch-packing-heuristic 0 --cost-smoothness-ratio 1 --empty-color ${BLACK} --working-folder ${SFM_DIR} --export-type ply --close-holes 50 --resolution-level 1

rm ${SFM_DIR}/*.log

# ----------------------------------------------------------------------

# Copy result to current dir
cp ${SFM_DIR}/final_mesh_texture.png ${PWD}
cp ${SFM_DIR}/final_mesh_texture.ply ${PWD}

