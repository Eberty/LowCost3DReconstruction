#!/bin/bash

# Copy this script into a folder and execute

# ----------------------------------------------------------------------

# Verify if the argument is a valid file (Use it for now)
if [[ ! -f ${1} || ${1: -4} != ".ply" ]]; then
	echo "Please inform a valid mesh file as argument."
	return;
fi

# ----------------------------------------------------------------------

# Set mesh file name (remove extention)
FILE_NAME=${1%.*}

# Set openMVS directory
OPENMVS_DIR=/usr/local/bin/OpenMVS

# Working directory (SFM folder)
SFM_DIR=${PWD}/images/temp

# Set directory where are the necessary executable for this pipeline
EXE_DIR=/usr/local/msc-research

# Set directory with meshlab scripts
MESHLAB_SCRIPTS_DIR=/usr/local/msc-research

# ----------------------------------------------------------------------

# Step 1: Alignment -> Accurately register two point clouds created from different image spectrums - TODO

# Open meshlab with result, open ${FILE_NAME}.ply, align using 4-point based for rigid transformation and run ICP for fine transformation
# Fix matrix of tranformed mesh and save as ${FILE_NAME}_transformed.ply
meshlab ${SFM_DIR}/model_outlier_removal.ply

# ----------------------------------------------------------------------

# Step 2: Reconstruction
cp ${MESHLAB_SCRIPTS_DIR}/mesh_reconstruction.mlx ${PWD}
meshlab.meshlabserver -i ${FILE_NAME}_transformed.ply -o mesh_file.ply -s mesh_reconstruction.mlx
rm ${PWD}/mesh_reconstruction.mlx

# ----------------------------------------------------------------------

# Step 3: Use the new model for texturization
${OPENMVS_DIR}/ReconstructMesh ${SFM_DIR}/model.mvs --mesh-file mesh_file.ply --smooth 0 --working-folder ${SFM_DIR}

# ----------------------------------------------------------------------

# Step 4: Mesh texturing for computing a sharp and accurate texture to color the mesh
${OPENMVS_DIR}/TextureMesh ${SFM_DIR}/model_mesh.mvs --patch-packing-heuristic 0 --cost-smoothness-ratio 1 --empty-color 16744231 --working-folder ${SFM_DIR}

# ----------------------------------------------------------------------

# Step 5: Copy result to current dir
cp ${SFM_DIR}/model_mesh_texture.png ${PWD}
cp ${SFM_DIR}/model_mesh_texture.ply ${PWD}

# Convert to OBJ
meshlab.meshlabserver -i model_mesh_texture.ply -o model_mesh_texture.obj -m vn wt
mv model_mesh_texture.obj.mtl model_mesh_texture.mtl
sed -i 's/model_mesh_texture.obj.mtl/model_mesh_texture.mtl/' model_mesh_texture.obj

# ----------------------------------------------------------------------

# Step 6: Texturization of bottom view - TODO
# See: https://github.com/Eberty/model_view
# ${MODEL_VIEW_DIR}/model_view model_mesh_texture.obj meshlab_project.mlp

# ----------------------------------------------------------------------

# Step 7: Validate pipeline - Compare the result with a ground truth or something else
# See: https://github.com/Yochengliu/awesome-point-cloud-analysis
