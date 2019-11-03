#!/bin/bash

# Copy this script into a folder and execute

# ----------------------------------------------------------------------

# Verify if the argument is a valid file (Use it for now)
if [[ ! -f "$1" || ${1: -4} != ".ply" ]]; then
	echo "Please inform a valid mesh file as argument."
	return;
fi

# ----------------------------------------------------------------------

# Set mesh file
INPUT_FILE=$1

# Set openMVS directory
OPENMVS_DIR=/usr/local/bin/OpenMVS

# Working directory (SFM folder)
SFM_DIR=$PWD/images/temp

# Set directory where are the necessary executable for this pipeline
EXE_DIR=/home/eberty/Dropbox/DocumentosMestrado/Codigos/msc-research/bin

# ----------------------------------------------------------------------

# Steps for pipelines integration

# Step 1: Alignment -> Accurately register two point clouds created from different image spectrums - TODO

# Note: As some of steps aren't working yet, we need do this steps manually by now:
# Crop sparse ply file with just the object to remove outliers
# $EXE_DIR/crop_cloud --input $SFM_DIR/model.ply --gui
# Calculate the normals
# $EXE_DIR/normal_estimation --input $SFM_DIR/model_croped.ply --output $SFM_DIR/model_croped_normal.ply
# Open meshlab with result
# LC_ALL=C meshlab $SFM_DIR/model_croped_normal.ply
# Open ${INPUT_FILE}.ply, align using 4-point based for rigid transformation and run ICP for fine transformation
# Fix matrix of tranformed mesh and save as ${INPUT_FILE}.ply

# ----------------------------------------------------------------------

# Step 2: Reconstruction - TODO
# meshlabserver -i ${INPUT_FILE}.ply -o mesh_file.ply -s meshlab_script_reconstruction.mlx -om vc vn

# ----------------------------------------------------------------------

# Step 3: Use the new model for texturization
$OPENMVS_DIR/ReconstructMesh $SFM_DIR/model.mvs --mesh-file mesh_file.ply --smooth 0

# ----------------------------------------------------------------------

# Step 4: Mesh texturing for computing a sharp and accurate texture to color the mesh
$OPENMVS_DIR/TextureMesh $SFM_DIR/model_mesh.mvs --patch-packing-heuristic 0 --cost-smoothness-ratio 1 --empty-color 16744231

# Result
# LC_ALL=C meshlab $SFM_DIR/model_mesh_texture.ply

# ----------------------------------------------------------------------

# Step 5: Texturization of bottom view - TODO
# See: https://github.com/Eberty/model_view

# ----------------------------------------------------------------------

# Step 6: Validate pipeline - Compare the result with a ground truth or something else - TODO
