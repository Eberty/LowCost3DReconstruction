#!/bin/bash

# Copy this script into a folder with a subfolder of images

# Extract images from video if need
# ffmpeg -i video.mp4 -vf fps=2 image-%3d.png

# ----------------------------------------------------------------------

# Set images directory
CUR_DIR=$PWD
IMAGES_DIR=$CUR_DIR/images

# Verify directory existence
if [[ ! -d "$IMAGES_DIR" || ! "$(ls -A $IMAGES_DIR)" ]]; then
	echo "Directory $CUR_DIR/images not found or empty."
	return;
fi

# ----------------------------------------------------------------------

# Set GPU usage
GPU=false
if [[ "$1" && "${1,,}" == "true" ]]; then
	GPU=true
fi
echo "Using GPU: $GPU"

# ----------------------------------------------------------------------

# Set colmap command
COLMAP_BIN=colmap

# Set openMVS directory
OPENMVS_DIR=/usr/local/bin/OpenMVS

# Set directory where are the necessary executable for this pipeline
EXE_DIR=/home/eberty/Dropbox/DocumentosMestrado/Codigos/msc-research/bin

# Working directory (temporary folder)
TEMP_DIR=$IMAGES_DIR/temp
mkdir $TEMP_DIR
cp $IMAGES_DIR/* $TEMP_DIR
cd $TEMP_DIR

# ----------------------------------------------------------------------

# Run SFM: sparse point cloud and camera pose estimation
$COLMAP_BIN feature_extractor --database_path $TEMP_DIR/database.db --image_path $TEMP_DIR --SiftExtraction.use_gpu=$GPU
$COLMAP_BIN spatial_matcher --database_path $TEMP_DIR/database.db --SiftMatching.use_gpu=$GPU
mkdir $TEMP_DIR/sparse
$COLMAP_BIN mapper --database_path $TEMP_DIR/database.db --image_path $TEMP_DIR --output_path $TEMP_DIR/sparse

# Convert colmap project into TXT files
# mkdir $TEMP_DIR/txt
# $COLMAP_BIN model_converter --input_path $TEMP_DIR/sparse/0 --output_path $TEMP_DIR/txt --output_type TXT

# Convert colmap project into an OpenMVS project
$COLMAP_BIN model_converter --input_path $TEMP_DIR/sparse/0 --output_path $TEMP_DIR/model.ply --output_type PLY
$COLMAP_BIN model_converter --input_path $TEMP_DIR/sparse/0 --output_path $TEMP_DIR/model.nvm --output_type NVM
$OPENMVS_DIR/InterfaceVisualSFM $TEMP_DIR/model.nvm

$OPENMVS_DIR/DensifyPointCloud $TEMP_DIR/model.mvs
$OPENMVS_DIR/ReconstructMesh $TEMP_DIR/model_dense.mvs --remove-spurious 20
$OPENMVS_DIR/RefineMesh --resolution-level 1 $TEMP_DIR/model_dense_mesh.mvs
$OPENMVS_DIR/TextureMesh --export-type ply $TEMP_DIR/model_dense_mesh_refine.mvs

# ----------------------------------------------------------------------

# Remove unnecessary points that no belong to object
$EXE_DIR/outlier_removal --input $TEMP_DIR/model_dense.ply --output $TEMP_DIR/model_dense_outlier_removal.ply --neighbors 100 --dev_mult 0.1

# ----------------------------------------------------------------------

cd $CUR_DIR
# LC_ALL=C meshlab $TEMP_DIR/model.ply
