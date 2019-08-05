#!/bin/bash

# Copy this script into a folder with images

# Extract images from video if need
# ffmpeg -i inputfile.avi -r 1 image-%3d.png

# Set images directory
IMAGES_DIR=$PWD

# Set colmap command
COLMAP=colmap

# Set openMVS directory
OPENMVS_DIR=/usr/local/bin/OpenMVS

# Working directory (temporary folder)
TEMP_DIR=$IMAGES_DIR/temp
mkdir $TEMP_DIR
cp *.JPG *.JPEG *.PNG *.jpg *.jpeg *.png $TEMP_DIR
cd $TEMP_DIR

# Run SFM: sparse point cloud and camera pose estimation
$COLMAP feature_extractor --database_path $TEMP_DIR/database.db --image_path $TEMP_DIR --SiftExtraction.use_gpu=false
$COLMAP exhaustive_matcher --database_path $TEMP_DIR/database.db --SiftMatching.use_gpu=false
mkdir $TEMP_DIR/sparse
$COLMAP mapper --database_path $TEMP_DIR/database.db --image_path $TEMP_DIR --output_path $TEMP_DIR/sparse

# Convert colmap project into an OpenMVS project
$COLMAP model_converter --input_path sparse/0 --output_path model.ply --output_type PLY
$COLMAP model_converter --input_path sparse/0 --output_path model.nvm --output_type NVM
$OPENMVS_DIR/InterfaceVisualSFM model.nvm

# Run SFM:
# Dense point-cloud reconstruction for obtaining a complete and accurate as possible point-cloud
$OPENMVS_DIR/DensifyPointCloud model.mvs
# Mesh reconstruction for estimating a mesh surface that explains the best the input point-cloud
$OPENMVS_DIR/ReconstructMesh --remove-spurious 50 model_dense.mvs
# Mesh refinement for recovering all fine details
$OPENMVS_DIR/RefineMesh --resolution-level 1 model_dense_mesh.mvs
# Mesh texturing for computing a sharp and accurate texture to color the mesh
$OPENMVS_DIR/TextureMesh model_dense_mesh_refine.mvs

# Copy results to images dir
mkdir $IMAGES_DIR/model
cp $TEMP_DIR/model_dense_mesh_refine_texture.png $IMAGES_DIR/model
cp $TEMP_DIR/*.ply $IMAGES_DIR/model
cd $IMAGES_DIR

# If you want to automate removal of the working folder, use the following line.
# Don't use it if you want to keep intermediate steps.
# rm -rf $TEMP_DIR

# LC_ALL=C meshlab $IMAGES_DIR/model/model_dense_mesh_refine_texture.ply
