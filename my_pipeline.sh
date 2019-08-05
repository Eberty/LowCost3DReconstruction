#!/bin/bash

# Copy this script into a folder with images

# Extract images from video if need
# ffmpeg -i inputfile.avi -r 1 image-%3d.png

# Verify if the argument is a valid file (Use it for now)
if [[ ! -f "$1" || ${1: -4} != ".ply" ]]; then
	echo "Please inform a valid mesh-file as argument."
	return;
fi

# Set mesh file
MESH_FILE=$1
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
cp $MESH_FILE $TEMP_DIR/mesh_file.ply
cd $TEMP_DIR

# Run SFM: sparse point cloud and camera pose estimation
$COLMAP feature_extractor --database_path $TEMP_DIR/database.db --image_path $TEMP_DIR --SiftExtraction.use_gpu=false
$COLMAP exhaustive_matcher --database_path $TEMP_DIR/database.db --SiftMatching.use_gpu=false
mkdir $TEMP_DIR/sparse
$COLMAP mapper --database_path $TEMP_DIR/database.db --image_path $TEMP_DIR --output_path $TEMP_DIR/sparse

# Convert colmap project into an OpenMVS project
$COLMAP model_converter --input_path $TEMP_DIR/sparse/0 --output_path $TEMP_DIR/model.ply --output_type PLY
$COLMAP model_converter --input_path $TEMP_DIR/sparse/0 --output_path $TEMP_DIR/model.nvm --output_type NVM
$OPENMVS_DIR/InterfaceVisualSFM $TEMP_DIR/model.nvm

# Eberty files (In the future, the arg will be replaced by this steps) - TODO
# Step 1: Capture -> depth_capture + robesa_control
# Step 2: Super resolution -> super_resolution

# Note about alignment steps: Maybe in the future we can use SFM to estimate the kinetic device pose
# and use it for alignment of point clouds, without the rotation table - TODO
# Step 3: Alignment -> rotate_align + pair_align (top and bottom)
# Step 4: Alignment [recommended] -> clouds_registration (ICP)

# Optional - DONE
# Step 5: Clean [optional] -> outlier_removal + crop_cloud (Remove unnecessary points, that no belong to object)
# Step 6: Clean [optional] -> cloud_downsampling (Reduce the number of points)

# Steps for pipelines integration - TODO
# Step 7: Alignment -> feature_based_alignment (accurately register two point clouds created from different image spectrums)
# Step 8: Reconstruction -> normal_estimation
# Step 9: Reconstruction -> poisson_reconstruction

# Use the new model to texturization
$OPENMVS_DIR/ReconstructMesh $TEMP_DIR/model.mvs --mesh-file $TEMP_DIR/mesh_file.ply --smooth 0

# Mesh refinement for recovering all fine details (Optional)
# $OPENMVS_DIR/RefineMesh --resolution-level 1 model_mesh.mvs

# Mesh texturing for computing a sharp and accurate texture to color the mesh
$OPENMVS_DIR/TextureMesh $TEMP_DIR/model_mesh.mvs

# Texturization of bottom view - TODO
# Step 10: Texturization -> bottom_view_texturization

# Validate pipeline - TODO
# Step 11: Comparison -> feature_based_alignment + model_accuracy (Compare the result with a ground truth)

# This file is no longer needed
rm $TEMP_DIR/mesh_file.ply

# Copy results to a new directory
mkdir $IMAGES_DIR/model
cp $TEMP_DIR/model_mesh_texture.png $IMAGES_DIR/model
cp $TEMP_DIR/*.ply $IMAGES_DIR/model
cd $IMAGES_DIR

# If you want to automate removal of the working folder, use the following line.
# Don't use it if you want to keep intermediate steps.
# rm -rf $TEMP_DIR

# LC_ALL=C meshlab $IMAGES_DIR/model/model_mesh_texture.ply
