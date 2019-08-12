#!/bin/bash

# Copy this script into a folder with images

# Extract images from video if need
# ffmpeg -i video.avi -vf fps=2 image-%3d.png

# Verify if the argument is a valid file (Use it for now)
if [[ ! -f "$1" || ${1: -4} != ".ply" ]]; then
	echo "Please inform a valid mesh file as argument."
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

# ----------------------------------------------------------------------

# Run SFM: sparse point cloud and camera pose estimation
$COLMAP feature_extractor --database_path $TEMP_DIR/database.db --image_path $TEMP_DIR --SiftExtraction.use_gpu=false
$COLMAP exhaustive_matcher --database_path $TEMP_DIR/database.db --SiftMatching.use_gpu=false
mkdir $TEMP_DIR/sparse
$COLMAP mapper --database_path $TEMP_DIR/database.db --image_path $TEMP_DIR --output_path $TEMP_DIR/sparse

# Debug: convert colmap project into TXT files
# mkdir $TEMP_DIR/txt
# $COLMAP model_converter --input_path $TEMP_DIR/sparse/0 --output_path $TEMP_DIR/txt --output_type TXT

# Convert colmap project into an OpenMVS project
$COLMAP model_converter --input_path $TEMP_DIR/sparse/0 --output_path $TEMP_DIR/model.ply --output_type PLY
$COLMAP model_converter --input_path $TEMP_DIR/sparse/0 --output_path $TEMP_DIR/model.nvm --output_type NVM
$OPENMVS_DIR/InterfaceVisualSFM $TEMP_DIR/model.nvm

# ----------------------------------------------------------------------

# In the future, the arg will be replaced by the following steps:
# KINECT_DIR=$IMAGES_DIR/kinect
# mkdir $KINECT_DIR
# cd $KINECT_DIR

# Set directory where are the necessary executable for this pipeline
# EXE_DIR=home/eberty/Dropbox/DocumentosMestrado/Codigos/msc-research/executables
# Set prefix of files containing captured points
# ARTEFACT_NAME=artefact

# Step 1: Capture
# $EXE_DIR/depth_capture --capture_name ${ARTEFACT_NAME} --capture_step 20 --sr_size 16

# Step 2: Super resolution
# $EXE_DIR/super_resolution --capture_name ${ARTEFACT_NAME} --capture_step 20 --num_captures $(( 360 / 20 )) --sr_size 16
# $EXE_DIR/super_resolution --capture_name ${ARTEFACT_NAME} --sr_size 16 --top
# $EXE_DIR/super_resolution --capture_name ${ARTEFACT_NAME} --sr_size 16 --bottom

# Note about alignment steps: Maybe in the future we can use SFM to estimate the kinetic device pose
# and use it for alignment of point clouds, without the rotation table

# Step 3: Coarse Alignment
# $EXE_DIR/rotate_align --capture_name ${ARTEFACT_NAME}_srmesh_ \
# --capture_step 20 --num_captures $(( 360 / 20 )) -x 44 -y 60 -z 632.5 \
# --accumulated_file ${ARTEFACT_NAME}_aligned.ply --gui

# $EXE_DIR/pair_align --point_cloud ${ARTEFACT_NAME}_srmesh_top.ply --ref_point_cloud ${ARTEFACT_NAME}_aligned.ply \
# --accumulated_file ${ARTEFACT_NAME}_top_aligned.ply --gui

# $EXE_DIR/pair_align --point_cloud ${ARTEFACT_NAME}_srmesh_bottom.ply --ref_point_cloud ${ARTEFACT_NAME}_top_aligned.ply \
# --accumulated_file ${ARTEFACT_NAME}_top_bottom_aligned.ply --gui

# Step 4: Fine alignment [recommended] -> clouds_registration (ICP) or incremental_registration
# This step is nesessary because the antecessor step just made a rigid transform
# TODO

# ----------------------------------------------------------------------

# Optional:

# Step 5: Remove unnecessary points that no belong to object
# $EXE_DIR/outlier_removal --input ${ARTEFACT_NAME}_top_bottom_aligned_refined.ply \
# --output ${ARTEFACT_NAME}_top_bottom_aligned_refined_cleaned.ply --neighbors 50 --dev_mult 1.0 --outliers_file

# $EXE_DIR/crop_cloud --input ${ARTEFACT_NAME}_top_bottom_aligned_refined_cleaned.ply \
# --min_x -1 --max_x 1 --min_y -1 --max_y 1 --min_z -1 --max_z 1 --save_removed --gui

# Step 6: Reduce the number of points [optional] - NOT USED
# $EXE_DIR/cloud_downsampling --leaf_size 1
# --input ${ARTEFACT_NAME}_top_bottom_aligned_refined_cleaned_croped.ply \
# --output ${ARTEFACT_NAME}_top_bottom_aligned_refined_cleaned_croped_downsampling.ply

# ----------------------------------------------------------------------

# Steps for pipelines integration

# Step 7: Alignment -> feature_based_alignment (accurately register two point clouds created from different image spectrums)
# We can also uses meshlab alignment (point based + ICP) for this
# TODO: Use --target $TEMP_DIR/model.ply or model_croped_normal_mesh.ply in note below

# Step 8: Normal estimation
# $EXE_DIR/normal_estimation --neighbors 50
# --input ${ARTEFACT_NAME}_top_bottom_aligned_refined_cleaned_croped_registered.ply \
# --output ${ARTEFACT_NAME}_top_bottom_aligned_refined_cleaned_croped_registered_normal.ply

# Step 9: Reconstruction - We can also uses meshlab poisson reconstruction for this
# $EXE_DIR/poisson_reconstruction
# --input ${ARTEFACT_NAME}_top_bottom_aligned_refined_cleaned_croped_registered_normal.ply \
# --output ${ARTEFACT_NAME}_top_bottom_aligned_refined_cleaned_croped_registered_normal_mesh.ply \
# --depth 8 --min_depth 4 --point_weight 4 --scale 1.1 \
# --solver_divide 8 --iso_divide 8 --samples_per_node 1 \
# --confidence 0 --output_polygons 0 --manifold 1

# cp ${ARTEFACT_NAME}_top_bottom_aligned_refined_cleaned_croped_registered_normal_mesh.ply $TEMP_DIR/mesh_file.ply

# ----------------------------------------------------------------------

# Note: As some of steps aren't working yet (step 7 mainly), we need do this steps manually by now:
# Crop sparse ply file with just the object to remove outliers
# $EXE_DIR/crop_cloud --input model.ply --gui
# Calculate the normals
# $EXE_DIR/normal_estimation --input model_croped.ply --output model_croped_normal.ply
# Generate a simple mesh
# $EXE_DIR/poisson_reconstruction --input model_croped_normal.ply --output model_croped_normal_mesh.ply
# Open meshlab with poisson reconstruction result
# LC_ALL=C meshlab model_croped_normal_mesh.ply
# Open mesh_file.ply, align using 4-point based for rigid transformation and run ICP for fine transformation
# Fix matrix of tranformed mesh and save as mesh_file.ply
# rm model_croped.ply model_croped_normal.ply model_croped_normal_mesh.ply

# ----------------------------------------------------------------------

# Use the new model for texturization
$OPENMVS_DIR/ReconstructMesh $TEMP_DIR/model.mvs --mesh-file $TEMP_DIR/mesh_file.ply --smooth 0

# Mesh texturing for computing a sharp and accurate texture to color the mesh
$OPENMVS_DIR/TextureMesh $TEMP_DIR/model_mesh.mvs --patch-packing-heuristic 0 --cost-smoothness-ratio 0.2 --empty-color 16744231

# Texturization of bottom view
# Step 10: Texturization -> bottom_view_texturization
# TODO

# ----------------------------------------------------------------------

# Validate pipeline
# Step 11: Comparison -> feature_based_alignment + model_accuracy (Compare the result with a ground truth or something else)
# TODO

# ----------------------------------------------------------------------

# Final steps

# This file is no longer needed
rm $TEMP_DIR/mesh_file.ply

# Copy results into a new directory
mkdir $IMAGES_DIR/model
cp $TEMP_DIR/model_mesh_texture.png $IMAGES_DIR/model
cp $TEMP_DIR/model_mesh_texture.ply $IMAGES_DIR/model
cd $IMAGES_DIR

# If you want to automate removal of the working folder, use the following line.
# Don't use it if you want to keep intermediate steps.
# rm -rf $TEMP_DIR

# LC_ALL=C meshlab $IMAGES_DIR/model/model_mesh_texture.ply
