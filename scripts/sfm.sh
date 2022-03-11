#!/bin/bash

# Copy this script into a folder with an 'images' subfolder

# Extract images from a video if need
# ffmpeg -i video.mp4 -vf fps=2 image-%3d.png

# ----------------------------------------------------------------------

# Print help message
for ARG in "${@:1}"; do
  if [[ ${ARG} == "-h" || ${ARG} == "--help" ]]; then
    echo "Usage: "
    echo "    source ${BASH_SOURCE}"
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

# Set GPU usage
GPU=false
if [[ $(which nvcc) ]]; then
  GPU=true
fi
echo "Using GPU: ${GPU}"

# ----------------------------------------------------------------------

# Set LowCost3DReconstruction directory
LOW_COST_3D_RECONSTRUCTION_DIR=/usr/local/LowCost3DReconstruction

# Set openMVS directory
OPENMVS_DIR=/usr/local/bin/OpenMVS

# Set meshlab commands
MESHLABSERVER="LC_ALL=C ${LOW_COST_3D_RECONSTRUCTION_DIR}/MeshLabServer2020.12-linux.AppImage"

# ----------------------------------------------------------------------

# Run SFM: sparse point cloud and camera pose estimation
colmap feature_extractor --database_path ${SFM_DIR}/database.db --image_path ${SFM_DIR} --SiftExtraction.use_gpu=${GPU}
colmap exhaustive_matcher --database_path ${SFM_DIR}/database.db --SiftMatching.use_gpu=${GPU}

mkdir ${SFM_DIR}/sparse
colmap mapper --database_path ${SFM_DIR}/database.db --image_path ${SFM_DIR} --output_path ${SFM_DIR}/sparse

# Convert colmap project into an OpenMVS project
colmap model_converter --input_path ${SFM_DIR}/sparse/0 --output_path ${SFM_DIR}/model.ply --output_type PLY
colmap model_converter --input_path ${SFM_DIR}/sparse/0 --output_path ${SFM_DIR}/model.nvm --output_type NVM
${OPENMVS_DIR}/InterfaceVisualSFM --input-file ${SFM_DIR}/model.nvm --working-folder ${SFM_DIR}

# ----------------------------------------------------------------------

# Run MVS densify point-cloud for obtaining a complete and accurate as possible point-cloud
${OPENMVS_DIR}/DensifyPointCloud --input-file ${SFM_DIR}/model.mvs --working-folder ${SFM_DIR}
${OPENMVS_DIR}/DensifyPointCloud --input-file ${SFM_DIR}/model_dense.mvs --output-file model_dense.mvs --filter-point-cloud -1 --working-folder ${SFM_DIR}

# Mesh reconstruction for estimating a mesh surface that explains the best the input point-cloud
# ${OPENMVS_DIR}/ReconstructMesh --input-file ${SFM_DIR}/model_dense_filtered.mvs --remove-spurious 60 --working-folder ${SFM_DIR}

# Mesh refinement for recovering all fine details
# ${OPENMVS_DIR}/RefineMesh --input-file ${SFM_DIR}/model_dense_filtered_mesh.mvs --resolution-level 1 --working-folder ${SFM_DIR}

# Mesh texturing for computing a sharp and accurate texture to color the mesh
# ${OPENMVS_DIR}/TextureMesh --input-file ${SFM_DIR}/model_dense_filtered_mesh_refine.mvs --export-type ply --working-folder ${SFM_DIR}

rm ${SFM_DIR}/*.log

# ----------------------------------------------------------------------

# Crop, downsampling, planar segmentation and outlier removal
${LOW_COST_3D_RECONSTRUCTION_DIR}/cluster_extraction -i ${SFM_DIR}/model_dense_filtered.ply -o ${SFM_DIR}/model_dense_outlier_removal.ply --tolerance 0.02
# ${LOW_COST_3D_RECONSTRUCTION_DIR}/crop_cloud -i ${SFM_DIR}/model_dense_filtered.ply -o ${SFM_DIR}/model_dense_outlier_removal.ply --radius 2.5
${LOW_COST_3D_RECONSTRUCTION_DIR}/cloud_downsampling -i ${SFM_DIR}/model_dense_outlier_removal.ply -o ${SFM_DIR}/model_dense_outlier_removal.ply --leaf_size 0.01
${LOW_COST_3D_RECONSTRUCTION_DIR}/planar_segmentation -i ${SFM_DIR}/model_dense_outlier_removal.ply -o ${SFM_DIR}/model_dense_outlier_removal.ply --threshold 0.05
# ${LOW_COST_3D_RECONSTRUCTION_DIR}/outlier_removal -i ${SFM_DIR}/model_dense_outlier_removal.ply -o ${SFM_DIR}/model_dense_outlier_removal.ply --neighbors 50 --dev_mult 5.0

eval ${MESHLABSERVER} -i ${SFM_DIR}/model_dense_outlier_removal.ply -o ${SFM_DIR}/model_dense_outlier_removal.ply -m vc vn &> /dev/null
