#!/bin/bash

# Copy this script into a folder with a subfolder of images

# Extract images from video if need
# ffmpeg -i video.mp4 -vf fps=2 image-%3d.png

# ----------------------------------------------------------------------

# Set images directory
CUR_DIR=${PWD}
IMAGES_DIR=${CUR_DIR}/images

# Verify directory existence
if [[ ! -d ${IMAGES_DIR} || ! "$(ls -A ${IMAGES_DIR})" ]]; then
    echo "Directory ${CUR_DIR}/images not found or empty."
    return;
fi

# ----------------------------------------------------------------------

# Set GPU usage
GPU=true
if [[ ${1} && ( "${1,,}" == "false" || "${1,,}" == "cpu" ) ]]; then
    GPU=false
else
    if [[ ${1} && ( "${1,,}" == "true" || "${1,,}" == "gpu" ) ]]; then
        GPU=true
    else
        echo "Please inform gpu usage (true or false)."
        return;
    fi
fi
echo "Using GPU: ${GPU}"

# ----------------------------------------------------------------------

# Set colmap command
COLMAP_BIN=colmap

# Set openMVS directory
OPENMVS_DIR=/usr/local/bin/OpenMVS

# Set directory where are the necessary executable for this pipeline
EXE_DIR=/usr/local/LowCost3DReconstruction

# Set meshlabserver command
MESHLABSERVER="LC_ALL=C meshlab.meshlabserver"

# Set directory with meshlab scripts
MESHLAB_SCRIPTS_DIR=/usr/local/LowCost3DReconstruction

# ----------------------------------------------------------------------

# Working directory
SFM_DIR=${IMAGES_DIR}/sfm
mkdir ${SFM_DIR}
cp ${IMAGES_DIR}/* ${SFM_DIR}
cd ${SFM_DIR}

# ----------------------------------------------------------------------

# Run SFM: sparse point cloud and camera pose estimation
${COLMAP_BIN} feature_extractor --database_path ${SFM_DIR}/database.db --image_path ${SFM_DIR} --SiftExtraction.use_gpu=${GPU}
${COLMAP_BIN} exhaustive_matcher --database_path ${SFM_DIR}/database.db --SiftMatching.use_gpu=${GPU}
mkdir ${SFM_DIR}/sparse
${COLMAP_BIN} mapper --database_path ${SFM_DIR}/database.db --image_path ${SFM_DIR} --output_path ${SFM_DIR}/sparse

# Convert colmap project into TXT files
# mkdir ${SFM_DIR}/txt
# ${COLMAP_BIN} model_converter --input_path ${SFM_DIR}/sparse/0 --output_path ${SFM_DIR}/txt --output_type TXT

# Convert colmap project into an OpenMVS project
${COLMAP_BIN} model_converter --input_path ${SFM_DIR}/sparse/0 --output_path ${SFM_DIR}/model.ply --output_type PLY
${COLMAP_BIN} model_converter --input_path ${SFM_DIR}/sparse/0 --output_path ${SFM_DIR}/model.nvm --output_type NVM
${OPENMVS_DIR}/InterfaceVisualSFM ${SFM_DIR}/model.nvm

# ----------------------------------------------------------------------

# Remove unnecessary points that no belong to object
${EXE_DIR}/crop_cloud -i ${SFM_DIR}/model.ply -o ${SFM_DIR}/model_outlier_removal.ply --radius 2.0
${EXE_DIR}/normal_estimation -i ${SFM_DIR}/model_outlier_removal.ply -o ${SFM_DIR}/model_outlier_removal.ply --neighbors 50 --centroid

cp ${MESHLAB_SCRIPTS_DIR}/normal_normalize.mlx ${PWD}
eval ${MESHLABSERVER} -i ${SFM_DIR}/model_outlier_removal.ply -o ${SFM_DIR}/model_outlier_removal.ply -m vc vn -s normal_normalize.mlx 2> /dev/null
rm ${PWD}/normal_normalize.mlx

# ----------------------------------------------------------------------

# Run MVS densify point-cloud for obtaining a complete and accurate as possible point-cloud
if [[ ${2} && "${2,,}" == "dense" ]]; then
    ${OPENMVS_DIR}/DensifyPointCloud ${SFM_DIR}/model.mvs
    ${EXE_DIR}/crop_cloud -i ${SFM_DIR}/model_dense.ply -o ${SFM_DIR}/model_dense_outlier_removal.ply --radius 1.7
    ${EXE_DIR}/outlier_removal -i ${SFM_DIR}/model_dense_outlier_removal.ply -o ${SFM_DIR}/model_dense_outlier_removal.ply --neighbors 100 --dev_mult 10.0
    ${EXE_DIR}/cloud_downsampling -i ${SFM_DIR}/model_dense_outlier_removal.ply -o ${SFM_DIR}/model_dense_outlier_removal.ply --leaf_size 0.01
    ${EXE_DIR}/normal_estimation -i ${SFM_DIR}/model_dense_outlier_removal.ply -o ${SFM_DIR}/model_dense_outlier_removal.ply --neighbors 75 --centroid

    cp ${MESHLAB_SCRIPTS_DIR}/normal_normalize.mlx ${PWD}
    eval ${MESHLABSERVER} -i ${SFM_DIR}/model_dense_outlier_removal.ply -o ${SFM_DIR}/model_dense_outlier_removal.ply -m vc vn -s normal_normalize.mlx 2> /dev/null
    rm ${PWD}/normal_normalize.mlx

    # Mesh reconstruction for estimating a mesh surface that explains the best the input point-cloud
    # ${OPENMVS_DIR}/ReconstructMesh ${SFM_DIR}/model_dense.mvs --remove-spurious 50

    # Mesh refinement for recovering all fine details
    # ${OPENMVS_DIR}/RefineMesh --resolution-level 1 ${SFM_DIR}/model_dense_mesh.mvs

    # Mesh texturing for computing a sharp and accurate texture to color the mesh
    # ${OPENMVS_DIR}/TextureMesh --export-type ply ${SFM_DIR}/model_dense_mesh_refine.mvs
fi

rm ${SFM_DIR}/*.log

# ----------------------------------------------------------------------

cd ${CUR_DIR}
# LC_ALL=C meshlab ${SFM_DIR}/model_outlier_removal.ply &> /dev/null
