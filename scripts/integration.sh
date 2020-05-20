#!/bin/bash

# Copy this script into a folder and execute

# ----------------------------------------------------------------------

# Print help message
for ARG in "${@:1}"; do
    if [[ ${ARG} == "-h" || ${ARG} == "--help" ]]; then
        echo "Usage: "
        echo "      source ${BASH_SOURCE} <object_name.ply> [dense [hybrid]]"
        return;
    fi
done

# ----------------------------------------------------------------------

# Verify if the argument is a valid file
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
SFM_DIR=${PWD}/images/sfm

# Set Super4PCS command
Super4PCS_BIN=/usr/local/bin/Super4PCS

# Set directory where are the necessary executable for this pipeline
EXE_DIR=/usr/local/LowCost3DReconstruction

# Set meshlabserver command
MESHLABSERVER="LC_ALL=C meshlab.meshlabserver"

# Set directory with meshlab scripts
MESHLAB_SCRIPTS_DIR=/usr/local/LowCost3DReconstruction

# ----------------------------------------------------------------------

# Step 1: Alignment -> Accurately register two point clouds created from different image spectrums
if [[ ${2} && "${2,,}" == "dense" ]]; then
    cp ${SFM_DIR}/model_dense_outlier_removal.ply sfm_model.ply
else
    cp ${SFM_DIR}/model_outlier_removal.ply sfm_model.ply
fi

${EXE_DIR}/centroid_align -i ${FILE_NAME}.ply -t sfm_model.ply -o ${FILE_NAME}.ply
eval ${MESHLABSERVER} -i ${FILE_NAME}.ply -o ${FILE_NAME}.ply -m vc vn 2> /dev/null

${Super4PCS_BIN} -i sfm_model.ply ${FILE_NAME}.ply -r tmp.ply -t 10 -m tmp.txt -d 0.13
sed -i '1,2d' tmp.txt
${EXE_DIR}/transform -i ${FILE_NAME}.ply -o ${FILE_NAME}_transformed.ply -t tmp.txt
eval ${MESHLABSERVER} -i ${FILE_NAME}_transformed.ply -o ${FILE_NAME}_transformed.ply -m vc vn 2> /dev/null
rm tmp.ply tmp.txt

# Fine alignment - TODO
echo "----- Fine alignment -----"
echo "Open meshlab with result, open ${FILE_NAME}_transformed.ply, align using 4-point based for rigid transformation, apply ICP align"
echo "Fix matrix of tranformed mesh and save ${FILE_NAME}_transformed.ply"
LC_ALL=C snap run meshlab sfm_model.ply 2> /dev/null

# ----------------------------------------------------------------------

# Create hybrid mesh
MESH=${FILE_NAME}_transformed.ply
MODEL_NAME=model

if [[ ( ${2} && "${2,,}" == "dense" ) && ( ${3} && "${3,,}" == "hybrid" ) ]]; then
    echo "----- Create hybrid mesh -----"
    echo "In order to make the mesh as accurate as possible, remove the points that do not belong to the object of interest in the tmp.ply file and save it"
    cp sfm_model.ply tmp.ply
    LC_ALL=C snap run meshlab tmp.ply 2> /dev/null

    ${EXE_DIR}/cloud_downsampling -i tmp.ply -o tmp_2.ply -s 0.05
    ${EXE_DIR}/accumulate_clouds -i ${MESH} -t tmp_2.ply -o tmp_3.ply -r 0.05 -c 0 -d 0 -n

    mv tmp_3_negative.ply sfm_complement.ply
    ${EXE_DIR}/accumulate_clouds -i sfm_complement.ply -t tmp.ply -o hybrid_model.ply -a

    MESH=hybrid_model.ply
    MODEL_NAME=hybrid

    rm tmp.ply tmp_2.ply tmp_3.ply
fi

# ----------------------------------------------------------------------

# Step 2: Reconstruction
cp ${MESHLAB_SCRIPTS_DIR}/mesh_reconstruction.mlx ${PWD}
eval ${MESHLABSERVER} -i ${MESH} -o mesh_file.ply -s mesh_reconstruction.mlx 2> /dev/null
rm ${PWD}/mesh_reconstruction.mlx

# ----------------------------------------------------------------------

# Step 3: Use the new model for texturization
cp mesh_file.ply ${SFM_DIR}/mesh_file.ply
${OPENMVS_DIR}/ReconstructMesh ${SFM_DIR}/model.mvs --mesh-file mesh_file.ply --smooth 0 --working-folder ${SFM_DIR} --output-file ${SFM_DIR}/${MODEL_NAME}_mesh.mvs

# ----------------------------------------------------------------------

# Step 4: Mesh texturing for computing a sharp and accurate texture to color the mesh
ORANGE=16744231
YELLOW=16776960
WHITE=16777215
BLACK=0
${OPENMVS_DIR}/TextureMesh ${SFM_DIR}/${MODEL_NAME}_mesh.mvs --patch-packing-heuristic 0 --cost-smoothness-ratio 1 --empty-color ${ORANGE} --working-folder ${SFM_DIR} --export-type ply --close-holes 50

rm ${SFM_DIR}/*.log

# ----------------------------------------------------------------------

# Step 5: Copy result to current dir
cp ${SFM_DIR}/${MODEL_NAME}_mesh_texture.png ${PWD}
cp ${SFM_DIR}/${MODEL_NAME}_mesh_texture.ply ${PWD}

# Convert to OBJ
# eval ${MESHLABSERVER} -i ${MODEL_NAME}_mesh_texture.ply -o ${MODEL_NAME}_mesh_texture.obj -m vn wt 2> /dev/null
# mv ${MODEL_NAME}_mesh_texture.obj.mtl ${MODEL_NAME}_mesh_texture.mtl
# sed -i 's/${MODEL_NAME}_mesh_texture.obj.mtl/${MODEL_NAME}_mesh_texture.mtl/' ${MODEL_NAME}_mesh_texture.obj

# ----------------------------------------------------------------------

# LC_ALL=C snap run meshlab ${MODEL_NAME}_mesh_texture.ply 2> /dev/null
