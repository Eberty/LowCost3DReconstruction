#!/bin/bash

# Copy this script into a folder with images/sfm subfolder

# ----------------------------------------------------------------------

# Print help message
for ARG in "${@:1}"; do
    if [[ ${ARG} == "-h" || ${ARG} == "--help" ]]; then
        echo "Usage: "
        echo "      source ${BASH_SOURCE} <meshlab_bundler.out> <raster_image_files>"
        return;
    fi
done

# ----------------------------------------------------------------------

# Set sfm directory
CUR_DIR=${PWD}
SFM_DIR=${CUR_DIR}/images/sfm

# Verify directory existence
if [[ ! -d ${SFM_DIR} || ! "$(ls -A ${SFM_DIR})" ]]; then
    echo "Directory ${SFM_DIR} not found or empty."
    return;
fi

# ----------------------------------------------------------------------

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

# Set mesh file name
FILE_NAME=${1}

# Set colmap command
COLMAP_BIN=colmap

# Set openMVS directory
OPENMVS_DIR=/usr/local/bin/OpenMVS

# Set directory where are the necessary executable for this pipeline
EXE_DIR=/usr/local/LowCost3DReconstruction

# Set meshlabserver command
MESHLABSERVER="LC_ALL=C meshlab.meshlabserver"

# ----------------------------------------------------------------------

# Working directory
cp ${FILE_NAME} ${SFM_DIR}/${FILE_NAME}
for IMG in "${@:2}"; do
    cp ${IMG} ${SFM_DIR}/
done
cd ${SFM_DIR}

# ----------------------------------------------------------------------

# Convert colmap project into bundler files
${COLMAP_BIN} model_converter --input_path ${SFM_DIR}/sparse/0 --output_path ${SFM_DIR}/bundler --output_type Bundler
mv ${SFM_DIR}/bundler.bundle.out ${SFM_DIR}/bundle.out
mv ${SFM_DIR}/bundler.list.txt ${SFM_DIR}/bundle-list.txt

COUNTER=0
while IFS= read -r line
do
  sed -i "$[$COUNTER +1]s|.*|${SFM_DIR}/undistorted_images/$(printf '%05d' $COUNTER).png|" ${SFM_DIR}/bundle-list.txt
  COUNTER=$[$COUNTER +1]
done < "${SFM_DIR}/bundle-list.txt"

${OPENMVS_DIR}/InterfaceVisualSFM ${SFM_DIR}/bundler.out

# ----------------------------------------------------------------------

# Merge bundle files
${EXE_DIR}/bundle_merge -i ${@:2} -m ${FILE_NAME} -b bundle.out -l bundle-list.txt -p merged
rm ${SFM_DIR}/bundle.out
rm ${SFM_DIR}/bundle-list.txt

# ----------------------------------------------------------------------

# Use the new file for texturization
${OPENMVS_DIR}/InterfaceVisualSFM ${SFM_DIR}/merged.bundle.out --output-file ${SFM_DIR}/final.mvs
${OPENMVS_DIR}/ReconstructMesh ${SFM_DIR}/final.mvs --mesh-file ${SFM_DIR}/mesh_file.ply --smooth 0 --working-folder ${SFM_DIR}

# ----------------------------------------------------------------------

# Mesh texturing for computing a sharp and accurate texture to color the mesh
${OPENMVS_DIR}/TextureMesh ${SFM_DIR}/final_mesh.mvs --patch-packing-heuristic 0 --cost-smoothness-ratio 1 --empty-color 16744231 --working-folder ${SFM_DIR}

rm ${SFM_DIR}/*.log

# ----------------------------------------------------------------------

# Copy result to current dir
cp ${SFM_DIR}/final_mesh_texture.png ${CUR_DIR}/final_mesh_texture.png
cp ${SFM_DIR}/final_mesh_texture.ply ${CUR_DIR}/final_mesh_texture.ply

cd ${CUR_DIR}

# Convert to OBJ
eval ${MESHLABSERVER} -i final_mesh_texture.ply -o final_mesh_texture.obj -m vn wt 2> /dev/null
mv final_mesh_texture.obj.mtl final_mesh_texture.mtl
sed -i 's/final_mesh_texture.obj.mtl/final_mesh_texture.mtl/' final_mesh_texture.obj

# ----------------------------------------------------------------------

# LC_ALL=C meshlab ${SFM_DIR}/final_mesh_texture.ply &> /dev/null
