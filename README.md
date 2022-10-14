# 3D reconstruction pipeline from depth and color cameras

**NOTE:** this repository is in development. Any suggestion or improvement please contact us. Feel free to use it and collaborate.

## **Overview**

This project proposes a low cost 3D reconstruction pipeline using point clouds from different sensors, combining captures of a low-cost depth sensor post-processed by Super-Resolution (SR) techniques with high-resolution RGB images from an external camera using structure-from-motion (SFM) and multi-view-stereo (MVS) output data.

Several phases of the 3D reconstruction pipeline were also specialized to improve the model's visual quality: from the acquisition of depth and color images, alignment of captures and mesh generation, down to the texturing and realistic visualization step.

**Abstract keywords:** Low Cost 3D Reconstruction, Depth Sensor, Photogrammetry.\
**Keywords:** C++, Structure from Motion (SFM), Point Cloud Library (PCL), Kinect, Bash Scripts.

&nbsp;

### **License**

The source code is released under the [MIT](https://github.com/Eberty/LowCost3DReconstruction/blob/master/LICENSE) license.

**Author:** Eberty Alves da Silva\
**Affiliation:** Universidade Federal da Bahia - UFBA\
**Maintainer:** Eberty Alves da Silva, <eberty.silva@hotmail.com>

The `LowCost3DReconstruction` package has been tested under *Ubuntu 16.04 LTS*.

&nbsp;

### **Publications**

**Paper:** https://doi.org/10.5335/rbca.v14i2.13045 (english)\
**Master's thesis:** https://repositorio.ufba.br/handle/ri/36086 (portuguease)

&nbsp;

## **Building and Installation**

Within the project *install* folder, there are some bash files. They are responsible for installing the dependencies necessary for this pipeline execution:

1. [tools_install.sh](https://github.com/Eberty/LowCost3DReconstruction/blob/master/install/tools_install.sh)
    * To install packages that can be installed through the official Ubuntu repositories and other packages like libfreenect, colmap, OpenMVS and Super4PCS. This file also contains the installation of this `LowCost3DReconstruction` package itself.
2. [cuda_install.sh](https://github.com/Eberty/LowCost3DReconstruction/blob/master/install/cuda_install.sh)
    * To install the CUDA Toolkit (Works on Ubuntu 18.04 and 20.04 - For older Ubuntu versions, replace all occurrences of `1804` by `1604` in the file).

Run `source <install_file.sh>` on your terminal to install all necessary dependencies and files.

&nbsp;

**NOTE:** It's recommended to install CUDA before running *tools_install.sh* if your PC has a NVIDIA graphics card.

## **Usage**

This project involves a low cost 3D reconstruction process that presents a variation of generic pipelines methodology suggested by previous works, aiming to improve the final quality of the model and the automation of the process.

A hybrid methodology have been developed, which is composed of: capturing images of depth and color; generation of point clouds from Kinect depth images; capture alignment and estimation of camera positions for high definition images; mesh generation; texturing with high quality photos resulting in the final 3D model of the object of interest.

The methodology described here is divided into four bash script files, responsible for making calls to various image and point clouds manipulation programs in order to generate a good 3D model.

1. [capture.sh](https://github.com/Eberty/LowCost3DReconstruction/blob/master/scripts/capture.sh)
    * To obtain depth images, the Kinect sensor (version 1 or 2) can be used. Depth images should be recorded so that the next one gradually increases the previous one until a complete cycle is performed on the object of interest. Top and bottom view images can be captured independently.
    * The meshes produced by kinect version one or two will have the same names. They will differ only by the accuracy (type) of channels in the generated depth images.
    * **Copy this script into a workspace folder and execute: `source capture.sh <object_name> <kinect_version=1|2>`**
    * The name of the object of interest is the first argument of this script, the second one indicates the kinect version used to make depth captures.
    * When the program is running, it is important to remember: The numbers **2** and **8** configures the captures from the bottom and top views respectively. The other numbers when selected sets the normal capture method. Run `/usr/local/LowCost3DReconstruction/depth_capture -h` to have more information.

    &nbsp;

    ```sh
        Image capture keys:
                    Depth      d
                    Color      c
                    Burst      b
                    Depth mesh m
        Perform all captures:  a
        View type:             Top(8), Front(*), Bottom(2)
        Close the application: esc
    ```

2. [alignment.sh](https://github.com/Eberty/LowCost3DReconstruction/blob/master/scripts/alignment.sh)
    * This script applies a rigid transformation matrix aligning the clouds obtained with the depth sensor, in a three-step process: definition of characteristics of the point clouds, simple alignment and fine registration.
    * It is easy to infer that the method will present results proportional to the better the captures by the device, that is, the lower the incidence of noise and the better the accuracy of the inferred depth. With this in mind, the depth images obtained can go through a filtering step with the application of super-resolution techniques (**sr**).
    * **Copy this script into the same workspace folder of *capture script* and execute: `source alignment.sh <object_name> <num_of_captures> [sr <kinect_version=1|2>]`**
    * The name of the object of interest is used as the first argument of this script. The second argument receives the number of captures. `sr` is optional and should be followed by kinect version used. **Note**: The number of captures does not include bottom and top views (if defined and taken manually).

3. [sfm.sh](https://github.com/Eberty/LowCost3DReconstruction/blob/master/scripts/sfm.sh)
    * This script calls a general-purpose Structure-from-Motion (SfM) and Multi-View Stereo (MVS) pipeline with command-line interface, offering a wide range of features for reconstruction with unordered image collections.
    * Based on: <https://peterfalkingham.com/2018/04/01/colmap-openmvs-scripts-updated/>.
    * **Copy this script into the same workspace folder of the others scripts. Make sure to create a subfolder with `images` for SFM pipeline and execute: `source sfm.sh`**
    * This script can be run independently of the previous scripts, feel free to run it while performing the capture and alignment steps mentioned above.
    * If no CUDA enabled device is available the SFM `use_gpu` option is set to false.

4. [integration.sh](https://github.com/Eberty/LowCost3DReconstruction/blob/master/scripts/integration.sh)
    * The integration consists in two stages: a) alignment between the point cloud produced by Kinect and the point cloud resulted from the SFM pipeline; b) reconstruction of the object's surface and texturing.
    * In our pipeline, the high resolution photos taken with a digital camera with the poses calculated using SFM will be used to perform the texturing of the model.
    * **Copy this script into the same workspace folder and execute: `source integration.sh <object_name.ply>`**
    * `object_name.ply` is the file generated by alignment process.
    * For comparison purposes, three meshes will be generated: the one produced with the aligned depth sensor data, the one generated by the SFM + MVS pipeline and the merged model containing the merging of data from these different capture methods. The merged model is the expected result of our pipeline.

---

**In short**, you can run the pipeline with:

```sh
source capture.sh <object_name> <kinect_version=1|2>
source alignment.sh <object_name> <num_of_captures> [sr <kinect_version=1|2>]
source sfm.sh
source integration.sh <object_name.ply>
```

* **Feel free to change sh files in order to get best results.**

---

**Example of running:**

```sh
source capture.sh museum_artifact 1
source alignment.sh museum_artifact 30 sr 1
source sfm.sh
source integration.sh museum_artifact.ply
```

or

```sh
source capture.sh object 2
source alignment.sh object 44
source sfm.sh
source integration.sh object.ply
```

You can also run all the executables generated by this project separately as well. They will all be installed in the folder `/usr/local/LowCost3DReconstruction`. Use `./<binary_file> -h` to see the options accepted by each program.

## **Texturization of bottom view**

The images with respective poses used by the SFM system will not be able to apply a texture on the bottom of the object as this view will be hidden when object is not on air. To omit this effect, you can generate (post-apply) a new texture, using the SFM output and a photo of the bottom of the object.

To produce a file with the new cameras (raster) for this post-apply is recommended to use the latest meshlab version already included as AppImage in this package.

You can produce the auxiliary file with the configuration of the new cameras in the following way:

1. Open the latest meshlab version typing on terminal: `/usr/local/LowCost3DReconstruction/MeshLab2020.12-linux.AppImage`
2. Select **File** > **Import mesh** > "Choose the **merged_mesh_texture.ply** file"
3. Select **File** > **Import Raster...** > "Choose a image file"
4. Select **Show Current Raster Mode** on menu
    * Press **Ctrl + H** to start from a initial point of view
    * You can use **Mouse right button** to rotate, **Ctrl + Mouse right button** to move and **Shift + Mouse right button** to scale
    * Find a good alignment of an image with respect to the 3D model
    * Select **Filters** > **Camera** > **Image alignment: Mutual Information** > Click on **Get shot** and **Apply** (Do this more than once for best results)
    * If in doubt, see: [Raster Layers: Set Raster Camera](https://www.youtube.com/watch?v=298OJABhkYs) and [Color Projection: Mutual Information, Basic](https://www.youtube.com/watch?v=Pv6_qFIr7gs)
5. Select **Filters** > **Camera** > **Set Raster Camera** > Click on **Get shot** and **Apply**
6. Select **Filters** > **Raster Layer** > **Export active rasters cameras to file** > "Choose a name, set the output format to **Bundler (.out)** and save using **Apply**"

Finally, run

```sh
source bottom_view.sh <meshlab_bundler.out> <raster_image_files>
```

**Note:** This script uses files generated after the entire pipeline has been executed. Therefore, it must be executed from the same root folder as the other scripts.

## **Dataset example**

You can download our ["horse" dataset](https://github.com/Eberty/LowCost3DReconstruction/releases/download/v1.0/dataset_horse_kv1.zip) to test the pipeline.

## **Bugs & Feature Requests**

Please report bugs and request features using the [Issue Tracker](https://github.com/Eberty/LowCost3DReconstruction/issues).
