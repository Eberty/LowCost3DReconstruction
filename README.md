# 3D reconstruction pipeline from depth and color cameras

## **Overview**

This project focus on low cost methodologies for 3D reconstruction of objects. We use state-of-the-art libraries and the main elements that a 3D reconstruction pipeline should include, from the acquisition of depth and color images, alignment of captures and mesh generation, down to the texturing and realistic visualization step.

It was developed a pipeline composed of a hybrid 3D reconstruction approach, combining the low resolution images of the Kinect sensor depth camera with the high resolution images of an external RGB camera, obtain three-dimensional models with considerable visual quality.

**Abstract keywords:** Low Cost 3D Reconstruction, Depth Sensor, Photogrammetry.<br />
**Keywords:** C++, Structure from Motion (SFM), Point Cloud Library (PCL), Kinect, Bash Scripts.

<br />

### **License**

The source code is released under a [MIT](https://github.com/Eberty/msc-research/blob/master/LICENSE) license.

**Author:** Eberty Alves da Silva<br />
**Affiliation:** Universidade Federal da Bahia - UFBA<br />
**Maintainer:** Eberty Alves da Silva, <eberty.silva@hotmail.com>

The msc-research package has been tested under *Ubuntu 16.04 LTS* and *Ubuntu 19.10*.

<br />

## **Installation**

#### Dependencies

First, on your terminal, update the package manager indexes:

`sudo apt update`

Then install git:

`sudo apt install git -y`

After installing git, you can clone this git repository: `git clone https://github.com/Eberty/msc-research.git`.

Within the *install* folder, there are several bash files. They are responsible for installing all the packages necessary for the execution of this pipeline:

1. [tools_install.sh](https://github.com/Eberty/msc-research/blob/master/install/tools_install.sh)
    * To install packages that can be installed through the official ubuntu repositories
2. [pcl_install.sh](https://github.com/Eberty/msc-research/blob/master/install/pcl_install.sh)
    * To install the Point Cloud Library (PCL)
3. [libfreenect2_install.sh](https://github.com/Eberty/msc-research/blob/master/install/libfreenect2_install.sh)
    * To install driver for Kinect v2
4. [colmap_install.sh](https://github.com/Eberty/msc-research/blob/master/install/colmap_install.sh)
    * To install the Structure-from-Motion (SfM) package
5. [openmvs_install.sh](https://github.com/Eberty/msc-research/blob/master/install/openmvs_install.sh)
    * To install the Multi-View Stereo (MVS) package
6. [super4pcs_install.sh](https://github.com/Eberty/msc-research/blob/master/install/super4pcs_install.sh)
    * To install a set of C++ libraries for 3D Global Registration

Run `source <install_file.sh>` in your terminal to install each required dependency.

To make your life even easier, we pack everything in one place and you can just run:

```sh
source run_all_install.sh
```

<br />

**NOTE:** in particular, for CUDA, we did not create a bash to install it. To do this, you can follow some of these tutorials:

* <https://www.pugetsystems.com/labs/hpc/How-to-install-CUDA-9-2-on-Ubuntu-18-04-1184/>
* <https://askubuntu.com/questions/799184/how-can-i-install-cuda-on-ubuntu-16-04>

If your PC has an NVIDIA graphics card, we recommend installing CUDA (right after running *tools_install.sh*).

#### Building

After all dependencies are installed, run the following commands:

```sh
git clone https://github.com/Eberty/msc-research.git
cd msc-research/
mkdir build && cd build
cmake ..
make -j$(nproc) && sudo make install
```

## **Usage**

This project involves a low cost 3D reconstruction process that presents a variation of generic pipelines methodology suggested by previous works, aiming to improve the final quality of the model and the automation of the process.

We developed hybrid methodology composed of: capturing images of depth and color; generation of point clouds from Kinect depth images; capture alignment and estimation of camera positions for high definition images; mesh generation; texturing with high quality photos resulting in the final 3D model of the object of interest.

The methodology described here was divided into four files developed in bash script, responsible for making calls to various image and point clouds manipulation programs, in order to generate a good 3D model. The execution files needed to achieve our goals are described here:

1. [capture.sh](https://github.com/Eberty/msc-research/blob/master/scripts/capture.sh)
    * To obtain depth images, the Kinect sensor (version 1 or 2) can be used. Depth images should be recorded so that the next one gradually increases the previous one, until a complete cycle is performed on the object of interest. Top and bottom view images can be captured independently.
    * The meshes produced by kinect version one and two will be the same names. They will differ only by the accuracy (type) of channels in the generated depth images.
    * **Copy this script into one workspace folder and execute: `source capture.sh <object_name> <kinect_version=1|2>`**
    * The name of the object of interest is the first argument of this script, the second argument indicates the version of kinect used to make depth captures.
    * When the program is running, it is important to remember: The numbers **2** and **8** configures the captures for the bottom and top views respectively. The other numbers when pressed sets the normal capture method. Run `/usr/local/msc-research/depth_capture -h` to more information.

    <br />

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

2. [alignment.sh](https://github.com/Eberty/msc-research/blob/master/scripts/alignment.sh)
    * This script applies a rigid transformation matrix aligning the clouds obtained with the depth sensor, in a three-step process: definition of characteristics of the point clouds, simple alignment and fine registration.
    * It is easy to infer that the method will present results proportional to the better the captures by the device, that is, the lower the incidence of noise and the better the accuracy of the inferred depth. With this in mind, the depth images obtained can go through a filtering step with the application of super-resolution techniques (**sr**).
    * **Copy this script into the same workspace folder of *capture script* and execute: `source alignment.sh <object_name> <num_of_captures> [sr <kinect_version=1|2>]`**
    * The name of the object of interest is used as the first argument of this script. The second argument receives the number of captures. `sr` is optional and should be followed by kinect version used. **Note**: The number of captures does not include bottom and top views (if defined and taken manually).

3. [sfm.sh](https://github.com/Eberty/msc-research/blob/master/scripts/sfm.sh)
    * This script calls a general-purpose Structure-from-Motion (SfM) and Multi-View Stereo (MVS) pipeline with command-line interface, offering a wide range of features for reconstruction with unordered image collections.
    * Based on: <https://peterfalkingham.com/2018/04/01/colmap-openmvs-scripts-updated/>.
    * **Copy this script into the same workspace folder of the others scripts. Make sure to create a subfolder with `images` for SFM pipeline and execute: `source sfm.sh <use_gpu=true|false> [dense]`**
    * If no CUDA enabled device is available, you can manually select to use CPU-based feature extraction and matching by setting the `use_gpu` option to false.
    * Use `dense` option to do a dense point cloud reconstruction in order to obtain a complete and accurate point cloud possible.

4. [integration.sh](https://github.com/Eberty/msc-research/blob/master/scripts/integration.sh)
    * The integration consists of two stages: a) alignment between the point cloud produced with the aid of Kinect with the point cloud resulting from the SFM pipeline; b) reconstruction of the object's surface and  texturing.
    * In our pipeline, the high resolution photos taken with a digital camera with the poses calculated using SFM, will be used to perform the texturing of the model.
    * **Copy this script into the same workspace folder and execute: `source integration.sh <mesh_file.ply> [dense]`**
    * `mesh_file.ply` is the file generated by alignment process, whose name is similar to **object_name.ply**.
    * Add `dense` option to use the dense point-cloud reconstruction obtained by SFM script.

---

**In short**, you can run the pipeline with:

```sh
source capture.sh <object_name> <kinect_version=1|2>
source alignment.sh <object_name> <num_of_captures> [sr <kinect_version=1|2>]
source sfm.sh <use_gpu=true|false> [dense]
source integration.sh <mesh_file.ply> [dense]
```

* **Feel free to change sh files in order to get best results.**

---

**Example of running:**

```sh
source capture.sh museum_artifact 1
source alignment.sh museum_artifact 30 sr 1
source sfm.sh true dense
source integration.sh museum_artifact.ply dense
```

or

```sh
source capture.sh object 2
source alignment.sh object 44
source sfm.sh cpu
source integration.sh object.ply
```

You can also run all the executables generated by this project separately as well. They will all be installed in the folder `/usr/local/msc-research`. Use `./<binary_file> -h` to see the options acepted by each program.

## **See also**

The images with respective poses used by the SFM system will not be able to apply texturing to the bottom of the object. To omit this effect, the [model_view](https://github.com/Eberty/model_view) program mainly aims to post-apply a second texture with a photo of the bottom view of one object.

## **Bugs & Feature Requests**

Please report bugs and request features using the [Issue Tracker](https://github.com/Eberty/msc-research/issues).
