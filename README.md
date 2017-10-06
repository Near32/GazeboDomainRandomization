# GazeboDomainRandom

This is an implementation of some Domain Randomization tools within the [ROS](https://www.ros.rg)+[Gazebo](https://gazebosim.org) framework, following the work of Tobin et al. ["Domain  Randomization  for  Transferring  Deep  Neural  Networks  from Simulation  to  the  Real  Worl"](https://arxiv.org/abs/1703.06907).

It can be used to generate virtual datasets for an object recognition task of your choice, as it will automatically generate the bounding boxes for the object we seek to recognize in every generated pictures. The object has to be rendered in a **.dae** file compatible with **Gazebo**, first.

## Requirements

This has been developped and tested using [ROS Indigo](http://wiki.ros.org/indigo/Installation) and [Gazebo v2.2.6](http://gazebosim.org/download).

## Installation :

### Install the 3D model :

Once the object you seek to recognize in real applications has been modeled in 3D, in a __dae__ file, you need to place that file as follows : **./models/GazeboDomainRandom/models/robot/mesh.dae**.

You will now need to install all the required models so that **ROS** and **Gazebo** will be able to find them.

### Install all the required model dependencies :

To install the model dependencies for **ROS** and **Gazebo** to find them, you need to execute :
```bash
./install_models.sh
```
It will install the models into the hidden folder _.gazebo_ of your home directory.

## Generate a Dataset :

In order to generate a dataset, execute :
```bash
python createDataset_auto.py
```
The following command keys apply :
* __n__ : create a new scene with random objects and colors. You will need to wait 5 seconds for the unloading and loading of the objects.
* __c__ : change the orientation of the objects that are currently in the scene.
* __v__ : change the position of the objects that are currently in the scene.
* __p__ : save the current picture in the _./src/dataset_test/images/_ folder and create an XML annotation file with the bounding boxes for the object we seek to recognize in the _./src/dataset_test/annotations/_ folder.

## Development Status

This work is still a work in progress with many flaws and every contributions and/or advices are welcome :).
