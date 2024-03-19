# BlueRecording

BlueRecording is used to produce an input file (also refered to as an electrodes file or a weights file) for the calculation of extracellular signals in [neurodamus](https://github.com/BlueBrain/neurodamus). Complete documentation for this calculation can be found [here](https://github.com/BlueBrain/neurodamus/tree/main/docs). 

This branch provides code that produces an electrodes file compatible with the [SONATA format](https://github.com/BlueBrain/sonata-extension/blob/master/source/sonata_tech.rst#format-of-the-electrodes_file). For scripts to produce an electrode file compatible with the old BlueConfig format, see the *non-sonata* branch of this repo. 

## System requirements

Our documentation and examples assume that you are running BlueRecording on a Linux system with slurm and the spack package manager. BlueRecording has not been tested on any other system. 

## Installation

BlueRecording depends on [BluePySnap](https://github.com/BlueBrain/snap/) and h5py and HDF5 built with MPI support. The provided examples depend on [Neurodamus](https://github.com/BlueBrain/neurodamus). BlueRecording and all its dependencies can be installed by running `spack install --add py-bluerecording`

## Steps to produce electrode files

1. Produce a compartment report from a target including the cells that will be used for the LFP calculation. Instructions for this step are found [here](https://github.com/BlueBrain/neurodamus/blob/main/docs/online-lfp.rst)
2. Create a csv file containing information about the electrodes. Each row of the file contains information about one electrode contact. The format of the csv file is defined as follows:
   - The header is *name,x,y,z,layer,region*
   - The first column is the name of the electrode contact. It is either a string or an integer
   - The second through fourth columns are the x, y, and z coordinates of the contact in Cartesian space. They are floats.
   - The fifth column is the cortical layer in which the electrode is located. It is a string in the format L*N*, where *N* is an integer.
       + If the electrode is outside of the brain, the value in the column is the strign *Outside*
       + If the electrode is in a region without laminar oraginzation, the value in the column is the string *NA*
   - The sixth column is the brain region in which the electrode is located. It is a string.
       + If the electrode is outside the brain, the value in the column is the strong *Outside* 

    The folder *examples/makeCsvFiles* contains an example python script that will generate a csv file for a Neuropixels probe.

3. Run the function getPositions(). This loads the compartment report produced in step 1, and will create a folder containing pickle files listing the (x,y,z) position of each segment in each cell in the target.
4. Run the function initializeH5File(). This loads the compartment report produced in step 1 and the csv file produced in step 2, and will create the electrodes file, populating all coefficients with 1s.
5. Run the file writeH5File(). This loads the position files created in step 3 and the electrode file created in step 4, populates the electrode file with the correct coefficients. This two-step procedure is used because the calculation of the LFP coefficients for large neural populatons is not feasible without parallelization, but MPI cannot be used when H5 files are created, since parallel writing of variable length strings is not supported.

## Examples
See [here](https://bbpgitlab.epfl.ch/conn/personal/tharayil/bluerecording/-/tree/main/examples?ref_type=heads)

## Contribution Guidelines
[Here](./CONTRIBUTING.md)

## Important notes about unit tests
Please note that some of the unit tests rely on the following configuration files *tests/data/simulation_config.json*, *tests/data/configuration/circuit_config.json*, *examples/compare-to-reference-solutions/data/simulation/simulation_config.json*, and *examples/compare-to-reference-solutions/data/simulation/configuration/circuit_config.json*. These configuration files contain absolute paths to files, which must be updated to match your system.  

## Citation
If you use this software, we kindly ask you to cite the following publication:
BlueRecording: A Pipeline for efficient calculation of extracellular recordings in large-scale neural circuit models

## Acknowledgment
The development of this software was supported by funding to the Blue Brain Project, a research center of the École polytechnique fédérale de Lausanne (EPFL), from the Swiss government's ETH Board of the Swiss Federal Institutes of Technology.
 
Copyright (c) 2023 Blue Brain Project/EPFL
