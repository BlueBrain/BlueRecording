# BlueRecording

BlueRecording is used to produce an input file (also refered to as an electrodes file or a weights file) for the calculation of extracellular signals in [neurodamus](https://github.com/BlueBrain/neurodamus). 

This branch provides code that produces an electrodes file compatible with the [SONATA format](https://github.com/BlueBrain/sonata-extension/blob/master/source/sonata_tech.rst#format-of-the-electrodes_file). For scripts to produce an electrode file compatible with the old BlueConfig format, see the *non-sonata* branch of this repo. 

# User instructions

## System requirements

Our documentation and examples assume that you are running BlueRecording on a Linux system with slurm and the spack package manager. BlueRecording has not been tested on any other system. 

## Dependencies

Bluerecording requires mpi4py, h5py, and hdf5 built with MPI support. These should be installed with spack, as per the instructions in the following section. Bluerecording also depends on several other python packages, which are automatically installed with setuptools when Bluerecording is installed.

Running neural simulations to generate inputs to BlueRecording (and to calculate extracellular signals using weights files produced by BlueRecording) requires [neurodamus](https://github.com/BlueBrain/neurodamus), the BBP Simulation Control application for Neuron, [neurodamus-models](https://github.com/BlueBrain/neurodamus-models), which contains mod files for neural mechanisms. These should be installed in a separate spack environment. The neural mechanisms vary based on the circuit simulated (in the paper and the provided examples, we simulate the neocortex and the thalamus), so the installation instructions are slightly different in each case.

## Installation

### BlueRecording

We recommend using a combimation of a spack environment and a `virtulenvironment` to install BlueRecording and its dependencies

First create a spack environment, which is used to satisfy the h5py+mpi and mpi4py dependencies 

```
spack env create bluerecording-dev
spack env activate -p bluerecording-dev
spack install --add py-h5py+mpi 
spack install --add py-mpi4py
```

Then install bluerecording in a `virtualenv`:

```
git clone https://github.com/BlueBrain/BlueRecording.git
cd BlueRecording
python -m venv bluerecording-dev
source bluerecording-dev/bin/activate
pip install -e .
```

### Neurodamus

We recommend installing Neurodamus and the Neurodamus-Models mechanisms in a separate spack environment. Separate environments should be used for the cortex and hippocampus packages.

To install Neurodamus for the neocortex, first run 

```
git clone https://github.com/BlueBrain/spack.git
. spack/share/spack/setup-env.sh
cd spack
spack env create neurodamus
spack env activate -p neurodamus
spack install --add neurodamus-models@develop+coreneuron
```
Then, create modules for neurodamus and its dependencies. To do so, make sure that `py-neurodamus`,`neurodamus-models`,and `neuron` are included in your `~/.spack/modules.yaml` file. An example is provided [here](https://github.com/BlueBrain/BlueRecording/blob/main/modules.yaml). Then, run 
```
spack module tcl refresh
module use $SPACK_INSTALL_PREFIX/modules/linux-rhel7-skylake
```
The second of the two lines above must be run every time you begin a new terminal session. 

To intall Neorodamus for the hippocampus, follow the same steps as for the neocortex, except that the spack commands should be:
```
spack env create hippocampus
spack env activate -p hippocampus
spack install --add neurodamus-models@develop+coreneuron model=hippocampus
```

Neurodamus-models expects that you have modules available on your system for `python/3.11.6`, `intel-oneapi-mkl/2023.2.0`, and `hpe-mpi/2.27.p1.hmpt`. The launch scripts provided in the examples folder assume that these modules are in an archive called `unstable`

## Testing

You must download the folder atlas.zip from our Zenodo repository (10.5281/zenodo.10927050) and unzip it into the folder examples/data/atlas.

After following the instructions above, run `pytest tests`

## Steps to produce electrode files

1. Produce a compartment report from a target including the cells that will be used in the extracellular recording. Complete documentation for this calculation can be found [here](https://github.com/BlueBrain/neurodamus/tree/main/docs). 

2. Create a csv file containing information about the electrodes. Each row of the file contains information about one electrode contact. The format of the csv file is defined as follows:
   - The header is *name,x,y,z,layer,region,type*
   - The first column is the name of the electrode contact. It is either a string or an integer
   - The second through fourth columns are the x, y, and z coordinates of the contact in Cartesian space. They are floats.
   - The fifth column is the cortical layer in which the electrode is located. It is a string in the format L*N*, where *N* is an integer.
       + If the electrode is outside of the brain, the value in the column is the strign *Outside*
       + If the electrode is in a region without laminar oraginzation, the value in the column is the string *NA*
   - The sixth column is the brain region in which the electrode is located. It is a string.
       + If the electrode is outside the brain, the value in the column is the strong *Outside*
   -  The seventh column is the calculation method used to determine the compartment weights. Supported values are *PointSource*, *LineSource*, *Reciprocity*, *DipoleReciprocity*, and *ObjectiveCSD*. The *PointSource* and *LineSource* methods assume that the neurons are in an infinite homogeneous medium. They should be used only for recordings made inside the brain tissue. If they are used, the tissue conductivity should be provided in step 6. *Reciprocity* and *DipoleReciprocity* assign the compartment weights based on a lead-field calculated in step 3. These should be used for EEG or ECoG recordings. In general, we recommend using the *Reciprocity* method. *ObjectiveCSD* assigns a coefficient of 1 to each compartment that is within a specified distance from the electrode (by defaut 50 um) and a 0 to all other compartments. 

    The folder *examples/makeCsvFiles* contains an example python script that will generate a csv file for a Neuropixels probe.

3. If the *Reciprocity* or *DipoleReciprocity* methods are used, you must calculate a lead-field. The lead field is the potential field (for the reciprocity method) or the E-field (for the dipole reciprocity method) produced in the neural tissue by running a current of 1 nA between the recording electrode and the reference electrode. BlueRecording assumes that this field is calculated using the Sim4Lfie finite element solver and exported as an h5 file. Other calculation methods are possible, asusming the field is exported in the same format. 

4. Run the function `bluerecording.getPositions.getPositions(path_to_simconfig, neurons_per_file, files_per_folder, path_to_positions_folder,replace_axons=True)`. This loads the compartment report produced in step 1, and will create folders containing pickle files listing the (x,y,z) position of each segment in each cell in the target. The argument `neurons_per_file` refers to the number of neurons whose positions are stored in each pickle file, and `files_per_folder` refers to the number of such files in each folder (which should be adjusted based on your filesystem)

5. Run the function `bluerecording.writeH5_prelim.initializeH5File(path_to_simconfig,outputfile,electrode_csv)`. This loads the compartment report produced in step 1 and the csv file produced in step 2, and will create the electrodes file, with the name `outputfile`, populating all coefficients with 1s.

6. Run the function `bluerecording.writeH5.writeH5File(path_to_simconfig,path_to_segment_position_folder,outputfile,neurons_per_file,files_per_folder,sigma=[0.277],path_to_fields=None,radius=[50.])`. This loads the position files created in step 4 and the electrode file created in step 4, populates the electrode file with the correct coefficients. Here `sigma` is the conductivity of the tissue, and must be provided if one of the analytic methods is used. For the *ObjectiveCSD* calculation method, `radius` defines the distance from the electrode for which a value of 1 is assigned. `path_to_fields` is the path to the finite element output calculated in step 3, which must be provided if the reciprocity-based methods are used. This two-step procedure is used because the calculation of the LFP coefficients for large neural populatons is not feasible without parallelization, but MPI cannot be used when H5 files are created, since parallel writing of variable length strings is not supported.

## Running an extracellular recording simulation
Once the electrode file has been generated, it can be used in a Neurodamus simulation that includes extracellular recording. Instructions for this step are found [here](https://github.com/BlueBrain/neurodamus/blob/main/docs/online-lfp.rst)

# Examples
See [here](https://github.com/BlueBrain/BlueRecording/tree/main/examples)

# Contribution Guidelines
[Here](./CONTRIBUTING.md)


# Citation
If you use this software, we kindly ask you to cite the following publication:
[Tharayil et al. BlueRecording: A Pipeline for efficient calculation of extracellular recordings in large-scale neural circuit models. *bioRxiv, (2024)*](https://www.biorxiv.org/content/10.1101/2024.05.14.591849v1)

# Acknowledgment
The development of this software was supported by funding to the Blue Brain Project, a research center of the École polytechnique fédérale de Lausanne (EPFL), from the Swiss government's ETH Board of the Swiss Federal Institutes of Technology.
 
Copyright (c) 2023 Blue Brain Project/EPFL
