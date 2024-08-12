# Examples

The notebook ComputationalComparison.ipynb estimates the time that would be required by LFPy and Bionet to run a simulation the size of the SSCx run with BlueRecording, assuming an equal number of cores.

The folder *scripts* contains python scripts that run the following steps of the BlueRecording pipeline: Calculating segment positions, initializing the weights file, and computing the weights. The scripts in this folder are called by bash scripts in each of the example folders (see the ReadMe files in the corresponding repos for instructions)

The folder *compare_to_reference_electrodes* is an example of BlueRecording's ability to calculate extracellular signals from a variety of recording modalities. It contains all of the code necessary to produce Figure 2 from the paper "BlueRecording: A Pipeline for the efficient calculation of extracellular recordings in large-scale neural circuit models". Some of the data required to run the examples must be downloaded from our Zenodo repository at 10.5281/zenodo.10927050

The folder *circuitTest* is an example of performing extracellular recordings fron a network of 100 cells. Some of the data required to run the examples must be downloaded from our Zenodo repository at 10.5281/zenodo.10927050

The folder *SSCx* is an example of BlueRecording's ability to produce signals from a large (~4.2M neurons) neural circuits. It contains the code to reproduce Figures 3-5 in the aforementioned paper. The data requried to run the simulations in that folder must be obtained from the authors.

The folder *hippocampus* is an example of BlueRecording's ability to produce signals from a different large (~456000 neurons) neural circuit. It contains the code to reproduce Figure 6 in the aforementioned paper. The data requried to run the simulations in that folder must be obtained from the authors.

The folder *makeCsvFile* contains an example of how to use BlueRecording to produce an electrode csv file for a Neuropixels probe aligned to the central column in the BBP SSCx model. This is not referenced in the paper. This example also requires the used in the *circuitTest* example

## Important note
Before running the examples in *circuitTest* or *SSCx*, copy the file atlas.zip from 10.5281/zenodo.10927050, and save it to the subfolder *data/atlas* in this directory 

## Another important note
The simulation_config.json and circuit_config.json files used in the examples contain relative paths to other files. These must be changed to absolute paths, or BlueRecording will throw errors.
