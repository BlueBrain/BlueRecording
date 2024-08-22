# Hippocampal simulations

This repo contains code used to create figures 6 in the BlueRecording paper. Unfortunately, the data required to run the neural simulations is too large to be shared generally. If you are interested in reproducing these figures, please contact the author. You will be provided with files that should be placed in the *data/simulation/configuration* folder. You will then be able to run the simulations as per the following instructions:

1. Run *data/simulation/launch_small.sh* and *data/simulation/launch_big.sh* to produce a one-timestep compartment report
2. Run *data/getPositions/GetPositions_Small.sh* and *data/getPositions/GetPositions_Big.sh* to interpolate segment positions
3. Run *electrodeFile/WriteH5Prelim_Small.sh* and *electrodeFile/WriteH5Prelim_Big.sh* to initialize the weights file
4. Run *electrodeFile/WriteH5_Small.sh* and *electrodeFile/WriteH5_Big.sh* to calculate the segment weights
5. Run *hippocampusSimulaton/launch_small.sh* and *hippocampusSimulaton/launch_big.sh* to run the full simulation of hippocampus, with extracellular recordings
6. Run *Analysis/GetEEG_Small.sh* and *Analysis/GetEEG_Big.sh* to load the simulaiton results and save them in pickle files grouped by region
7. Run *Analysis/plotSignals.ipynb* to produce Figure 6
