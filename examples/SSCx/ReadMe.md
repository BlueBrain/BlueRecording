# SSCx simulations

This repo contains code used to create figures 3-5 in the BlueRecording paper. Unfortunately, the data required to run the neural simulations is too large to be shared generally. If you are interested in reproducing these figures, please contact the author. You will be provided with files that should be placed in the *data/simulation/configuration* folder. You will then be able to run the simulations as per the following instructions:

1. Run *data/simulation/launch.sh* to produce a one-timestep compartment report
2. Run *data/getPositions/GetPositions.sh* to interpolate segment positions
3. Run *electrodeFile/WriteH5Prelim.sh* to initialize the weights file
4. Run *electrodeFile/WriteH5.sh* to calculate the segment weights
5. Run *sscxSimulaton/launch.sh* to run the full simulation of SSCx, with extracellular recordings
6. Run *Analysis/GetEEG.sh* and *Analysis/GetSpikes.sh* to load the simulaiton results and save them in pickle files grouped by region
7. Run *Analysis/MakeMovie.ipynb* to produce Figure 3B and Supplementary Video 1.
8. Run *Analysis/3danalysis.ipynb* to produce Figure 3C, Figure 4, and Figure 5
