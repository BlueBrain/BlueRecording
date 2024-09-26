# Whisker flick simulations

This repo contains code used to create Figure 7 in the BlueRecording paper. It compares the EEG produced by a whisker flick stimulus in the original circuit and in an identical circuit with no cortico-cortical connectivity. It relies on the same data as the examples in the *SSCx* folder. After downloading that data as described in the Readme file in the *SSCx* folder, you will be able to run the simulations as per the following instructions:

1. Run *data/simulation/launch.sh*  to produce a one-timestep compartment report
2. Run *data/getPositions/GetPositions.sh* to interpolate segment positions
3. Run *electrodeFile/WriteH5Prelim.sh* to initialize the weights file
4. Run *electrodeFile/WriteH5.sh* to calculate the segment weights
5. Run *launch.sh* in each of the subfolders in the *whiskerFlickSim/original* and *whiskerFlickSim/disconnected* folders. Each of these runs a trial of the whisker flick simulation, in the connected and disconnectd circuits, respectively and produces the EEG report.
6. Run *Analysis/Geteeg.sh* and *Analysis/Geteeg_Disconnected.sh* to load the simulaiton results and save them in pickle files grouped by region
7. Run *Analysis/Figures_For_BlueRecording_Paper.ipynb* to produce Figure 7
