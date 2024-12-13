# SSCx simulations

This repo contains code used to create figures 3-5 in the BlueRecording paper. To reproduce these figures:

1. Download the [somatosensory cortex model](https://dataverse.harvard.edu/dataset.xhtml?persistentId=doi:10.7910/DVN/HISHXN&version=1.0) and save it to the folder *data/simulation/configuration* following the instructions in the download link. Make sure to use the .asc morphology files.
2. Download the files EEG.h5, ECoG.h5, and LFP.h5 from [our Zenodo repository](https://zenodo.org/records/14419388) and copy them to the electrodeFile directory
3. Run *data/simulation/launch.sh* to produce a one-timestep compartment report
4. Run *data/getPositions/GetPositions.sh* to interpolate segment positions
5. Run *electrodeFile/WriteH5Prelim.sh* to initialize the weights file
6. Run *electrodeFile/WriteH5.sh* to calculate the segment weights
7. Run *sscxSimulaton/launch.sh* to run the full simulation of SSCx, with extracellular recordings
8. Run *Analysis/GetEEG.sh* and *Analysis/GetSpikes.sh* to load the simulaiton results and save them in pickle files grouped by region
9. Run *Analysis/MakeMovie.ipynb* to produce Figure 3B and Supplementary Video 1.
10. Run *Analysis/3danalysis.ipynb* to produce Figure 3C, Figure 4, and Figure 5
