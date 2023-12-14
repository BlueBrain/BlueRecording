# create_lfp_weights_for_neurodamus

This repository contains scripts to porduce an input file (also refered to as an electrodes file or a weights file) for the calculation of extracellular signals in [neurodamus](https://github.com/BlueBrain/neurodamus). Complete documentation for this calculation can be found [here](https://github.com/BlueBrain/neurodamus/blob/main/docs/online-lfp.rst](https://github.com/BlueBrain/neurodamus/tree/main/docs). 

The root folder provides scripts for the production of an electrodes file compatible with BluleConfig based simulations. The SONATA folder provides scripts that produces ane electrodes file compatible with the [SONATA format](https://github.com/BlueBrain/sonata-extension/blob/master/source/sonata_tech.rst#format-of-the-electrodes_file) 

## User instructions

1. Produce a compartment report from a target including the cells that will be used for the LFP calculation. Instructions for this step are found [here](https://github.com/BlueBrain/neurodamus/blob/main/docs/online-lfp.rst)
2. Create a csv file containing information about the electrodes. Each row of the file contains information about one electrode contact. The scripts writeEEGToCSV.py and writeNeuropixelsToCSV.py are provided to create this file for a two-contact EEG system and a Neuropixels probe, respectively. The altter can be launched with WriteNP2CSV.sh. The format of the csv file is defined as follows:
   - The header is *,x,y,z,layer,region*
   - The first column is the name of the electrode contact. It is either a string or an integer
   - The second through fourth columns are the x, y, and z coordinates of the contact in Cartesian space. They are floats.
   - The fifth column is the cortical layer in which the electrode is located. It is a string in the format L*N*, where *N* is an integer.
       + If the electrode is outside of the brain, the value in the column is the strign *Outside*
       + If the electrode is in a region without laminar oraginzation, the value in the column is the string *NA*
   - The sixth column is the brain region in which the electrode is located. It is a string.
       + If the electrode is outside the brain, the value in the column is the strong *Outside* 
3. Run the file getPositions.py, using the bash script GetPositions.sh. This loads the compartment report produced in step 1, and will create a folder containing pickle files listing the (x,y,z) position of each segment in each cell in the target.
4. Run the file writeH5_MPI_prelim.py, using the bash script WriteH5Prelim.sh. This loads the compartment report produced in step 1 and the csv file produced in step 2, and will create the electrodes file, populating all coefficients with 1s.
5. Run the file writeH5_MPI.py, using the bash script WriteH5.sh. This loads the position files created in step 3 and the electrode file created in step 4, populates the electrode file with the correct coefficients. This step requires the use of a version of h5py built with MPI support. This two-step procedure is used because the calculation of the LFP coefficients is not feasible without parallelization, but MPI cannot be used when H5 files are created.

