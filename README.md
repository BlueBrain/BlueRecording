# create_lfp_weights_for_neurodamus

First, run getPositions.py to interpolate the segment positions.
Second, if necessary, run writeCSV.py to convert the MEAUtility/ProbeInterface object to a csv file.
Third, run writeH5_prelim.py to generate the coefficients file. 
Finally, run writeHy.py to calculate the segment coefficients. Note that this requires h5py to be compiled with MPI support
