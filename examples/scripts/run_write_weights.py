# SPDX-License-Identifier: GPL-3.0-or-later
import sys
import pandas as pd
from bluerecording.writeH5 import writeH5File
from bluerecording.utils import process_writeH5_inputs

if __name__=='__main__':


    path_to_simconfig = sys.argv[1] # simulation_config.json with one-timestep compartment report
    segment_position_folder = sys.argv[2] # Folder with segment positions; output of getPositions.py
    outputfile = sys.argv[3]
    
    neurons_per_file = int(sys.argv[4])

    numFilesPerFolder = int(sys.argv[5]) # Number of files per subfolder in segment positions folder

    # Radius is the radius to be used for the objective csd calculation
    sigma, path_to_fields, objectiveCsdIdx = process_writeH5_inputs(sys.argv)
    
    writeH5File(path_to_simconfig,segment_position_folder,outputfile,neurons_per_file,numFilesPerFolder,sigma,path_to_fields,objectiveCsdIdx)
