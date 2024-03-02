import sys
from bluerecording.writeH5_prelim import initializeH5File

if __name__=='__main__':

    '''
    path_to_simconfig refers to the simulation_config from the 1-timestep simulation used to get the segment positions
    electrode_csv is a csv file containing the position, region, and layer of each electrode
    type is either LineSource or Reciprocity
    '''

    electrode_csv = sys.argv[1]

    path_to_simconfig = sys.argv[2]

    outputfile = sys.argv[3]

    initializeH5File(path_to_simconfig,outputfile,electrode_csv)
