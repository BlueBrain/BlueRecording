import sys
from bluerecording.getPositions import getPositions

if __name__=='__main__':

    path_to_simconfig = sys.argv[1] #simulation_config with one-timestep simulation outputting a compartment report

    path_to_positions_folder = sys.argv[2]
    
    neurons_per_file = int(sys.argv[3])

    files_per_folder = int(sys.argv[4]) # Number of pickle files to write to each subfolder
    
    if len(sys.argv)>5:
        replace_axons = sys.argv[5]
    else:
        replace_axons = True

    getPositions(path_to_simconfig, neurons_per_file, files_per_folder, path_to_positions_folder,replace_axons)
