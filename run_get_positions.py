import sys
from bluerecording.getPositions import getPositions

if __name__=='__main__':

    path_to_simconfig = sys.argv[1] #simulation_condif with one-timestep simulation outputting a compartment report

    path_to_positions_folder = sys.argv[2]

    chunk_size = int(sys.argv[3]) # Number of pickle files to write to each subfolder
    
    if len(sys.argv)>4:
        replace_axons = sys.argv[4]
    else:
        replace_axons = True

    getPositions(path_to_simconfig, chunk_size, path_to_positions_folder,replace_axons)
