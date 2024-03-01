import numpy as np
import bluepysnap as bp
from .utils import getSimulationInfo
import sys
import os

def makeFolders(path_to_simconfig,chunk_size,path_to_positions_folder):
    
    _, _, _, ids, _ = getSimulationInfo(path_to_simconfig)
    
    numFiles = np.ceil(num_nodes/1000)
    
    numFolders = np.ceil(numFiles/chunk_size)

    for f in range(numFolders):

        os.mkdir(os.join(path_to_positions_folder, str(f)))
        
if __name__ == '__main__':
    
    path_to_simconfig = sys.argv[1]
    chunk_size = sys.argv[2]
    path_to_positions_folder = sys.argv[3]
    
    makeFolders(path_to_simconfig,chunk_size,path_to_positions_folder)
