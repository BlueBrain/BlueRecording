from bluerecording import *


makePositionFolders(path_to_simconfig,chunk_size,path_to_positions_folder)

getPositions(path_to_simconfig, chunk_size, path_to_positions_folder,replace_axons=True)

writeH5File(path_to_simconfig,outputfile,electrode_csv)



if __name__=='__main__':
    
    pass
