import libsonata as lb
import numpy as np
import h5py
import pandas as pd
import sys
from mpi4py import MPI
from scipy.spatial import distance
from scipy.spatial.transform import Rotation
import json
from scipy.interpolate import RegularGridInterpolator
import time 


def add_data(h5, ids, coeffs ,population_name):

    
    
    dset = 'electrodes/'+population_name+'/scaling_factors'
    
    node_ids = h5[population_name+'/node_ids'][:]
        
    isInInput = np.isin(node_ids,ids)
    nodesInInput = node_ids[isInInput] # This contains the same values as the variable ids, but in the order as in node_ids
    idIndex = np.where(isInInput)[0]

    offset0 = h5[population_name+'/offsets'][idIndex] # Finds offset in  'electrodes/'+population_name+'/scaling_factors' for this particular node id

    offset1 = np.zeros_like(offset0)
    
    if np.any(idIndex ==  len(h5[population_name+'/offsets'][:])-1):
        
        lastNodeIdx =  np.where(idIndex == len(h5[population_name+'/offsets'][:])-1)[0]
        offset1[lastNodeIdx] = len(h5[dset][:]) # If this is the last node in the list, we write the coefficients up to the end of the coefficient array

    notLastNodeIdx = np.where(idIndex != len(h5[population_name+'/offsets'][:])-1)[0]
    offset1[notLastNodeIdx] =  h5[population_name+'/offsets'][idIndex[notLastNodeIdx]+1] # Otherwise, we write up to the offset for the next node
    
    for i, id in enumerate(nodesInInput):


        h5[dset][offset0[i]:offset1[i],:-1] = coeffs.loc[:,id].values.T

def get_line_coeffs(startPos,endPos,electrodePos,sigma):

    '''
    startPos and endPos are the starting and ending positions of the segment
    sigma is the extracellular conductivity
    '''

    ### Convert from um to m
    startPos = startPos * 1e-6
    endPos = endPos * 1e-6
    electrodePos = electrodePos * 1e-6
    ###

    segLength = np.linalg.norm(startPos-endPos) 

    x1 = electrodePos[0]-endPos[0]
    y1 = electrodePos[1]-endPos[1]
    z1 = electrodePos[2]-endPos[2]

    

    xdiff = endPos[0]-startPos[0]
    ydiff = endPos[1]-startPos[1]
    zdiff = endPos[2]-startPos[2]

    
    h = 1/segLength * (x1*xdiff + y1*ydiff + z1*zdiff)

    r2 = (electrodePos[0]-startPos[0])**2 + (electrodePos[1]-startPos[1])**2 + (electrodePos[2]-startPos[2])**2 - h**2
    r2 = np.abs(r2)
    l = h + segLength

    segCoeff = 1/(4*np.pi*sigma*segLength)*np.log(np.abs(((h**2+r2)**.5-h)/((l**2+r2)**.5-l)))

    segCoeff *= 1e-9 # Convert from nA to A

    return segCoeff

def get_coeffs_lfp(positions,columns,electrodePos,sigma):

    for i in range(len(positions.columns)-1):

        if positions.columns[i][-1]==0: # Implies that it is a soma

            somaPos = positions.iloc[:,i]

            distance = np.linalg.norm(somaPos-electrodePos)

            distance *= 1e-6 # Converts from um to m

            somaCoeff = 1/(4*np.pi*sigma*distance) # We treat the soma as a point, so the contribution at the electrode follows the formula for the potential from a point source

            somaCoeff *= 1e-9 # Converts from nA to A
            
            if i == 0:
                coeffs = somaCoeff
            else:

                coeffs = np.hstack((coeffs,somaCoeff))

        elif positions.columns[i][-1]==positions.columns[i+1][-1]: # Ensures we are not at the far end of a section

            segCoeff = get_line_coeffs(positions.iloc[:,i],positions.iloc[:,i+1],electrodePos,sigma)

            coeffs = np.hstack((coeffs,segCoeff))


    coeffs = pd.DataFrame(data=coeffs[np.newaxis,:])

    coeffs.columns = columns

    return coeffs

def get_coeffs_pointSource(positions,electrodePos,sigma):
    
    distances = np.linalg.norm(positions.values-electrodePos[:,np.newaxis],axis=0)
   
    distances *= 1e-6 # Converts from um to m

    coeffs = 1/(4*np.pi*sigma*distances)
    
    coeffs *= 1e-9 # Converts from nA to A

    coeffs = pd.DataFrame(data=coeffs[np.newaxis,:])

    coeffs.columns = positions.columns

    return coeffs 

def geth5Dataset(h5f, group_name, dataset_name):
    """
    Find and get dataset from h5 file.
    out = geth5Dataset(h5f, group_name, dataset_name)
    h5f - string - h5 file path and name
    group_name - string - where to initiate search, '/' for root
    dataset_name - string - dataset to be found
    return - numpy array
    """

    def find_dataset(name):
        """ Find first object with dataset_name anywhere in the name """
        if dataset_name in name:
            return name

    with h5py.File(h5f, 'r') as f:
        k = f[group_name].visit(find_dataset)
        return f[group_name + '/' + k][()]
    
def get_coeffs_dipoleReciprocity(positions, path_to_fields,center):

    '''
    path_to_fields is the path to the h5 file containing the potential field, outputted from Sim4Life
    path_to_positions is the path to the output from the position-finding script
    '''

    # Get new output file potential field

    with h5py.File(path_to_fields, 'r') as f:
        for i in f['FieldGroups']:
            tmp = 'FieldGroups/' + i + '/AllFields/EM E(x,y,z,f0)/_Object/Snapshots/0/'
            
        Ex = geth5Dataset(path_to_fields, tmp, 'comp0')
        Ey = geth5Dataset(path_to_fields, tmp, 'comp1')
        Ez = geth5Dataset(path_to_fields, tmp, 'comp2')
        
        for i in f['Meshes']:
            tmp = 'Meshes/'+i
            break
        x = geth5Dataset(path_to_fields, tmp, 'axis_x')
        y = geth5Dataset(path_to_fields, tmp, 'axis_y')
        z = geth5Dataset(path_to_fields, tmp, 'axis_z')


        try:
            currentApplied = f['CurrentApplied'][()] # The potential field should have a current, but if not, just assume it is 1
        except:
            currentApplied = 1


    positions *= 1e-6 # Converts um to m, to match the potential field file

    center *= 1e-6


    InterpFcnX = RegularGridInterpolator((x, y, z), Ex[:, :, :, 0], method='linear')
    InterpFcnY = RegularGridInterpolator((x, y, z), Ex[:, :, :, 0], method='linear')
    InterpFcnZ = RegularGridInterpolator((x, y, z), Ex[:, :, :, 0], method='linear')

    XComp = InterpFcnX(center)[np.newaxis]  # Interpolate E field at location of neural center
    
    YComp = InterpFcnY(center)[np.newaxis]  # Interpolate E field at location of neural center
    
    ZComp = InterpFcnZ(center)[np.newaxis]  # Interpolate E field at location of neural center
    
    out2rat = positions.values[0]*XComp + positions.values[1]*YComp + positions.values[2]*ZComp


    outdf = pd.DataFrame(data=(out2rat / currentApplied), columns=positions.columns) # Scale potential field by applied current

    return outdf

def get_coeffs_eeg(positions, path_to_fields):

    '''
    path_to_fields is the path to the h5 file containing the potential field, outputted from Sim4Life
    path_to_positions is the path to the output from the position-finding script
    '''

    # Get new output file potential field

    with h5py.File(path_to_fields, 'r') as f:
        for i in f['FieldGroups']:
            tmp = 'FieldGroups/' + i + '/AllFields/EM Potential(x,y,z,f0)/_Object/Snapshots/0/'
        pot = geth5Dataset(path_to_fields, tmp, 'comp0')
        for i in f['Meshes']:
            tmp = 'Meshes/'+i
            break
        x = geth5Dataset(path_to_fields, tmp, 'axis_x')
        y = geth5Dataset(path_to_fields, tmp, 'axis_y')
        z = geth5Dataset(path_to_fields, tmp, 'axis_z')


        try:
            currentApplied = f['CurrentApplied'][()] # The potential field should have a current, but if not, just assume it is 1
        except:
            currentApplied = 1


    positions *= 1e-6 # Converts um to m, to match the potential field file

    xSelect = positions.values[0]
    ySelect = positions.values[1]
    zSelect = positions.values[2]


    selections = np.array([xSelect, ySelect, zSelect]).T


    InterpFcn = RegularGridInterpolator((x, y, z), pot[:, :, :, 0], method='linear')

    out2rat = InterpFcn(selections)[np.newaxis]  # Interpolate potential field at location of neural segments


    outdf = pd.DataFrame(data=(out2rat / currentApplied), columns=positions.columns) # Scale potential field by applied current

    return outdf

def load_positions(segment_position_folder, filesPerFolder, numPositionFiles, rank):

    '''
    Loads positions file based on rank
    '''

    index = int(rank % numPositionFiles) # Selects position file to load
    folder = int(index/filesPerFolder) # Finds which subfolder the position file is in

    allPositions = pd.read_pickle(segment_position_folder+'/'+str(folder)+'/positions'+str(index)+'.pkl')

    return allPositions

def getNeuronSegmentMidpts(position):
    '''
    Gets midpoints for a single neuron
    '''

    
    secIds = np.array(list(position.columns))[:,1]
    uniqueSecIds = np.unique(secIds)

    for sId in uniqueSecIds: # Iterates through sections

        pos = position.iloc[:,np.where(sId == secIds)[0]]

        if sId == 0: # Implies that section is a soma, so we just take the position from the file

            newPos = pos
 
        elif np.shape(pos.values)[-1] == 1: # If there is only one point in the section, we just take the value
            newPos = pd.concat((newPos,pos),axis=1)

        else: # We take the midpoints of the values in the file, which are the endpoints of the segments
            pos = (pos.iloc[:,:-1]+pos.iloc[:,1:])/2

            newPos = pd.concat((newPos,pos),axis=1)

    return newPos

def getSegmentMidpts(positions,node_ids):

    newPos = positions.groupby(level=0,axis=1,group_keys=False).apply(getNeuronSegmentMidpts)

    
    return newPos

def getReport(path_to_simconfig):

    '''
    Loads compartment report from simulation_config
    '''

    with open(path_to_simconfig) as f:

        simconfig = json.load(f)

        outputdir = simconfig['output']['output_dir']

        report = simconfig['reports']['compartment']['file_name']

        path_to_report = outputdir + '/' + report + '.h5'

    r = lb.ElementReportReader(path_to_report)
    population_name = r.get_population_names()[0]

    r = r[population_name]

    return r, population_name

def get_indices(rank, nranks,numPositionFiles):
    
    iterationsPerFile = int(nranks/numPositionFiles) # How many ranks is any position file divided among

    iterationSize = int(1000/iterationsPerFile)  # Number of node_ids processed on this rank

    iteration = int(rank/numPositionFiles)
    
    return iteration, iterationSize

def get_position_file(filesPerFolder, numPositionFiles, rank):
    
    index = int(rank % numPositionFiles)
    folder = int(index/filesPerFolder)
    
    return str(folder)+'/positions'+str(index)+'.pkl'

def load_positions(segment_position_folder, filesPerFolder, numPositionFiles, rank):
    
    position_file = get_position_file(filesPerFolder, numPositionFiles, rank)

    allPositions = pd.read_pickle(segment_position_folder+'/'+position_file)

    return allPositions

def getCurrentIds(positions,iteration,iterationSize):
    
    #### For the current rank, selects node ids for which to calculate the coefficients
    try:
        node_ids= np.unique(np.array(list(positions.columns))[:,0])[iteration*iterationSize:(iteration+1)*iterationSize]
    except:
        node_ids = np.unique(np.array(list(positions.columns))[:,0])[iteration*iterationSize:]

    assert len(node_ids)>0

    #####
    
    return node_ids

def getIdsAndPositions(ids, segment_position_folder, numFilesPerFolder):
    
    '''
    For the current rank, selects gids for which to calculate the coefficients
    '''
    
    rank = MPI.COMM_WORLD.Get_rank()
    nranks = MPI.COMM_WORLD.Get_size()
    
    numPositionFiles = np.ceil(len(ids)/1000) # Each position file has 1000 gids

    positions = load_positions(segment_position_folder,numFilesPerFolder, numPositionFiles, rank)

    iteration, iterationSize = get_indices(rank, nranks,numPositionFiles)
    
    g = getCurrentIds(positions,iteration,iterationSize)
    
    positions = positions[g] # Gets positions for specific gids
    
    return g, positions
    
def sort_electrode_names(electrodeKeys,population_name):
    
    electrodeNames = np.array(list(electrodeKeys))
    
    electrodeNames = electrodeNames[np.where(electrodeNames!=population_name)] # The field /electrodes/{population_name} contains the scaling factors, not the metadata
    
    electrode_list = []
    
    for e in electrodeNames:
        
        try:
            name = int(e)
            
        except:
            name = e
            
        electrode_list.append(name)
        
    electrode_list = np.sort(electrode_list)
    
    return electrode_list
            
            

def writeH5File(path_to_simconfig,segment_position_folder,outputfile,numFilesPerFolder,sigma=0.277,path_to_fields=None):

    '''
    path_to_simconfig refers to the BlueConfig from the 1-timestep simulation used to get the segment positions
    segment_position_folder refers to the path to the pickle file containing the potential at each segment. This is the output of the interpolation script
    '''
    t = time.time()
    r, population_name = getReport(path_to_simconfig)


    allNodeIds = r.get_node_ids()
    h5 = h5py.File(outputfile, 'a',driver='mpio',comm=MPI.COMM_WORLD)

    try:
        node_ids, positions = getIdsAndPositions(allNodeIds, segment_position_folder,numFilesPerFolder)

    except:
        h5.close()
        return 1

    node_ids_sonata = lb.Selection(values=node_ids)

    
    data_frame = r.get(node_ids=node_ids_sonata,tstart=0,tstop=0.1) # Loads compartment report for sleected node_ids
    
    data = pd.DataFrame(data_frame.data, columns=pd.MultiIndex.from_tuples(tuple(map(tuple,data_frame.ids)), names=['id','section']), index=data_frame.times) # Writes compartment report as pandas dataframe

    columns = data.columns


    coeffList = []
    
    electrodeNames = sort_electrode_names(h5['electrodes'].keys(),population_name)
    
    reciprocityIdx = 0 # Keeps track of number of non-analytical electrodes
    
    for electrodeIdx, electrode in enumerate(electrodeNames):


        epos = h5['electrodes'][str(electrode)]['position'][:] # Gets position for each electrode
        
        
        electrodeType = h5['electrodes'][str(electrode)]['type'][()].decode() # Gets position for each electrode

        if electrodeType == 'LineSource':
            
            coeffs = get_coeffs_lfp(positions,columns,epos,sigma)
            
        else:

            newPositions = getSegmentMidpts(positions,node_ids) # For other methods, we need the segment centers, not the endpoints
            
            
            if electrodeType == 'PointSource':
                
                coeffs = get_coeffs_pointSource(newPositions, epos, sigma)
                
            else:
            
                if electrodeType == 'DipoleReciprocity':

                    center = np.mean(newPositions,axis=1)

                    coeffs = get_coeffs_dipoleReciprocity(newPositions,path_to_fields[reciprocityIdx],center)

                else:

                    coeffs = get_coeffs_eeg(newPositions,path_to_fields[reciprocityIdx])

                reciprocityIdx += 1


        if electrodeIdx == 0:
            coeffList = coeffs
        else:
            coeffList = pd.concat((coeffList,coeffs))

    add_data(h5,node_ids,coeffList,population_name)

    h5.close()

    return 0


if __name__=='__main__':


    path_to_simconfig = sys.argv[1] # simulation_config.json with one-timestep compartment report
    segment_position_folder = sys.argv[2] # Folder with segment positions; output of getPositions.py
    outputfile = sys.argv[3]

    numFilesPerFolder = int(sys.argv[4]) # Number of files per subfolder in segment positions folder

    electrode_csv = sys.argv[5] # Data about each electrode in array

    electrode_df = pd.read_csv(electrode_csv,header=0,index_col=0)

    numElectrodes = len(electrode_df.index)

    sigma = 0.277 # Conductance of the brain tissue, in S/m
    path_to_fields = None # H5 file generated by the finite element solver with the potential field resulting from a current between two recording electrodes

    if len(sys.argv)>6: # Specify conductance or a potential field. If both are used, conductance must be first

        try:
            sigma = float(sys.argv[6]) # If the argument is a number, assume it is a conductance
        except:
            path_to_fields = sys.argv[6]
            
    if len(sys.argv)>7:
        path_to_fields = sys.argv[7]

    if path_to_fields is not None:
        if ' ' in path_to_fields: # If multiple potential field files, splits them into a list
            path_to_fields = path_to_fields.split(' ')
        else:
            path_to_fields = [path_to_fields] # Converts to list so that we can still call path_to_fields[0]


    writeH5File(path_to_simconfig,segment_position_folder,outputfile,numFilesPerFolder,sigma,path_to_fields)
