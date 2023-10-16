import bluepysnap as bp
import numpy as np
import h5py
import pandas as pd
import sys
from mpi4py import MPI
from scipy.spatial import distance
from scipy.spatial.transform import Rotation
from voxcell.nexus.voxelbrain import Atlas

def add_data(h5, gid, coeffs ,population_name):

    dset = 'electrodes/'+population_name+'/scaling_factors'

    node_ids = h5[population_name+'/node_ids'][:]

    gidIndex = np.where(gid==node_ids)[0][0]


    offset0 = h5[population_name+'/offsets'][gidIndex]


    if gidIndex == len(h5[population_name+'/offsets'][:])-1:

        h5[dset][offset0:] = coeffs

    else:
        offset1 = h5[population_name+'/offsets'][gidIndex+1]
        h5[dset][offset0:offset1] = coeffs

def get_line_coeffs(startPos,endPos,electrodePos,sigma):

    '''
    startPos and endPos are the starting and ending positions of the segment
    sigma is the extracellular conductivity
    '''

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

    return segCoeff

def get_coeffs_lfp(positions,columns,electrodePos,sigma):

    #print('len is' + str(len(positions.columns)))
    for i in range(len(positions.columns)-1):

        if positions.columns[i][-1]==0:

            somaPos = positions.iloc[:,i]

            distance = np.linalg.norm(somaPos-electrodePos)

            somaCoeff = 1/(4*np.pi*sigma*distance)

            if i == 0:
                coeffs = somaCoeff
            else:

                coeffs = np.hstack((coeffs,somaCoeff))

        elif positions.columns[i][-1]==positions.columns[i+1][-1]:


            segCoeff = get_line_coeffs(positions.iloc[:,i],positions.iloc[:,i+1],electrodePos,sigma)

            coeffs = np.hstack((coeffs,segCoeff))


    coeffs = pd.DataFrame(data=coeffs[np.newaxis,:])

    coeffs.columns = columns

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

        currentApplied = f['CurrentApplied'][0]


    positions *= 1e-6 # Converts um to m

    xSelect = positions.values[0]
    ySelect = positions.values[1]
    zSelect = positions.values[2]


    selections = np.array([xSelect, ySelect, zSelect]).T


    InterpFcn = RegularGridInterpolator((x, y, z), pot[:, :, :, 0], method='linear')

    out2rat = InterpFcn(selections)


    outdf = pd.DataFrame(data=(out2rat / currentApplied), columns=positions.columns)

    return outdf

def load_positions(segment_position_folder, filesPerFolder, numFolders, rank, nranks):

    index = rank % numFolders
    folder = int(rank/filesPerFolder)

    allPositions = pd.read_pickle(segment_position_folder+'/'+str(folder)+'/positions'+str(index)+'.pkl')

    return allPositions

def writeH5File(type,path_to_simconfig,segment_position_folder,outputfile,numFilesPerFolder,sigma=0.277,path_to_fields=None):

    '''
    path_to_blueconfig refers to the BlueConfig from the 1-timestep simulation used to get the segment positions
    inputfile refers to the path to the pickle file containing the potential at each segment. This is the output of the interpolation script
    electrodePositions is a list containing the positions (in 3D cartesian space) of the recording and reference electrodes (if EEG)
    gidList is a list of the desired gids
    '''

    r = bp.Simulation(path_to_simconfig)

    population_name = r.population_names[0]

    report = r[population_name]
    allNodeIds = report.node_ids

    numGids = len(allNodeIds)

    numFolders = np.ceil(numGids/1000)

    rank = MPI.COMM_WORLD.Get_rank()
    nranks = MPI.COMM_WORLD.Get_size()

    positions = load_positions(segment_position_folder,numFilesPerFolder, numFolders, rank, nranks)

    iterationsPerFile = int(nranks/numFolders)
    iterationSize = int(1000/iterationsPerFile)

    iteration = int(rank/numFolders)


    h5 = h5py.File(outputfile, 'a',driver='mpio',comm=MPI.COMM_WORLD)

    try:
        node_ids= np.unique(np.array(list(positions.columns))[:,0])[iteration*iterationSize:(iteration+1)*iterationSize]
    except:
        node_ids = np.unique(np.array(list(positions.columns))[:,0])[iteration*iterationSize:]

    if len(node_ids) == 0:
        h5.close()
        return 1

    data_frame = report.get(node_ids=node_ids,tstart=0,tstop=r.dt)

    data = pd.DataFrame(data_frame.data, columns=pd.MultiIndex.from_tuples(tuple(map(tuple,data_frame.ids)), names=['gid','section']), index=data_frame.times)


    columns = data.columns

    positions = positions[node_ids]

    coeffList = []

    for electrode in h5['electrodes'].keys():

        if electrode != population_name:

            epos = h5['electrodes'][electrode]['position']

            if electrodeType == 'LFP':
                coeffs = get_coeffs_lfp(positions,columns,ePos,sigma)
            else:
                coeffs = get_coeffs_eeg(positions,path_to_fields)

            coeffList.append(coeffs)


    for i, gid in enumerate(g):


        for j, l in enumerate(coeffList):

            coeffs = np.array(l.loc[:,gid].values).T

            if j == 0:
                newCoeffs = coeffs
            else:
                newCoeffs = np.hstack((newCoeffs, coeffs))

        add_data(h5, gid, newCoeffs ,population_name)

    h5.close()

    return 0


if __name__=='__main__':


    type = sys.argv[1]

    path_to_simconfig = sys.argv[2]
    segment_position_folder = sys.argv[3]
    outputfile = sys.argv[4]

    numFilesPerFolder = int(sys.argv[5])

    numElectrodes = np.arange(384)

    sigma = 0.277
    path_to_fields = None

    if len(sys.argv)>6:
        sigma = float(sys.argv[6])
        if len(sys.argv)>8:
            path_to_fields = sys.argv[7]


    file = h5py.File(outputfile)

    names = []
    types = []
    positions = []
    for i in range(len(numElectrodes)):
        names.append(probe_name+'_'+str(i))
        types.append(type)

        positions.append(file['electrodes'][probe_name+'_'+str(i)]['position'][:])



    file.close()

    electrodePositions = np.array(positions)

    writeH5File(type,path_to_simconfig,segment_position_folder,outputfile,numFilesPerFolder,sigma,path_to_fields)
