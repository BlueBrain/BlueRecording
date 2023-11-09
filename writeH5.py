import bluepy as bp
import numpy as np
import h5py
import pandas as pd
import sys
from mpi4py import MPI
from scipy.spatial import distance
from scipy.spatial.transform import Rotation
from voxcell.nexus.voxelbrain import Atlas

def add_data(h5, gid, coeffs, electrode_struc, rank):

    dset = 'electrodes/electrode_grid/'+str(int(gid))
    h5[dset][:,:-1] = coeffs


def get_line_coeffs(startPos,endPos,electrodePos,sigma):

    '''
    Uses line soruce approximation to get contribution from a segment
    startPos and endPos are the starting and ending positions of the segment
    sigma is the extracellular conductivity
    Implementation copied from LFPy
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

    for i in range(len(positions.columns)-1):

        if positions.columns[i][-1]==0: # Implies that it is a soma

            somaPos = positions.iloc[:,i]

            distance = np.linalg.norm(somaPos-electrodePos)

            somaCoeff = 1/(4*np.pi*sigma*distance) # We treat the soma as a point, so the contribution at the electrode follows the formula for the potential from a point source

            if i == 0:
                coeffs = somaCoeff
            else:

                coeffs = np.hstack((coeffs,somaCoeff))

        elif positions.columns[i][-1]==positions.columns[i+1][-1]: # Ensures we are not at the far end of a section

            segCoeff = get_line_coeffs(startPos,endPos,electrodePos,sigma)

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


        try:
            currentApplied = f['CurrentApplied'][0] # The potential field should have a current, but if not, just assume it is 1
        except:
            currentApplied = 1


    positions *= 1e-6 # Converts um to m, to match the potential field file

    xSelect = positions.values[0]
    ySelect = positions.values[1]
    zSelect = positions.values[2]


    selections = np.array([xSelect, ySelect, zSelect]).T


    InterpFcn = RegularGridInterpolator((x, y, z), pot[:, :, :, 0], method='linear')

    out2rat = InterpFcn(selections) # Interpolate potential field at location of neural segments


    outdf = pd.DataFrame(data=(out2rat / currentApplied), columns=positions.columns) # Scale potential field by applied current

    return outdf

def getGids(path_to_blueconfig):

    sim = bp.Simulation(path_to_blueconfig)

    rep = sim.report('Current')

    c = bp.Circuit(path_to_blueconfig)
    ids = c.cells.ids({'$target':'hex_O1'})

    return ids, rep

def load_positions(segment_position_folder, filesPerFolder, numPositionFiles, rank, nranks):

    index = int(rank % numPositionFiles)
    folder = int(index/filesPerFolder)

    allPositions = pd.read_pickle(segment_position_folder+'/'+str(folder)+'/positions'+str(index)+'.pkl')

    return allPositions

def getSegmentMidpts(positions,node_ids):

    for gidx, gid in enumerate(node_ids):

        position = positions[gid]

        secIds = np.array(list(position.columns))
        uniqueSecIds = np.unique(secIds)

        for sId in uniqueSecIds: # Iterates through sections

            pos = position.iloc[:,np.where(sId == secIds)[0]]

            if sId == 0: # Implies that section is a soma, so we just take the position from the file

                newcols = pd.MultiIndex.from_product([[gid],pos.columns])
                pos.columns = newcols

                if gidx == 0:
                    newPos = pos
                else:
                    newPos = pd.concat((newPos,pos),axis=1)

            elif np.shape(pos.values)[-1] == 1: # If there is only one point in the section, we just take the value
                newcols = pd.MultiIndex.from_product([[gid],pos.columns])
                pos.columns = newcols
                newPos = pd.concat((newPos,pos),axis=1)

            else: # We take the midpoints of the values in the file, which are the endpoints of the segments
                pos = (pos.iloc[:,:-1]+pos.iloc[:,1:])/2

                newcols = pd.MultiIndex.from_product([[gid],pos.columns])
                pos.columns = newcols
                newPos = pd.concat((newPos,pos),axis=1)

    return newPos


def writeH5File(electrodeType,path_to_simconfig,segment_position_folder,outputfile,numFilesPerFolder,sigma=0.277,path_to_fields=None):

    '''
    path_to_simconfig refers to the BlueConfig from the 1-timestep simulation used to get the segment positions
    segment_position_folder refers to the path to the pickle file containing the potential at each segment. This is the output of the interpolation script
    '''

    ids, rep = getGids(path_to_simconfig)


    rank = MPI.COMM_WORLD.Get_rank()
    nranks = MPI.COMM_WORLD.Get_size()

    numPositionFiles = np.ceil(len(ids)/1000) # Each position file has 1000 gids

    positions = load_positions(segment_position_folder,numFilesPerFolder, numPositionFiles, rank, nranks)

    iterationsPerFile = int(nranks/numPositionFiles)

    iterationSize = int(1000/iterationsPerFile)

    iteration = int(rank/numPositionFiles)

    h5 = h5py.File(outputfile, 'a',driver='mpio',comm=MPI.COMM_WORLD)

    #### For the current rank, selects gids for which to calculate the coefficients
    try:
        g = np.unique(np.array(list(positions.columns))[:,0])[iteration*iterationSize:(iteration+1)*iterationSize]
    except:
        g = np.unique(np.array(list(positions.columns))[:,0])[iteration*iterationSize:]

    if len(g) == 0:
        h5.close()
        return 1
    ##########

    data = rep.get(t_start=rep.t_start, t_end=rep.t_start + rep.t_step,gids=g) # Loads compartment report for selected GIDs

    columns = data.columns

    positions = positions[g] # Gets positions for specific gids

    coeffList = []

    for electrode in h5['electrodes'].keys():

        epos = h5['electrodes'][electrode]['position'] # Gets position for each electrode

        if electrodeType == 'LFP':
            coeffs = get_coeffs_lfp(positions,columns,ePos,sigma)
        else:

            newPositions = getSegmentMidpts(positions,node_ids) # For EEG, we need the segment centers, not the endpoints
            coeffs = get_coeffs_eeg(newPositions,path_to_fields)

        coeffList.append(coeffs)


    for i, gid in enumerate(g):


        for j, l in enumerate(coeffList):

            coeffs = np.array(coeffList.loc[:,gid].values).T

            if j == 0:
                newCoeffs = coeffs
            else:
                newCoeffs = np.hstack((newCoeffs, coeffs))

        add_data(h5, gid, newCoeffs, electrodes,rank)

    h5.close()

    return 0

if __name__=='__main__':

    type = sys.argv[1] # Either EEG or LFP

    path_to_simconfig = sys.argv[2]
    segment_position_folder = sys.argv[3]
    outputfile = sys.argv[4]

    numFilesPerFolder = int(sys.argv[5])

    electrode_csv = sys.argv[6]

    electrode_df = pd.read_csv(electrode_csv,header=0,index_col=0)

    numElectrodes = len(electrode_df.index)

    sigma = 0.277 # Conductance of the brain tissue, in S/m
    path_to_fields = None # H5 file generated by the finite element solver with the potential field resulting from a current between two recording electrodes

    if len(sys.argv)>7: # Specify either conductance or a potential field, not both

        try:
            sigma = float(sys.argv[7]) # If the argument is a number, assume it is a conductance
        except:
            path_to_fields = sys.argv[7]


    file = h5py.File(outputfile)

    positions = []

    for i in range(numElectrodes):

        positions.append(file['electrodes'][str(i)]['position'][:]) # Take electrode positions from h5 coefficient file



    file.close()

    electrodePositions = np.array(positions)

    writeH5File(type,path_to_simconfig,segment_position_folder,outputfile,numFilesPerFolder,sigma,path_to_fields)
