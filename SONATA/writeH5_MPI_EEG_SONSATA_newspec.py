import libsonata as lb
import numpy as np
import h5py
import pandas as pd
import sys
from mpi4py import MPI
from scipy.spatial import distance
from scipy.spatial.transform import Rotation
from voxcell.nexus.voxelbrain import Atlas
from scipy.interpolate import RegularGridInterpolator

def add_data(h5, gid, coeffs, electrode_struc,population_name):

    dset = 'electrodes/'+population_name+'/scaling_factors'

    node_ids = h5[population_name+'/node_ids'][:]

    gidIndex = np.where(gid==node_ids)[0][0]

    
    offset0 = h5[population_name+'/offsets'][gidIndex]


    if gidIndex == len(h5[population_name+'/offsets'][:])-1:
        
        h5[dset][offset0:] = coeffs

    else:
        offset1 = h5[population_name+'/offsets'][gidIndex+1]
        h5[dset][offset0:offset1] = coeffs


def getAtlasInfo(BlueConfig,electrodePositions):


    regionList = []
    layerList = []


    for position in electrodePositions:

        try:

            for id_ in brain_regions.lookup([position]):

                region = region_map.get(id_, 'acronym')
                regionList.append(region.split(';')[0])
                layerList.append(region.split(';')[1])

        except:

            regionList.append('Outside')
            layerList.append('Outside')

    return regionList, layerList

def makeElectrodeDict(names,positions,types,regions,layers):

    electrodes = {}


    for i, name in enumerate(names):
        electrodes[name] = {'position': positions[i],'type': types[i],
        'region':regions[i],'layer':layers[i]}

    return electrodes

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

            meanPos = (positions.iloc[:,i]+positions.iloc[:,i+1])/2
            distance = np.linalg.norm(meanPos-electrodePos)

            segCoeff = 1/(4*np.pi*sigma*distance)

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

        currentApplied = 1 #f['CurrentApplied'][0]


    positions *= 1e-6 # Converts um to m

    xSelect = positions.values[0]
    ySelect = positions.values[1]
    zSelect = positions.values[2]


    selections = np.array([xSelect, ySelect, zSelect]).T


    InterpFcn = RegularGridInterpolator((x, y, z), pot[:, :, :, 0], method='linear')

    out2rat = InterpFcn(selections)

    out2rat = out2rat[np.newaxis]

    outdf = pd.DataFrame(data=(out2rat / currentApplied), columns=positions.columns)

    return outdf



def writeH5File(path_to_blueconfig,outputfile,electrodePositions,electrodeNames,typeList,positions,sigma=0.277,path_to_fields=None):

    '''
    path_to_blueconfig refers to the BlueConfig from the 1-timestep simulation used to get the segment positions
    inputfile refers to the path to the pickle file containing the potential at each segment. This is the output of the interpolation script
    electrodePositions is a list containing the positions (in 3D cartesian space) of the recording and reference electrodes (if EEG)
    gidList is a list of the desired gids
    '''

    regionList, layerList = getAtlasInfo(path_to_blueconfig,electrodePositions)

    nameList = electrodeNames
    electrodeType = typeList[0]

    r = lb.ElementReportReader('reporting/voltage.h5')

    population_name = r.get_population_names()[0]

    r = r[population_name]

    rank = MPI.COMM_WORLD.Get_rank()
    nranks = MPI.COMM_WORLD.Get_size()

    iterPerFile = int(nranks/212)
    iterSize = int(1000/iterPerFile)

    set = int(rank/212)
    
#    iterSize = 1
    electrodes = makeElectrodeDict(nameList,electrodePositions,typeList,regionList,layerList)


    h5 = h5py.File(outputfile, 'a',driver='mpio',comm=MPI.COMM_WORLD)
    try:
        g = np.unique(np.array(list(positions.columns))[:,0])[set*iterSize:(set+1)*iterSize]
    except:
        g = np.unique(np.array(list(positions.columns))[:,0])[set*iterSize:]

    if len(g) == 0:
        h5.close()
        return 1

    gSonata = lb.Selection(values=g)
    
    data_frame = r.get(node_ids=gSonata,tstart=0,tstop=0.1)

    data = pd.DataFrame(data_frame.data, columns=pd.MultiIndex.from_tuples(tuple(map(tuple,data_frame.ids)), names=['gid','section']), index=data_frame.times)


    columns = data.columns

    positions = positions[g]

    for gidx, gid in enumerate(g):
       
        position = positions[gid]

        segIds = np.array(list(position.columns))
        uniqueSegIds = np.unique(segIds)

        for sId in uniqueSegIds:
            
            
            pos = position.iloc[:,np.where(sId == segIds)[0]]
    
            if sId == 0:
                
                newcols = pd.MultiIndex.from_product([[gid],pos.columns])
                pos.columns = newcols

                if gidx == 0:
                    newPos = pos
                else:
                    newPos = pd.concat((newPos,pos),axis=1)
                
            elif np.shape(pos.values)[-1] == 1:
                newcols = pd.MultiIndex.from_product([[gid],pos.columns])
                pos.columns = newcols
                newPos = pd.concat((newPos,pos),axis=1)
            else:
                pos = (pos.iloc[:,:-1]+pos.iloc[:,1:])/2
            
                newcols = pd.MultiIndex.from_product([[gid],pos.columns])
                pos.columns = newcols
                newPos = pd.concat((newPos,pos),axis=1)

    positions = newPos
    
    coeffList = []
    
    for ePos in electrodePositions:

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

        add_data(h5, gid, newCoeffs, electrodes,population_name)

    h5.close()

    return 0

if __name__=='__main__':


    probe_name = sys.argv[1]

    path_to_Blueconfig = sys.argv[3]
    inputfolder = sys.argv[4]
    outputfolder = sys.argv[5]

    numElectrodes = np.arange(1)


    type = sys.argv[2]
    
    file = h5py.File(outputfolder)

    names = []
    types = []
    positions = []
    for i in range(len(numElectrodes)):
        names.append(probe_name+'_'+str(i))
        types.append(type)

        positions.append(file['electrodes'][probe_name+'_'+str(i)]['position'][:])

    numFilesPerFolder = int(sys.argv[6])
    
    file.close()
    electrodePositions = np.array(positions)

    sigma = 0.277
    path_to_fields = '/gpfs/bbp.cscs.ch/project/proj85/scratch/bbp_workflow/SSCx-O1-Calibrated-Workflow-5-12-22/81adc949-b514-4d78-853a-b369e4420dff/3/C6SurfNew.h5'


    if len(sys.argv)>7:
        sigma = float(sys.argv[7])
        if len(sys.argv)>8:
            path_to_fields = sys.argv[8]

            
    r = lb.ElementReportReader('reporting/voltage.h5')
    r = r[r.get_population_names()[0]]
    node_ids = r.get_node_ids()
                            

    numGids = len(node_ids)

    numFolders = int(np.ceil(numGids/1000))

    rank = MPI.COMM_WORLD.rank

    nranks = MPI.COMM_WORLD.Get_size()

    filesPerFolder = 50

    rank = MPI.COMM_WORLD.Get_rank() % 212
    
    folder = int(rank/filesPerFolder)

    allPositions = pd.read_pickle(inputfolder+'/'+str(folder)+'/positions'+str(rank)+'.pkl')


    writeH5File(path_to_Blueconfig,outputfolder,electrodePositions,names,types,allPositions,sigma,path_to_fields)
