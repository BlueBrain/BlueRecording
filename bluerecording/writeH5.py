# SPDX-License-Identifier: GPL-3.0-or-later
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
from .utils import *
import warnings
from sklearn.decomposition import PCA

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

def line_source_cases(h,r2,l):


    if h < 0 and l < 0:

        lineSourceTerm = np.log(((h**2+r2)**.5-h)/((l**2+r2)**.5-l))

    elif h < 0 and l > 0:

        lineSourceTerm = np.log( ( ((h**2+r2)**.5-h)* (l + (l**2+r2)**.5 ) ) / r2  )

    elif h > 0 and l > 0:

        lineSourceTerm = np.log( ( (l + (l**2+r2)**.5 ) ) / ( (r2+h**2)**.5 + h)  )


    return lineSourceTerm

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

    l = h + segLength

    subtractionTerm = h**2

    r2 = (electrodePos[0]-endPos[0])**2 + (electrodePos[1]-endPos[1])**2 + (electrodePos[2]-endPos[2])**2 - subtractionTerm

    r2 = np.abs(r2)


    lineSourceTerm = line_source_cases(h,r2,l)

    segCoeff = 1/(4*np.pi*sigma*segLength)*lineSourceTerm

    segCoeff *= 1e-9 # Convert from nA to A

    return segCoeff


def get_coeffs_lineSource(positions,columns,electrodePos,sigma):

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

def getArraySpacing(allEpos):

    ### Finds main axis of electrode array
    pca = PCA(n_components=1)
    pca.fit(allEpos)
    main_axis = pca.components_[0]/np.linalg.norm(pca.components_[0])
    main_axis = main_axis[:,np.newaxis]
    ###

    allEpos_projected = np.matmul(allEpos,main_axis).flatten()

    arraySpacing = np.abs(np.diff(allEpos_projected))

    arraySpacing = arraySpacing[arraySpacing >1e-3] # removes zeros in order to not take into account electrodes on the same plane

    return main_axis, arraySpacing

def get_coeffs_objectiveCSD_Sphere(positions,electrodePos,allEpos):

    _, arraySpacing = getArraySpacing(allEpos)

    #radius = np.abs(np.mean(arraySpacing)/2) # Assumes that all electrodes are evenly spaced. TODO: Relax this assumption

    radius = 10

    distances = np.linalg.norm(positions.values-electrodePos[:,np.newaxis],axis=0) # in microns

    coeffs = np.array((distances <= radius).astype(int)) # Coeff is 1 if segment is within radius, zero otherwise

    coeffs = pd.DataFrame(data=coeffs[np.newaxis,:])

    coeffs.columns = positions.columns

    return coeffs


def get_coeffs_objectiveCSD_Plane(compartment_positions,electrodePos,allEpos):

    main_axis, arraySpacing = getArraySpacing(allEpos)

    planeThickness = getThickness(arraySpacing) # Assumes that all electrodes are evenly spaced. TODO: Relax this assumption

    axialDistances, _ = distances_in_planar_coords(compartment_positions,electrodePos,main_axis)

    coeffs = np.array((axialDistances <= planeThickness).astype(int)).flatten() ### Coeff is 1 if segment is within infinite plane, zero otherwise

    coeffs = pd.DataFrame(data=coeffs[np.newaxis,:])

    coeffs.columns = compartment_positions.columns

    return coeffs

def getThickness(arraySpacing):

    # Given the spacing between electrodes in an array, calculates the thickness for objective plane and objective disk

    return np.abs(np.mean(arraySpacing)/2)

def calculate_axial_vectors(axialDistances,main_axis):

    axialVectors = main_axis.T # Size 1x3
    for i in range(len(axialDistances)-1):
        axialVectors = np.vstack((axialVectors,main_axis.T))

    axialVectors = axialVectors * axialDistances

    return axialVectors


def distances_in_planar_coords(compartment_positions, electrodePos, main_axis):

    '''
    For a disk or plane perpendicular to main_axis, returns the axial and radial coordinates of each of the compartment positions
    '''

    ### Projects compartment positions onto plane, containing the point electrodePos, normal to electrode array
    differenceVectors = compartment_positions.values - electrodePos[:,np.newaxis]

    axialDistances = np.matmul(differenceVectors.T,main_axis) # Size len(compartment_positions)x1

    axialVectors = calculate_axial_vectors(axialDistances,main_axis) # Projection of diffence vector onto the main axis of the electrode array

    radialVectors = differenceVectors - axialVectors.T

    radialDistances = np.linalg.norm(radialVectors,axis=0) # in microns

    return np.abs(axialDistances), radialDistances


def get_coeffs_objectiveCSD_Disk(compartment_positions,electrodePos,allEpos):

    radius = 500 # Hard-coded. Todo make user-configurable

    main_axis, arraySpacing = getArraySpacing(allEpos)

    diskThickness = getThickness(arraySpacing) # Assumes that all electrodes are evenly spaced. TODO: Relax this assumption

    axialDistances, radialDistances = distances_in_planar_coords(compartment_positions,electrodePos,main_axis)

    ###

    ### Coeff is 1 if segment is within disk, zero otherwise
    coeffs1 = np.array((radialDistances <= radius).astype(int)).flatten()
    coeffs2 = np.array((axialDistances <= diskThickness).astype(int)).flatten()

    coeffs = coeffs1 * coeffs2
    ###

    coeffs = pd.DataFrame(data=coeffs[np.newaxis,:])

    coeffs.columns = compartment_positions.columns

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

def get_coeffs_dipoleReciprocity(compartment_positions, path_to_fields,center):

    '''
    path_to_fields is the path to the h5 file containing the potential field, outputted from Sim4Life
    '''


    positionColumns = compartment_positions.columns
    compartment_positions = compartment_positions.values


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

        ### E field is cell-centered, so we need to take midpoints of mesh
        xCenter = (x[:-1]+x[1:])/2
        yCenter = (y[:-1]+y[1:])/2
        zCenter = (z[:-1]+z[1:])/2
        ####

        currentApplied = f['CurrentApplied'][()] # The potential field should have a current, but if not, just assume it is 1


    compartment_positions = compartment_positions * 1e-6 # Converts um to m, to match the potential field file

    center = center * 1e-6

    compartment_positions_New = compartment_positions - center.values[:,np.newaxis]


    InterpFcnX = RegularGridInterpolator((xCenter, y, z), Ex[:, :, :, 0], method='linear')
    InterpFcnY = RegularGridInterpolator((x, yCenter, z), Ey[:, :, :, 0], method='linear')
    InterpFcnZ = RegularGridInterpolator((x, y, zCenter), Ez[:, :, :, 0], method='linear')

    XComp = InterpFcnX(center)[np.newaxis]  # Interpolate E field at location of neural center

    YComp = InterpFcnY(center)[np.newaxis]  # Interpolate E field at location of neural center

    ZComp = InterpFcnZ(center)[np.newaxis]  # Interpolate E field at location of neural center


    out2rat = compartment_positions_New[0]*XComp + compartment_positions_New[1]*YComp + compartment_positions_New[2]*ZComp


    outdf = pd.DataFrame(data=(-out2rat / currentApplied), columns=positionColumns) # Scale potential field by applied current

    return outdf

def get_coeffs_reciprocity(compartment_positions, path_to_fields):

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


        currentApplied = f['CurrentApplied'][()] # The potential field should have a current, but if not, just assume it is 1


    compartment_positions *= 1e-6 # Converts um to m, to match the potential field file

    xSelect = compartment_positions.values[0]
    ySelect = compartment_positions.values[1]
    zSelect = compartment_positions.values[2]


    selections = np.array([xSelect, ySelect, zSelect]).T


    InterpFcn = RegularGridInterpolator((x, y, z), pot[:, :, :, 0], method='linear')

    out2rat = InterpFcn(selections)[np.newaxis]  # Interpolate potential field at location of neural segments


    outdf = pd.DataFrame(data=(out2rat / currentApplied), columns=compartment_positions.columns) # Scale potential field by applied current

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


def get_indices(rank, nranks,neurons_per_file,numPositionFiles):

    iterationsPerFile = int(nranks/numPositionFiles) # How many ranks is any position file divided among

    if iterationsPerFile < 1:
        raise AssertionError("One rank cannot process more than one position file. Either increase the number of ranks or increase the number of neurons per file if necessary")

    iterationSize = int(np.ceil(neurons_per_file/iterationsPerFile))  # Number of node_ids processed on this rank

    if iterationSize < 1:
        raise AssertionError("Each rank must process at least one neuron. Either decrease the number of ranks or decrease the number of neurons per file if necessary")

    iteration = int(rank/numPositionFiles)

    return iteration, iterationSize

def get_position_file_name(filesPerFolder, numPositionFiles, rank):

    index = int(rank % numPositionFiles)
    folder = int(index/filesPerFolder)

    return str(folder)+'/positions'+str(index)+'.pkl'

def load_positions(segment_position_folder, filesPerFolder, numPositionFiles, rank):

    position_file = get_position_file_name(filesPerFolder, numPositionFiles, rank)

    allPositions = pd.read_pickle(segment_position_folder+'/'+position_file)

    return allPositions

def getCurrentIds(positions,iteration,iterationSize):

    #### For the current rank, selects node ids for which to calculate the coefficients
    try:
        node_ids= np.unique(np.array(list(positions.columns))[:,0])[iteration*iterationSize:(iteration+1)*iterationSize]
    except:
        node_ids = np.unique(np.array(list(positions.columns))[:,0])[iteration*iterationSize:]

    #####

    return node_ids

def getIdsAndPositions(ids, segment_position_folder,neurons_per_file, numFilesPerFolder):

    '''
    For the current rank, selects node_ids for which to calculate the coefficients, and returns their positions
    '''

    rank = MPI.COMM_WORLD.Get_rank()
    nranks = MPI.COMM_WORLD.Get_size()

    numPositionFiles = np.ceil(len(ids)/neurons_per_file)

    positions = load_positions(segment_position_folder,numFilesPerFolder, numPositionFiles, rank)

    iteration, iterationSize = get_indices(rank, nranks,neurons_per_file,numPositionFiles)

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

def ElectrodeType(electrodeType):

    if electrodeType == 'LineSource' or electrodeType == 'PointSource' or electrodeType == 'DipoleReciprocity' or electrodeType == 'Reciprocity' or electrodeType == 'ObjectiveCSD_Sphere' or electrodeType == 'ObjectiveCSD_Disk' or electrodeType == 'ObjectiveCSD_Plane':
        return 0
    else:
        raise AssertionError("Electrode type not recognized")

def get_objectiveCSD_array(electrodeType,objective_csd_array_indices,objectiveCSD_count,electrodeNames, h5, electrodeIdx):

    if objective_csd_array_indices is None: # Assume all electrodes of given type are used to calculate CSD

        allTypes = []
        for electrode in electrodeNames:
            allTypes.append( h5['electrodes'][str(electrode)]['type'][()].decode() )

        arrayIdx = [i for i, e in enumerate(allTypes) if e==electrodeType]

    else:

        arrayIdx = processSubsampling(objective_csd_array_indices[objectiveCSD_count])

        if electrodeIdx not in arrayIdx:
            objectiveCSD_count += 1
            arrayIdx = processSubsampling(objective_csd_array_indices[objectiveCSD_count])
            if electrodeIdx not in arrayIdx:
                raise AssertionError('Electrode arrays used in objective CSD must be sequential in eletcrode file')

    return arrayIdx, objectiveCSD_count

def writeH5File(path_to_simconfig,segment_position_folder,outputfile,neurons_per_file,files_per_folder,sigma=[0.277],path_to_fields=None,objective_csd_array_indices=None):

    '''
    path_to_simconfig refers to the BlueConfig from the 1-timestep simulation used to get the segment positions
    segment_position_folder refers to the path to the top-level folder containing pickle files with the position of each segment. This is generated by the getPositions() function
    outputfile is the h5 file containing the compartment weights
    neurons_per_file is the number of neurons in each of the segment positions pickle files. This is specified by the user in the getPositions() step
    files_per_folder is the number of positions pickle files in each subfolder in segment_position_folder. This is also specified by the user in getPositions()
    '''

    r, allNodeIds = getSimulationInfo(path_to_simconfig)
    population_name = getPopulationName(path_to_simconfig)


    h5 = h5py.File(outputfile, 'a',driver='mpio',comm=MPI.COMM_WORLD)

    node_ids, positions = getIdsAndPositions(allNodeIds, segment_position_folder,neurons_per_file, files_per_folder)

    if len(node_ids)==0:

        warnings.warn('No nodes are processed on rank '+str(MPI.COMM_WORLD.Get_rank())+' Either increase or reduce the number of ranks such that it is an integer multiple of the number of position files')

        h5.close()

        return 1


    data = getMinimalReport(r,node_ids) # Loads compartment report for sleected node_ids


    columns = data.columns


    coeffList = []

    electrodeNames = sort_electrode_names(h5['electrodes'].keys(),population_name)

    reciprocityIdx = 0 # Keeps track of number of non-analytical electrodes
    sigmaIdx = 0 # Keeps track of number of analytical electrodes
    objectiveCSD_count = 0 # Keeps track of number of objective CSD electrodes

    for electrodeIdx, electrode in enumerate(electrodeNames):


        epos = h5['electrodes'][str(electrode)]['position'][:] # Gets position for each electrode


        electrodeType = h5['electrodes'][str(electrode)]['type'][()].decode() # Gets type for each electrode


        ElectrodeType(electrodeType)

        if electrodeType == 'LineSource':

            coeffs = get_coeffs_lineSource(positions,columns,epos,sigma[sigmaIdx])

            if len(sigma) > 1:
                sigmaIdx += 1

        else:

            newPositions = getSegmentMidpts(positions,node_ids) # For other methods, we need the segment centers, not the endpoints


            if electrodeType == 'PointSource':

                coeffs = get_coeffs_pointSource(newPositions, epos, sigma[sigmaIdx])

                if len(sigma) > 1:
                    sigmaIdx += 1

            elif 'ObjectiveCSD' in electrodeType:

                arrayIdx, objectiveCSD_count = get_objectiveCSD_array(electrodeType, objective_csd_array_indices, objectiveCSD_count, electrodeNames, h5, electrodeIdx)

                allEpos = [] # List of electrode positions used to calculate CSD

                for e in electrodeNames[arrayIdx]:
                    allEpos.append( h5['electrodes'][str(e)]['position'][:] )

                if electrodeType == 'ObjectiveCSD_Sphere':

                    coeffs = get_coeffs_objectiveCSD_Sphere(newPositions,epos,allEpos)

                elif electrodeType == 'ObjectiveCSD_Disk':

                    coeffs = get_coeffs_objectiveCSD_Disk(newPositions,epos,allEpos) # Radius is hardcoded to 500 um for disk. TODO make this user-configurable

                elif electrodeType == 'ObjectiveCSD_Plane':

                    coeffs = get_coeffs_objectiveCSD_Plane(newPositions,epos,allEpos) # Radius is hardcoded to 500 um for disk. TODO make this user-configurable


            else:

                if electrodeType == 'DipoleReciprocity':

                    center = newPositions.mean(axis=1)

                    coeffs = get_coeffs_dipoleReciprocity(newPositions,path_to_fields[reciprocityIdx],center)

                else:

                    coeffs = get_coeffs_reciprocity(newPositions,path_to_fields[reciprocityIdx])

                reciprocityIdx += 1


        if electrodeIdx == 0:
            coeffList = coeffs
        else:
            coeffList = pd.concat((coeffList,coeffs))

    add_data(h5,node_ids,coeffList,population_name)

    h5.close()

    return 0
