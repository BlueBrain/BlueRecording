import numpy as np
from morphio import Morphology, SectionType
import bluepy as bp

import pandas as pd
import sys
from scipy.interpolate import interp1d
import time
from mpi4py import MPI
import warnings

warnings.filterwarnings('error', '', RuntimeWarning)
'''
'''


def interp_points(coords, ncomps):

    '''
    For a given dendritic section with 3d points coords and a number of segments ncomps, we interpolate the start and end points of each segment,
    with each segment having equal length
    '''

    xyz = np.array([]).reshape(ncomps + 1, 0)
    npoints = coords.shape[0]

    for dim in range(coords.shape[1]):
        distances = np.cumsum(np.linalg.norm(np.diff(coords,axis=0),axis=1))
        distances /= distances[-1]
        distances = np.insert(distances,0,0)

        f = interp1d(distances, coords[:, dim], kind='linear')
        ic = f(np.linspace(0, 1, ncomps + 1)).reshape(ncomps + 1, 1)
        xyz = np.hstack((xyz, ic))

    return xyz


def get_axon_points(m):

    '''
    This function returns the 3d positions and the cumulative length of the part of the axon that is simulated
    We only simulate two sections of the AIS, each 30 um long, and a 1000 um myelinated section
    The positions of these sections are not defined by the simulator, so we assume that it is the first 1060 um of a particular axonal branch
    For efficiency, we just take the first axonal branch we find that has a length of at least 1060 um
    '''

    targetLength = 1060

    points = np.Inf

    currentLen = 0

    somaPos = np.mean(m.soma.points, axis=0)[:, np.newaxis]

    needExtension = False # Flag that indicates whether the longest branch that actually exists in the morphology is shorter than 1060 um

    for sec in m.sections: # Iterates through axon to find the longest branch

        if sec.type == SectionType.axon and len(sec.children) == 0: # Finds an axonal end point

            length = 0
            longestLength = 0
            thisSec = sec

            idxs = [] # List of section ids in the branch

            idxs.append(thisSec.id)

            while not thisSec.is_root: # From the end section, iterates backwards through tree until we reach the soma

                newSec = thisSec.parent

                length += np.linalg.norm(newSec.points[-1] - thisSec.points[-1]) # Adds euclidean distance from end of parent to end of child to length

                thisSec = newSec
                idxs.append(thisSec.id)

            length += np.linalg.norm(np.diagonal(thisSec.points[-1] - somaPos)) # Euclidean length of root axonal section

            if length > longestLength: # If this branch is longer than the previous longest branch we update the longest brancg
                longestLength = length
                bestIdx = idxs

            if length > 1060: # We select the first branch we find that is longer than 1060 um
                break

    if length < 1060: # If none of the branches is longer than 1060 um, we need to extrapolate points
        idxs = bestIdx
        needExtension = True

    idxs.reverse() # We put the section indices in order from soma to end of the axon

    lastpt = somaPos

    points = somaPos.reshape(-1,3)
    runningLen = [0]

    for x in idxs: # We iterate through the selected axonal sections

        sec = m.sections[x]

        pts = sec.points

        for pt in pts: # Iterate through 3d points in the section

            if (lastpt == somaPos).all(): # We calculate the distance of the current point from the soma
                currentLen += np.linalg.norm(pt - lastpt)
            else:
                currentLen += np.linalg.norm(pt - lastpt)

            runningLen.append(currentLen)

            points = np.vstack((points, pt))

            lastpt = pt

            if currentLen > targetLength: #Stop when we have 1060 um
                break

        if currentLen > targetLength:
            break

    if needExtension: # If we need to extrapolate positions, we do a linear extrapolation based on the last two points

        toAdd = targetLength - currentLen

        slopes = (points[-1] - points[-2]) / (runningLen[-1] - runningLen[-2])

        newpt = points[-1] + slopes * toAdd

        points = np.vstack((points, newpt))

        currentLen += np.linalg.norm(newpt - points[-2])

        runningLen.append(currentLen)

    return np.unique(np.array(points),axis=0), np.unique(np.array(runningLen)) # We delete duplicate points that may occur if the morphology is in a format where section start and end points are repeated


def interp_points_axon(axonPoints, runningLens, secName, numCompartments, somaPos):

    segPos = []


    if secName == 1: # First AIS section

        secLen = 30 # By construction, has length of 30 um
        segLen = secLen / numCompartments # Assumes each segment has the same length

        startPoint = 0
        endPoint = 30

        idx = np.where(runningLens <= endPoint) # Finds indices of axon 3d points where cumulative length < 30 um

        axonRelevant = axonPoints[idx]


        lensRelevant = runningLens[idx] / secLen # Gets fraction of the total section length for each 3d point


        if len(axonRelevant) < 2: # If there are not enough points, we use the soma position and the first point in the axon
            idx = 0

            axonRelevant = np.vstack((somaPos.reshape((-1,3)), axonPoints[idx]))

            lensRelevant = np.array([0, runningLens[idx] / secLen])


    elif secName == 2: # Second AIS section

        secLen = 30 # Length is 30 unm, by construction
        segLen = secLen / numCompartments

        startPoint = 30 # Cumulative length of first AIS section
        endPoint = 60 # Cumulative length of both AIS sections

        idx = np.intersect1d(np.where(runningLens <= endPoint), np.where(runningLens >= startPoint)) # Finds indices 3d points falling in this length bin

        axonRelevant = axonPoints[idx]

        lensRelevant = (runningLens[idx] - startPoint) / secLen

        if len(axonRelevant) < 2: # If there aren't enough points, we estimate

            idxSmall = np.argmin(np.abs(runningLens - startPoint)) # Index closest to 30 um

            idxBig = np.argmin(np.abs(runningLens - endPoint)) # Index closest to 60 um

            idx = [idxSmall, idxBig]

            axonRelevant = axonPoints[idx]
            lensRelevant = (runningLens[idx] - startPoint) / secLen

            if idxSmall == idxBig: # If these two points are the same, we estimate based on the soma
                axonRelevant = np.vstack((somaPos.reshape((-1,3)), axonPoints[idxBig]))
                lensRelevant = np.array([0, (runningLens[idxBig] - startPoint) / secLen])


    else: # Myelinated section

        secLen = 1000
        segLen = secLen / numCompartments

        startPoint = 60
        endPoint = 1060

        idx = np.where(runningLens >= startPoint)[0] # Get indices of 3d points that are beyond the AIS

        if len(idx) == 1:
            idx = [idx[0] - 1, idx[0]]

        axonRelevant = axonPoints[idx]

        lensRelevant = (runningLens[idx] - startPoint) / secLen

    for i in range(numCompartments+1): # Interpolates segment positions
    
        frac = (i * segLen) / secLen


        fx = interp1d(lensRelevant, axonRelevant[:, 0], kind='linear', fill_value='extrapolate')
        fy = interp1d(lensRelevant, axonRelevant[:, 1], kind='linear', fill_value='extrapolate')
        fz = interp1d(lensRelevant, axonRelevant[:, 2], kind='linear', fill_value='extrapolate')

        newx = fx(frac)
        newy = fy(frac)
        newz = fz(frac)

        segPos.append([newx, newy, newz])


    segPos = np.array(segPos)
    return segPos

def getSimulationInfo(path_to_BlueConfig, newidx):

    '''
    Returns a list of 1000 gids and the corresponding compartment report
    '''

    c = bp.Circuit(path_to_BlueConfig)
    s = bp.Simulation(path_to_BlueConfig)

    r = s.report('Current')
    g = c.cells.ids({'$target':'hex_O1'})

    try:
        ids = g[1000*newidx:1000*(newidx+1)]
    except:
        ids = g[1000*newidx:]


    data = r.get(gids=ids,t_start=r.meta['start_time'],t_end=r.meta['start_time']+r.meta['time_step'])

    return ids, data

def getSomaPosition(i, m, data):

    '''
    It is possible that the soma has multiple segments. In this case, the morphIO format does not make it easy to get the position for each of the segments
    Therefore, we just take the average of the points, and assume that each of the segments has the same position
    '''

    numSomas = np.shape(data[i][0])[-1]

    if numSomas < 2:
        somaPos = np.mean(m.soma.points, axis=0)[np.newaxis, :]

    else:
        somaPos = np.mean(m.soma.points,axis=0)[:,np.newaxis]

    return somaPos

def getNewIndex(colIdx):

    '''
    Because we are saving the start and end points for each non-somatic segment, we need to add an additional entry to the dataframe column indices for section
    '''

    newIdx = []

    for i, col in enumerate(colIdx): # Iterates through column indices of compartment report
        newIdx.append(col)
        if i == len(colIdx)-1: # For the last compartment, we need to add an index to account for the end point
            newIdx.append(col)
        elif col[-1]!=0: # If the compartment is not a soma
            if colIdx[i+1]!=col: # and if the segment is not in the middle of the section
                newIdx.append(col)

    newCols = pd.MultiIndex.from_tuples(newIdx, names=colIdx.names)

    return newCols


def main(path_to_BlueConfig, path_to_positions_folder, newidx,chunk_size):


    ids, data = getSimulationInfo(path_to_BlueConfig, newidx)

    colIdx = data.columns # GID and Section IDs for each cell
    cols = np.array(list(data.columns))


    for idx, i in enumerate(ids): # Iterates through GIDs and gets segment positions


        m = c.morph.get(i,transform=True) # MorphIO morphology object

        axonPoints, runningLens = get_axon_points(m) # Gets 3d positions and cumulative length of the axon

        sections = np.unique(cols[np.where(cols[:,0]==i),1:].flatten()) # List of sections for the given neuron

        somaPos = getSomaPosition(i,m,data)

        if idx ==0: # For the first cell, the array of positions xyz is initialized with the soma position
            xyz = somaPos.reshape(3,1)
        else:
            xyz = np.hstack((xyz,somaPos.reshape(3,1)))

        if numSomas > 1: # If there is more than one somatic segment, we assume that they all have the same position
            for k in np.arange(1,numSomas):
                xyz = np.hstack((xyz,somaPos.reshape(3,1)))

        for secName in list(sections[1:]):

            '''
            For each non-somatic segment, we interpolate the start and end point. Having the start and end, rather than the center, allows us to use either the
            line-source approximation (for LFP) or the reciprocity approach (for EEG). Note that for segments in the middle of a section, the end point is the
            same as the start point of the next section, so we do not need to save the end point, just the start point
            '''

            numCompartments = np.shape(data[i][secName])[-1]

            if secName < 3: # Section 1 and Section 2 are always axonal sections

                segPos = interp_points_axon(axonPoints,runningLens,secName,numCompartments,somaPos)

            else:

                try: # Most other sections are dendritic, and are therefore included in the morphIO morphology object

                    sec = m.sections[secName-1] # If the section is not dendritic, this will trhow an Exception
                    pts = sec.points

                    secPts = np.array(pts)

                    segPos = interp_points(secPts,numCompartments)

                except: # If the section is not in the morphIO object, then it is the myelinated part of the AIS

                    segPos = interp_points_axon(axonPoints,runningLens,secName,numCompartments,somaPos)


            xyz = np.hstack((xyz,segPos.T))

    newCols = getNewIndex(colIdx)

    positionsOut = pd.DataFrame(xyz,columns=newCols)

    positionsOut.to_pickle(path_to_positions_folder+'/' + str(int(newidx / chunk_size)) + '/positions'+str(newidx)+'.pkl')

if __name__=='__main__':

    path_to_BlueConfig = sys.argv[1] #BlueConfig with one-timestep simulation outputting a compartment report with name "Current"
    path_to_positions_folder = sys.argv[2]

    newidx = MPI.COMM_WORLD.Get_rank()
    chunk_size = int(sys.argv[2]) # Number of pickle files to write to each subfolder

    main(path_to_BlueConfig, path_to_positions_folder, newidx, chunk_size)
