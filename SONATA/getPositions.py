import numpy as np
from morphio import SectionType
from morphio import Morphology
import libsonata as lb
import bluepysnap as bp

import pandas as pd
import sys
from scipy.interpolate import interp1d
import time
from mpi4py import MPI
import warnings
import json
from scipy.spatial.transform import Rotation as R

warnings.filterwarnings('error', '', RuntimeWarning)
'''
'''


class MutableMorph():

    '''
    This class defines a version of the morphIO morphology object that is both mutable and contains all of the data of the immutable object
    '''

    def __init__(self,morphioMorph):

        for attr in dir(morphioMorph):
            if '__' not in attr:
                setattr(self,attr,getattr(morphioMorph,attr))

        #### self.indices is a list of lists, where self.indices[i] is a list containing the indices of the 3d points for the ith section
        self.indices = []
        index = 0
        for section in self.sections:
            self.indices.append([])

            for i in range(len(section.points)):
                self.indices[-1].append(index)
                index += 1


def interp_points(coords, ncomps):

    '''
    For a given dendritic section with 3d points coords and a number of segments ncomps, we interpolate the start and end points of each segment,
    with each segment having equal length
    '''

    xyz = np.array([]).reshape(ncomps + 1, 0)
    npoints = coords.shape[0]

    for dim in range(coords.shape[1]):
        f = interp1d(np.linspace(0, 1, npoints), coords[:, dim], kind='linear')
        ic = f(np.linspace(0, 1, ncomps + 1)).reshape(ncomps + 1, 1)
        xyz = np.hstack((xyz, ic))

    return xyz


def get_axon_points(m,center):

    '''
    This function returns the 3d positions and the cumulative length of the part of the axon that is simulated
    We only simulate two sections of the AIS, each 30 um long, and a 1000 um myelinated section
    The positions of these sections are not defined by the simulator, so we assume that it is the first 1060 um of a particular axonal branch
    For efficiency, we just take the first axonal branch we find that has a length of at least 1060 um
    '''

    targetLength = 1060

    points = np.Inf

    currentLen = 0
    runningLen = []

    somaPos = center[:,np.newaxis]

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

                length += np.linalg.norm(m.points[m.indices[newSec.id][-1]] - m.points[m.indices[thisSec.id][0]]) # Adds euclidean distance from end of parent to start of child to length
                length += np.sum(np.linalg.norm(np.diff(m.points[m.indices[thisSec.id]],axis=0),axis=1)) # Adds sum of distance between each pair of points in child

                thisSec = newSec
                idxs.append(thisSec.id)

            length += np.linalg.norm(m.points[m.indices[thisSec.id][0]][:,np.newaxis] - somaPos) # Euclidean length of root axonal section
            length += np.sum(np.linalg.norm(np.diff(m.points[m.indices[thisSec.id]],axis=0),axis=1)) # Adds sum of distance between each pair of points in root

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

    for x in idxs: # We iterate through the selected axonal sections

        sec = m.sections[x]

        pts = m.points[m.indices[sec.id]]

        for pt in pts: # Iterate through 3d points in the section

            if (lastpt == somaPos).all(): # We calculate the distance of the current point from the soma
                currentLen += np.linalg.norm(np.diagonal(pt - lastpt))
            else:
                currentLen += np.linalg.norm(pt - lastpt)

            if (pt == lastpt).all(): # If the point is identical to the previous one, we add a small offset
                pt += 1e-3
                currentLen += 1e-5

            runningLen.append(currentLen)

            if np.any(points == np.Inf): # If this is the first point, we initialize the list of 3d positions
                points = pt
            else: # Otherwise, we append to the list
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

    return np.array(points), np.array(runningLen)


def interp_points_axon(axonPoints, runningLens, secName, numCompartments, somaPos):

    segPos = []

    if secName == 1: # First AIS section

        secLen = 30 # By construction, has length of 30 um
        segLen = secLen / numCompartments # Assumes each segment has the same length

        startPoint = 0
        endPoint = 30

        idx = np.where(runningLens <= endPoint)[0] # Finds indices of axon 3d points where cumulative length < 30 um

        if len(idx) > 0:

            axonRelevant = axonPoints[idx]

            lensRelevant = runningLens[idx] / secLen # Gets fraction of the total section length for each 3d point

            axonRelevant = np.vstack((somaPos.reshape((-1,3)),axonRelevant))
            lensRelevant = np.insert(lensRelevant,0,0)

        else:
            axonInterp = np.vstack((somaPos.reshape((-1,3)),axonPoints[0]))
            
            lensRelevant = np.array([0,runningLens[0] / secLen])

            lensInterp = lensRelevant # Gets fraction of the total section length for each 3d point

            fx = interp1d(lensInterp, axonInterp[:,0], kind='linear', fill_value='extrapolate')
            fy = interp1d(lensInterp, axonInterp[:,1], kind='linear', fill_value='extrapolate')
            fz = interp1d(lensInterp, axonInterp[:,2], kind='linear', fill_value='extrapolate')

            axonRelevant = np.array([fx(30/secLen),fy(30/secLen),fz(30/secLen)])
            axonRelevant = np.vstack((somaPos.reshape((-1,3)),axonRelevant))


    elif secName == 2: # Second AIS section

        secLen = 30 # Length is 30 um, by construction
        segLen = secLen / numCompartments

        startPoint = 30 # Cumulative length of first AIS section
        endPoint = 60 # Cumulative length of both AIS sections

        idx = np.intersect1d(np.where(runningLens <= endPoint), np.where(runningLens >= startPoint)) # Finds indices 3d points falling in this length bin

        if len(idx) >= 2:

            axonRelevant = axonPoints[idx]

            lensRelevant = (runningLens[idx] - startPoint) / secLen

        else: # If there aren't enough points, we estimate

            idxSmall = np.argmin(np.abs(runningLens - startPoint)) # Index closest to 30 um

            idxBig = np.argmin(np.abs(runningLens - endPoint)) # Index closest to 60 um

            
            if idxSmall == idxBig:
                 axonInterp = np.vstack((somaPos.reshape((-1,3)), axonPoints[idxBig]))
                 lensInterp = np.array([(0 - startPoint) / secLen,(runningLens[idxBig] - startPoint) / secLen])
            else:
                idx = [idxSmall, idxBig]

                axonInterp = axonPoints[idx]
                lensInterp = (runningLens[idx] - startPoint) / secLen

            fx = interp1d(lensInterp, axonInterp[:,0], kind='linear', fill_value='extrapolate')
            fy = interp1d(lensInterp, axonInterp[:,1], kind='linear', fill_value='extrapolate')
            fz = interp1d(lensInterp, axonInterp[:,2], kind='linear', fill_value='extrapolate')


            lensRelevant = np.array([0,1])
            axonRelevant = np.array([fx(lensRelevant),fy(lensRelevant),fz(lensRelevant)]).T



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

def getSimulationInfo(path_to_simconfig, newidx):

    '''
    Returns a list of 1000 node_ids and the corresponding compartment report
    '''

    rSim = bp.Simulation(path_to_simconfig)
    r = rSim.reports[list(rSim.reports.keys())[0]] # We assume that the compartment report is the only report produced by the simulation

    circuit = rSim.circuit

    population_name = r.population_names[0]

    report = r[population_name]
    nodeIds = report.node_ids

    try:
        ids = nodeIds[1000*newidx:1000*(newidx+1)]
    except:
        ids = nodeIds[1000*newidx:]


    data = report.get(group=ids,t_start=0,t_stop=r.dt)

    population = rSim.circuit.nodes[population_name]

    return ids, data, population

def getMorphology(population, i):

    morphName = population.get(i, 'morphology') # Gets name of the morphology file for node_id i

    with open(path_to_simconfig) as f:

        circuitpath = json.load(f)['network'] # path to circuit_config file

    with open(circuitpath) as f: # Gets path to morphology file from circuit_config

        js = json.load(f)

        basedir = js['manifest']['$BASE_DIR']
        morphpath = js['manifest']['$MORPHOLOGIES']
        finalmorphpath = basedir
        for m in morphpath.split('/')[1:]:
            finalmorphpath = finalmorphpath + '/'+m


    mImmutable = Morphology(finalmorphpath+'/ascii/'+morphName+'.asc') # Immutable MorphIO morphology object

    m = MutableMorph(mImmutable) # Mutable version, so that we can change the positions to orient the cell correctly within the circuit

    center = np.array([population.get(group=[i],properties='x'),population.get(group=[i],properties='y'),population.get(group=[i],properties='z')]) # Gets soma position

    center = center.flatten()

    ### Gets orientation of the cell in the circuit
    rotW = population.get(group=[i],properties='orientation_w')
    rotX = population.get(group=[i],properties='orientation_x')
    rotY = population.get(group=[i],properties='orientation_y')
    rotZ = population.get(group=[i],properties='orientation_z')
    ####

    ### Creates rotation quaternion for cell
    rotQuat = np.array([rotX,rotY,rotZ,rotW])
    rotQuat /= np.linalg.norm(rotQuat)
    rotation = R.from_quat(rotQuat.flatten())
    ###

    m.points = R.apply(rotation,m.points) # Rotates cell

    m.points += center # Translates cell

    return m, center

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


def main(path_to_simconfig, newidx,chunk_size, path_to_positions_folder):

    ids, data, population = getSimulationInfo(path_to_simconfig, newidx)


    colIdx = data.columns # node_id and Section IDs for each cell
    cols = np.array(list(data.columns))

    for idx, i in enumerate(ids): # Iterates through node_ids and gets segment positions

        m, center = getMorphology(population,i)
        somaPos = center[:,np.newaxis]

        axonPoints, runningLens = get_axon_points(m,center) # Gets 3d positions and cumulative length of the axon

        sections = np.unique(cols[np.where(cols[:,0]==i),1:].flatten()) # List of sections for the given neuron

        if idx ==0: # For the first cell, the array of positions xyz is initialized with the soma position
            xyz = somaPos.reshape(3,1)
        else:
            xyz = np.hstack((xyz,somaPos.reshape(3,1)))

        try:

            numSomas = len(data[i][0].iloc[0])
        except:
            numSomas = 1

        if numSomas > 1: # If there is more than one somatic segment, we assume that they all have the same position
            for k in np.arange(1,numSomas):
                xyz = np.hstack((xyz,somaPos.reshape(3,1)))


        for secName in list(sections[1:]):

            '''
            For each non-somatic segment, we interpolate the start and end point. Having the start and end, rather than the center, allows us to use either the
            line-source approximation (for LFP) or the reciprocity approach (for EEG). Note that for segments in the middle of a section, the end point is the
            same as the start point of the next section, so we do not need to save the end point, just the start point
            '''

            try:
                numCompartments = len(data[i][secName].iloc[0].values)
            except:
                numCompartments = 1

            if secName < 3: # Section 1 and Section 2 are always axonal sections

                segPos = interp_points_axon(axonPoints,runningLens,secName,numCompartments,somaPos)

            else:

                try: # Most other sections are dendritic, and are therefore included in the morphIO morphology object

                    secId = secName-1
                    ptIdx = m.indices[secId]
                    pts = m.points[ptIdx]

                    secPts = np.array(pts)

                    segPos = interp_points(secPts,numCompartments)

                except: # If the section is not in the morphIO object, then it is the myelinated part of the AIS

                    segPos = interp_points_axon(axonPoints,runningLens,secName,numCompartments,somaPos)


            xyz = np.hstack((xyz,segPos.T))

    newCols = getNewIndex(colIdx)

    positionsOut = pd.DataFrame(xyz,columns=newCols)

    positionsOut.to_pickle(path_to_positions_folder+'/' + str(int(newidx / chunk_size)) + '/positions'+str(newidx)+'.pkl')

if __name__=='__main__':

    path_to_simconfig = sys.argv[1] #simulation_condif with one-timestep simulation outputting a compartment report

    path_to_positions_folder = sys.argv[2]

    newidx = MPI.COMM_WORLD.Get_rank()
    chunk_size = int(sys.argv[3]) # Number of pickle files to write to each subfolder

    main(path_to_simconfig, newidx, chunk_size, path_to_positions_folder)
