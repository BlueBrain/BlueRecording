# SPDX-License-Identifier: GPL-3.0-or-later
import numpy as np
from morphio import SectionType
from morphio import Morphology
import libsonata as lb
import bluepysnap as bp

import pandas as pd
import sys
from scipy.interpolate import interp1d
from mpi4py import MPI
import warnings
import json
from scipy.spatial.transform import Rotation as R
from .utils import *
import os
import math

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

        #### self.indices is a list of lists, where self.indices[i] is a list containing the indices of the 3d points for the ith section. The soma is not included because it is not part of morphioImmutableObject.sections
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

        distances = np.cumsum(np.linalg.norm(np.diff(coords,axis=0),axis=1))
        distances /= distances[-1]
        distances = np.insert(distances,0,0)

        f = interp1d(distances, coords[:, dim], kind='linear')
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

    points = somaPos.reshape(-1,3)
    runningLen = [0]


    for x in idxs: # We iterate through the selected axonal sections

        sec = m.sections[x]

        pts = m.points[m.indices[sec.id]]

        for pt in pts: # Iterate through 3d points in the section

            if (lastpt == somaPos).all(): # We calculate the distance of the current point from the soma
                currentLen += np.linalg.norm(pt.flatten() - lastpt.flatten())
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

    axonPoints, indices = np.unique(np.array(points),axis=0,return_index=True)

    return axonPoints, np.array(runningLen)[indices] # We delete duplicate points that may occur if the morphology is in a format where section start and end points are repeated


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


        if len(axonRelevant) < 2: # If there are not enough points, we use the soma position (which would be included in the axon point list) and the first real point in the axon
            idx = 0

            axonRelevant = axonPoints[:2]

            lensRelevant = runningLens[:2] / secLen


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

            if idxSmall == idxBig: # If these two points are the same, we use different points
                if idxBig < len(runningLens)-1:
                    idxBig += 1
                else:
                    idxSmall -= 1 # If the two points are identical, then idxSmall can never be zero, since otherwise this would imply a one-point axon

            idx = [idxSmall, idxBig]

            axonRelevant = axonPoints[idx]
            lensRelevant = (runningLens[idx] - startPoint) / secLen


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

def remove_variables(js, finalmorphpath):

    '''
    Removes references to variables in path to morphology
    Assumes that all references are in the manifest section
    '''

    while '$' in finalmorphpath:

        elements = finalmorphpath.split('/')

        for i, element in enumerate(elements):

            if '$' in element:
                element = js['manifest'][element]
                
            if i == 0:
                finalmorphpath = element
            else:
                finalmorphpath = finalmorphpath + '/' + element

    return finalmorphpath

def tryFileNames(morphName, finalmorphpath):

    asc = finalmorphpath+'/ascii/'+morphName+'.asc'
    asc1 = finalmorphpath+'/'+morphName+'.asc'
    asc2 = finalmorphpath+'/morphologies_asc/'+morphName+'.asc'

    swc = finalmorphpath+'/swc/'+morphName+'.swc'
    swc1 = finalmorphpath+'/'+morphName+'.swc'
    swc2 = finalmorphpath+'/morphologies_swc/'+morphName+'.swc'

    options = [asc, asc1, asc2, swc, swc1, swc2]

    for option in options:
        if os.path.exists(option):
            fileName = option
            break

    return fileName

def get_morph_path(population, i, path_to_simconfig):

    morphName = population.get(i, 'morphology') # Gets name of the morphology file for node_id i

    circuitpath = getCircuitPath(path_to_simconfig) # path to circuit_config file

    with open(circuitpath) as f: # Gets path to morphology file from circuit_config

        js = json.load(f)

        if 'components' in js.keys() and 'morphologies_dir' in js['components'].keys():
            finalmorphpath = js['components']['morphologies_dir']

        else:
            finalmorphpath = js['manifest']['$MORPHOLOGIES']

        finalmorphpath = remove_variables(js, finalmorphpath)

        finalmorphpath = concretize_path(circuitpath,finalmorphpath) # Goes from relative to absolute path

    fileName = tryFileNames(morphName, finalmorphpath)

    return fileName


def getMorphology(population, i, path_to_simconfig):


    finalmorphpath = get_morph_path(population, i, path_to_simconfig)

    mImmutable = Morphology(finalmorphpath) # Immutable MorphIO morphology object

    m = MutableMorph(mImmutable) # Mutable version, so that we can change the positions to orient the cell correctly within the circuit

    m, center = positionMorphology(m, population, i)

    return m, center

def get_transform(population, i):

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

    return center, rotation

def apply_transform(m, center, rotation):

    m.points = R.apply(rotation,m.points) # Rotates cell

    m.points += center # Translates cell

    return m

def positionMorphology(m, population, i):

    center, rotation = get_transform(population, i)

    m = apply_transform(m, center, rotation)

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

def checkAxonsFirst(morphology):

    '''
    Checks if the morphology file is has axons first (like with cortical neurons)
    or dendrites first (like thalamic neurons)
    '''

    if morphology.sections[0].type == 2:
        axonFirst = True
    else:
        axonFirst = False

    return axonFirst

def getPositions(path_to_simconfig, neurons_per_file, files_per_folder, path_to_positions_folder,replace_axons=True):

    '''
    path_to_simconfig refers to the BlueConfig from the 1-timestep simulation used to get the segment positions
    path_to_positions_folder refers to the path to the top-level folder containing pickle files with the position of each segment.
    neurons_per_file is the number of neurons in each of the segment positions pickle files.
    files_per_folder is the number of positions pickle files in each subfolder in segment_position_folder. This parameter is used in order to avoid stressing the file system with too many files in a given folder
    '''

    newidx = MPI.COMM_WORLD.Get_rank()

    report, nodeIds = getSimulationInfo(path_to_simconfig)
    population = getPopulationObject(path_to_simconfig)

    if len(nodeIds)/neurons_per_file > MPI.COMM_WORLD.Get_size():
        raise AssertionError("Make sure that enough processes have been allocated to write position files")

    try:
        ids = nodeIds[neurons_per_file*newidx:neurons_per_file*(newidx+1)]
    except:
        ids = nodeIds[neurons_per_file*newidx:]

    if len(ids) == 0:
        return 1

    data = getMinimalReport(report,ids)

    colIdx = data.columns # node_id and Section IDs for each cell
    cols = np.array(list(data.columns))

    for idx, i in enumerate(ids): # Iterates through node_ids and gets segment positions

        m, center = getMorphology(population,i, path_to_simconfig)

        axonsFirst = checkAxonsFirst(m)

        somaPos = center[:,np.newaxis]

        if replace_axons: # If the axons are replaced by a stub axon, we need to get the positions thereof

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

        morphSectionIdx = 0 # Index used if morphology file lists dendrites before axons

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

            if secName < 3 and replace_axons: # Section 1 and Section 2 are always axonal sections, if the axon is being replaced

                segPos = interp_points_axon(axonPoints,runningLens,secName,numCompartments,somaPos)

            elif axonsFirst:

                secId = secName - 1

                if secId >= len(m.indices): # If the section is not in the morphIO object, then it is the myelinated part of the AIS

                    segPos = interp_points_axon(axonPoints,runningLens,secName,numCompartments,somaPos)

                else: # Most other sections are dendritic, and are therefore included in the morphIO morphology object

                    ptIdx = m.indices[secId]
                    pts = m.points[ptIdx]

                    secPts = np.array(pts)

                    segPos = interp_points(secPts,numCompartments)

            else:

                if m.sections[morphSectionIdx].type == 3 or m.sections[morphSectionIdx].type == 4: # If dendrite

                    ptIdx = m.indices[morphSectionIdx]
                    pts = m.points[ptIdx]

                    secPts = np.array(pts)

                    segPos = interp_points(secPts,numCompartments)

                    morphSectionIdx += 1

                elif m.sections[morphSectionIdx].type == 2: # If axon

                    segPos = interp_points_axon(axonPoints,runningLens,secName,numCompartments,somaPos)

                else:
                    raise ValueError('Non-axonal and non-dendritic section found in morphology file')

            xyz = np.hstack((xyz,segPos.T))

    newCols = getNewIndex(colIdx)

    positionsOut = pd.DataFrame(xyz,columns=newCols)

    positionsOut.to_pickle(path_to_positions_folder+'/' + str(int(newidx / files_per_folder)) + '/positions'+str(newidx)+'.pkl')
