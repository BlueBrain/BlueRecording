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

    def __init__(self,morphioMorph):

        for attr in dir(morphioMorph):
            if '__' not in attr:
                setattr(self,attr,getattr(morphioMorph,attr))

        self.indices = []
        index = 0
        for section in self.sections:
            self.indices.append([])

            for i in range(len(section.points)):
                self.indices[-1].append(index)
                index += 1


def interp_points(coords, ncomps):
    xyz = np.array([]).reshape(ncomps + 1, 0)
    npoints = coords.shape[0]

    for dim in range(coords.shape[1]):
        f = interp1d(np.linspace(0, 1, npoints), coords[:, dim], kind='linear')
        ic = f(np.linspace(0, 1, ncomps + 1)).reshape(ncomps + 1, 1)
        xyz = np.hstack((xyz, ic))

    return xyz


def get_axon_points(m,center):
    targetLength = 1060

    points = np.Inf

    currentLen = 0
    runningLen = []

    somaPos = center[:,np.newaxis]

    needExtension = False

    for sec in m.sections:

        if sec.type == SectionType.axon and len(sec.children) == 0:

            length = 0
            longestLength = 0
            thisSec = sec
            idxs = []

            idxs.append(thisSec.id)

            while not thisSec.is_root:
                newSec = thisSec.parent
                length += np.linalg.norm(m.points[m.indices[newSec.id][-1]] - m.points[m.indices[thisSec.id][-1]])

                thisSec = newSec
                idxs.append(thisSec.id)

            length += np.linalg.norm(m.points[m.indices[thisSec.id][-1]][:,np.newaxis] - somaPos)

            if length > longestLength:
                longestLength = length
                bestIdx = idxs

            if length > 1060:
                break

    if length < 1060:
        idxs = bestIdx
        needExtension = True

    idxs.reverse()

    lastpt = somaPos

    for x in idxs:

        sec = m.sections[x]

        pts = m.points[m.indices[sec.id]]


        for pt in pts:

            if (lastpt == somaPos).all():


                currentLen += np.linalg.norm(pt[:,np.newaxis] - lastpt)
            else:
                currentLen += np.linalg.norm(pt - lastpt)



            if (pt == lastpt).all():
                pt += 1e-3
                currentLen += 1e-5

            runningLen.append(currentLen)

            if np.any(points == np.Inf):
                points = pt
            else:
                points = np.vstack((points, pt))

            lastpt = pt

            if currentLen > targetLength:
                break

        if currentLen > targetLength:
            break


    if needExtension:
        toAdd = targetLength - currentLen

        slopes = (points[-1] - points[-2]) / (runningLen[-1] - runningLen[-2])

        newpt = points[-1] + slopes * toAdd

        points = np.vstack((points, newpt))

        currentLen += np.linalg.norm(newpt - points[-2])

        runningLen.append(currentLen)

    return np.array(points), np.array(runningLen)


def interp_points_axon(axonPoints, runningLens, secName, numCompartments, somaPos):
    segPos = []



    if secName == 1:

        secLen = 30
        segLen = secLen / numCompartments

        startPoint = 0
        endPoint = 30

        idx = np.where(runningLens <= endPoint)

        axonRelevant = axonPoints[idx]

        axonRelevant = np.vstack((somaPos.reshape((-1,3)),axonPoints[idx]))

        lensRelevant = runningLens[idx] / secLen

        lensRelevant = np.hstack((0,lensRelevant))

        if len(axonRelevant) < 2:
            idx = 0

            axonRelevant = np.vstack((somaPos.reshape((-1,3)), axonPoints[idx]))

            lensRelevant = np.array([0, runningLens[idx] / secLen])


    elif secName == 2:

        secLen = 30
        segLen = secLen / numCompartments

        startPoint = 30
        endPoint = 60

        idx = np.intersect1d(np.where(runningLens <= endPoint), np.where(runningLens >= startPoint))

        axonRelevant = axonPoints[idx]

        lensRelevant = (runningLens[idx] - startPoint) / secLen

        if len(axonRelevant) < 2:

            idxSmall = np.argmin(np.abs(runningLens - startPoint))

            idxBig = np.argmin(np.abs(runningLens - endPoint))

            idx = [idxSmall, idxBig]

            axonRelevant = axonPoints[idx]
            lensRelevant = (runningLens[idx] - startPoint) / secLen

            if idxSmall == idxBig:
                axonRelevant = np.vstack((somaPos.reshape((-1,3)), axonPoints[idxBig]))
                lensRelevant = np.array([0, (runningLens[idxBig] - startPoint) / secLen])


    else:

        secLen = 1000
        segLen = secLen / numCompartments

        startPoint = 60
        endPoint = 1060

        idx = np.where(runningLens >= startPoint)[0]

        if len(idx) == 1:
            idx = [idx[0] - 1, idx[0]]

        axonRelevant = axonPoints[idx]

        lensRelevant = (runningLens[idx] - startPoint) / secLen

    for i in range(numCompartments+1):
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

def main(path_to_simconfig, newidx,chunk_size):


    rSim = bp.Simulation(path_to_simconfig)
    r = rSim.reports[list(rSim.reports.keys())[0]]

    circuit = rSim.circuit

    population_name = r.population_names[0]

    report = r[population_name]
    nodeIds = report.node_ids

    try:
        ids = nodeIds[1000*newidx:1000*(newidx+1)]#{'$target':colName,Cell.SYNAPSE_CLASS:'EXC',Cell.LAYER:5})
    except:
        ids = nodeIds[1000*newidx:]


    data = report.get(group=ids,t_start=0,t_stop=r.dt)


    population = rSim.circuit.nodes[population_name]

    colIdx = data.columns
    cols = np.array(list(data.columns))

    for idx, i in enumerate(ids):

        try:

            numSomas = len(data[i][0].iloc[0])
        except:
            numSomas = 1


        num = 0


        morphName = population.get(i, 'morphology')
        
        with open(path_to_simconfig) as f:

            circuitpath = json.load(f)['network']
            
        with open(circuitpath) as f:
            
            js = json.load(f)
            
            basedir = js['manifest']['$BASE_DIR']
            morphpath = js['manifest']['$MORPHOLOGIES']
            finalmorphpath = basedir 
            for m in morphpath.split('/')[1:]:
                finalmorphpath = finalmorphpath + '/'+m


        mImmutable = Morphology(finalmorphpath+'/ascii/'+morphName+'.asc')

        m = MutableMorph(mImmutable)

        center = np.array([population.get(group=[i],properties='x'),population.get(group=[i],properties='y'),population.get(group=[i],properties='z')])

        center = center.flatten()
        
        rotW = population.get(group=[i],properties='orientation_w')
        rotX = population.get(group=[i],properties='orientation_x')
        rotY = population.get(group=[i],properties='orientation_y')
        rotZ = population.get(group=[i],properties='orientation_z')

        rotQuat = np.array([rotX,rotY,rotZ,rotW])
        rotQuat /= np.linalg.norm(rotQuat)

        rotation = R.from_quat(rotQuat.flatten())

        
        m.points = R.apply(rotation,m.points)

        m.points += center

        axonPoints, runningLens = get_axon_points(m,center)

        sections = np.unique(cols[np.where(cols[:,0]==i),1:].flatten())


        somaPos = center[:,np.newaxis]


        if idx ==0:
            xyz = somaPos.reshape(3,1)
        else:
            xyz = np.hstack((xyz,somaPos.reshape(3,1)))

        if numSomas > 1:
            for k in np.arange(1,numSomas):
                xyz = np.hstack((xyz,somaPos.reshape(3,1)))


        for secName in list(sections[1:]):


            try:
                numCompartments = len(data[i][secName].iloc[0].values)
            except:
                numCompartments = 1

            if secName < 3:

                segPos = interp_points_axon(axonPoints,runningLens,secName,numCompartments,somaPos)

            else:

                try:

                    secId = secName-1
                    ptIdx = m.indices[secId]
                    pts = m.points[ptIdx]

                    secPts = np.array(pts)

                    segPos = interp_points(secPts,numCompartments)

                except:

                    segPos = interp_points_axon(axonPoints,runningLens,secName,numCompartments,somaPos)


            xyz = np.hstack((xyz,segPos.T))



    newIdx = []

    for i, col in enumerate(colIdx):


        newIdx.append(col)

        if i == len(colIdx)-1:
            newIdx.append(col)
        elif col[-1]!=0 and colIdx[i+1]!=col:
            newIdx.append(col)

    newCols = pd.MultiIndex.from_tuples(newIdx, names=colIdx.names)


    positionsOut = pd.DataFrame(xyz,columns=newCols)

    positionsOut.to_pickle(path_to_positions+'/' + str(int(newidx / chunk_size)) + '/positions'+str(newidx)+'.pkl')

if __name__=='__main__':

    path_tosimconfig = sys.argv[1]
    path_to_positions = sys.argv[2]
    newidx = MPI.COMM_WORLD.Get_rank()
    chunk_size = int(sys.argv[3])
    main(path_tosimconfig, newidx, chunk_size)
