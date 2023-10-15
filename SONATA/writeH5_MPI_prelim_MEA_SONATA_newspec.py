import libsonata as lb
import numpy as np
import h5py
import os
import tqdm
import pandas as pd
import sys
from scipy.spatial import distance
from scipy.spatial.transform import Rotation
import time
from voxcell.nexus.voxelbrain import Atlas
import datetime
import multiprocessing
import MEAutility as MEA
from sklearn.decomposition import PCA

class ElectrodeFileStructure(object):
    def __init__(self, fn, h5, lst_gids, electrodes, population_name, **kwargs):
        self._fn = fn

        dset = h5.create_dataset(population_name+"/node_ids", data=sorted(lst_gids))
        for k, v in kwargs.items():
            dset.attrs.create(k, v)

        index = 0
        for k, electrode in electrodes.items():

            h5.create_dataset("electrodes/" + k + '/'+population_name,data=index)
            index += 1

            for item in electrode.items():
                h5.create_dataset("electrodes/" + k + '/' + item[0],
                              data=item[1])
        self._gids = np.array(lst_gids)

    def file(self):
        return h5py.File(self._fn, "r+")

    def lengths(self, gid):
        assert gid in self._gids
        return "lengths/" + str(int(gid))

    def offsets(self,population_name):
        return population_name+"/offsets"

    def weights(self, population_name):

        return '/electrodes/'+population_name+'/scaling_factors'


def morph_stats_for_neuron(m):
    out_L = [10.0]
    for sec in m.sections:
        out_L.append(np.sum(np.diff(sec.points, axis=0) ** 2, axis=1))
    out_L = np.hstack(out_L)
    out_offsets = np.hstack([0, 1, 1 + np.cumsum(np.diff(m.section_offsets) - 1).astype(int)])
    return out_L, out_offsets


def weights_for_neuron(m, location):
    out_W = [1. / distance.cdist(m.soma.center.reshape((1, 3)), location)]
    for sec in m.sections:
        midpoints = 0.5 * (sec.points[:-1] + sec.points[1:])
        out_W.append(1. / distance.cdist(midpoints, location))
    out_W = np.vstack(out_W)
    return out_W


def offsets_for_compartments(count_dict, in_offsets):
    out_offsets = np.zeros_like(in_offsets, dtype=int)
    for sec_id in range(len(in_offsets) - 1):
        out_offsets[sec_id + 1] = out_offsets[sec_id] + count_dict.get(sec_id, 0)
    return out_offsets


def weights_for_compartments(count_dict, in_offsets, in_weights, in_lengths):
    out_W = []
    for sec_id in range(len(in_offsets) - 1):
        if sec_id in count_dict:
            sec_len = count_dict[sec_id]
            lst_weights = in_weights[in_offsets[sec_id]:in_offsets[sec_id + 1]]
            lst_lengths = in_lengths[in_offsets[sec_id]:in_offsets[sec_id + 1]]
            out = ad_hoc_interpolation(lst_weights, lst_lengths, sec_len)
            out_W.append(out)
    return np.vstack(out_W)

def ad_hoc_interpolation(lst_weights, lst_lengths, n_seg):
    nrmlz_len = np.hstack([0.0, np.cumsum(lst_lengths)]) / np.sum(lst_lengths)
    splts = np.linspace(0.0, 1.0, n_seg + 1)

    out = []
    for a, b in zip(splts[:-1], splts[1:]):
        w1 = np.minimum(1.0,
                           np.maximum(0.0,
                                         (nrmlz_len[1:] - a) / (nrmlz_len[1:] - nrmlz_len[:-1])
                                         )
                           )
        w2 = np.minimum(1.0,
                           np.maximum(0.0,
                                         (b - nrmlz_len[:-1]) / (nrmlz_len[1:] - nrmlz_len[:-1])
                                         )
                           )
        w_use = (lst_lengths * w1 * w2).reshape((len(w1), 1))
        out.append(np.sum(lst_weights * w_use, axis=0) / np.sum(w_use))
    return np.array(out)

def writer_factory(f, population_name):
    
    def write_all_neuron(h5, file, electrode_struc, sec_ids):
        
        file.create_dataset(h5.weights(population_name), data=np.zeros([len(f['gid'].values),len(electrode_struc.items())]))
        
        unique, counts = np.unique(f['gid'].values,return_counts=True)
        
        out_offsets = np.hstack((np.array([0]),np.cumsum(counts)[:-1]))
        

        file.create_dataset(h5.offsets(population_name), data=out_offsets)

        #file.create_dataset(population_name+'/sec_ids',shape=len(sec_ids.values),data=sec_ids.values)
    
    return write_all_neuron

def add_data(h5, gid, coeffs, electrode_struc):
    with h5.file() as h5_file:

        i = 0
        for elec_name, elec_location in electrode_struc.items():
            dset = h5.weights(elec_name, gid)
            h5_file[dset] = coeffs[i].values
            i += 1

def count_segments(in_data):


    gid = in_data.values[0,0]

    in_data = in_data[in_data.columns[1]]
    bins = np.hstack([np.unique(in_data), np.max(in_data) + 1]).astype(int)

    df = pd.DataFrame.from_dict({gid:[np.vstack((bins[:-1], np.histogram(in_data, bins=bins)[0].astype(int)))]},orient='index')

    df.index.name = 'gid'

    return df
    #return np.vstack((bins[:-1], np.histogram(in_data, bins=bins)[0].astype(int)))

def applyParallel(dfGrouped, func):
    retLst = Parallel(n_jobs=multiprocessing.cpu_count())(delayed(func)(group) for name, group in dfGrouped)
    return pd.concat(retLst)

def getAtlasInfo(BlueConfig,electrodePositions):


#    atlas = Atlas.open('/gpfs/bbp.cscs.ch/project/proj83/jira-tickets/NSETM-1948-extract-hex-O1/data/O1_data/atlas')
 #   brain_regions = atlas.load_data('flatmap')

  #  region_map = atlas.load_region_map()

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


    print(regionList,flush=True)
    return regionList, layerList

def makeElectrodeDict(names,positions,types,regions,layers):

    electrodes = {}


    for i, name in enumerate(names):
        electrodes[name] = {'position': positions[i],'type': types[i],
        'region':regions[i],'layer':layers[i]}

#        for population in population_offsets[i]:

#            population_name = population['name']
#            offset = population['offset']

#            electrodes[name][population_name] = {'offsets':offset}

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

    for i in range(len(positions.columns)):

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

    coeffs = pd.DataFrame(data=coeffs, columns=columns)

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



def writeH5File(path_to_blueconfig,inputfolder,outputfolder,electrodePositions,electrodeNames,electrodeType,numFilesPerFolder,sigma=0.277,path_to_fields=None):

    '''
    path_to_blueconfig refers to the BlueConfig from the 1-timestep simulation used to get the segment positions
    inputfile refers to the path to the pickle file containing the potential at each segment. This is the output of the interpolation script
    electrodePositions is a list containing the positions (in 3D cartesian space) of the recording and reference electrodes (if EEG)
    gidList is a list of the desired gids
    '''



    regionList, layerList = getAtlasInfo(path_to_blueconfig, electrodePositions)

    electrodeName = electrodeNames[0].split('_')[0]

    nameList = electrodeNames
    typeList = electrodeType
    
    r = lb.ElementReportReader('reporting/voltage.h5')

    population_name = r.get_population_names()[0]
    r = r[population_name]
    g = r.get_node_ids()

    data_frame = r.get(node_ids=g,tstart=0,tstop=0.1)

    data = pd.DataFrame(data_frame.data, columns=pd.MultiIndex.from_tuples(tuple(map(tuple,data_frame.ids)), names=['gid','section']), index=data_frame.times)


    f = data.columns.to_frame()
    f.index = range(len(f))

    #seg_counts = applyParallel(f.groupby("gid"), count_segments)
    
    
    seg_counts = f.groupby("gid").apply(count_segments)
    
    
    #seg_counts = groupby_to_series_to_frame(f,count_segments,n_jobs=8, use_apply=False, by="gid")

#    population_offsets = []
#    for i in range(len(nameList)):
#        population_offsets.append([{'name':population_name,'offset':0}])

    electrodes = makeElectrodeDict(nameList,electrodePositions,typeList,regionList,layerList)

    filename = outputfolder+'/coeffs'+electrodeName+'.h5'

    h5file = h5py.File(filename,'w')

    h5id = h5file.id
    cc = h5id.get_mdc_config()
    cc.max_size = 1024*1024*124
    h5id.set_mdc_config(cc)

    h5 = ElectrodeFileStructure(filename, h5file, g, electrodes, population_name, circuit='/gpfs/bbp.cscs.ch/project/proj83/jira-tickets/NSETM-1948-extract-hex-O1/data/O1_data/S1nonbarrel_neurons/nodes.h5')

    write_all_neuron = writer_factory(f, population_name)
    secIds = pd.DataFrame(data=f.values[:,1],index=f.values[:,0])


    write_all_neuron(h5, h5file, electrodes,secIds)

    h5file.close()

if __name__=='__main__':
 

    probe_name = sys.argv[1]

    path_to_Blueconfig = sys.argv[3]
    inputfolder = sys.argv[4]
    outputfolder = sys.argv[5]

    probe = MEA.return_mea(probe_name)
    
    nodes = lb.NodeStorage('/gpfs/bbp.cscs.ch/project/proj83/jira-tickets/NSETM-1948-extract-hex-O1/data/O1_data/S1nonbarrel_neurons/nodes.h5')
    population = nodes.open_population(list(nodes.population_names)[0])
    
    r = lb.ElementReportReader('reporting/voltage.h5')
    r = r[r.get_population_names()[0]]
    node_ids = r.get_node_ids()
    
    somaPos = np.array([population.get_attribute('x',node_ids),population.get_attribute('y',node_ids),population.get_attribute('z',node_ids)])
    

    center = np.mean(somaPos,axis=1)


    pca = PCA(n_components=3)
    pca.fit(somaPos)
    main_axis = pca.components_[0]

    elevation = np.arctan2(np.sqrt(main_axis[0]**2+main_axis[1]**2),main_axis[2])
    azimuth = np.arctan2(main_axis[1],main_axis[0])

    probe.rotate([0,1,0],elevation*180/np.pi)
    probe.rotate([0,0,1],azimuth*180/np.pi)
    probe.move(center)

    electrodePositions = probe.positions

    type = sys.argv[2]

    names = []
    types = []
    for i in range(len(electrodePositions)):
        names.append(probe_name+'_'+str(i))
        types.append(type)

    numFilesPerFolder = int(sys.argv[6])

    sigma = 0.277
    path_to_fields = None

    if len(sys.argv)>7:
        sigma = float(sys.argv[7])
        if len(sys.argv)>8:
            path_to_fields = sys.argv[8]


    writeH5File(path_to_Blueconfig,inputfolder,outputfolder,electrodePositions,names,types,numFilesPerFolder,sigma,path_to_fields)
