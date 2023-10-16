import bluepy as bp
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
    def __init__(self, fn, h5, lst_gids, electrodes, **kwargs):
        self._fn = fn

        dset = h5.create_dataset("neuron_ids", data=sorted(lst_gids))
        for k, v in kwargs.items():
            dset.attrs.create(k, v)
        for k, electrode in electrodes.items():
            for item in electrode.items():
                h5.create_dataset("electrodes/" + k + '/' + item[0],
                              data=item[1])
        self._gids = np.array(lst_gids)

    def file(self):
        return h5py.File(self._fn, "r+")

    def lengths(self, gid):
        assert gid in self._gids
        return "lengths/" + str(int(gid))

    def offsets(self, gid):
        assert gid in self._gids
        return "offsets/" + str(int(gid))

    def weights(self, gid, idx):
        assert gid in self._gids

        folder = int(idx/50)
        return '/electrodes/electrode_grid/'+str(int(gid))


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

def writer_factory(circ, seg_counts):
    def write_neuron(h5, file, gid, electrode_struc,sec_ids,idx):
        m = circ.morph.get(gid, transform=True)
        count_dict = seg_counts.loc[gid].values[0][0]


        count_dict = dict(zip(count_dict[0], count_dict[1]))

        in_L, in_offsets = morph_stats_for_neuron(m)
        out_offsets = offsets_for_compartments(count_dict, in_offsets)


        file.create_dataset(h5.offsets(gid), data=out_offsets)

        elecIdx = 0


        file.create_dataset(h5.weights(gid, idx), data=np.ones([len(sec_ids.values),len(electrode_struc.items())+1]))


        file.create_dataset('sec_ids/'+str(gid),shape=len(sec_ids.values),data=sec_ids.values)
    return write_neuron

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

def applyParallel(dfGrouped, func):
    retLst = Parallel(n_jobs=multiprocessing.cpu_count())(delayed(func)(group) for name, group in dfGrouped)
    return pd.concat(retLst)

def getAtlasInfo(BlueConfig,electrodePositions):


    bluefile = open(BlueConfig,'r')
    bluelines = bluefile.readlines()
    bluefile.close()

    for line in bluelines:
        if 'Atlas' in line:
            atlasName = line.split('Atlas ')[-1].split('\n')[0]
            break

    atlas = Atlas.open(atlasName)
    brain_regions = atlas.load_data('brain_regions')

    region_map = atlas.load_region_map()

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
        electrodes[name] = {'location': positions[i],'type': types[i],
        'region':regions[i],'layer':layers[i],'offset':i}

    return electrodes



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

    sim = bp.Simulation(path_to_blueconfig)

    circ = sim.circuit


    rep = sim.report('Current')
    col = 'hex0'
    g = circ.cells.ids({'$target':col})

    data = rep.get(t_start=rep.t_start, t_end=rep.t_start + rep.t_step,gids=g)


    f = data.columns.to_frame()
    f.index = range(len(f))


    seg_counts = f.groupby("gid").apply(count_segments)



    electrodes = makeElectrodeDict(nameList,electrodePositions,typeList,regionList,layerList)

    filename = outputfolder+'/coeffs'+electrodeName+'.h5'

    h5file = h5py.File(filename,'w')

    h5id = h5file.id
    cc = h5id.get_mdc_config()
    cc.max_size = 1024*1024*124
    h5id.set_mdc_config(cc)


    h5 = ElectrodeFileStructure(filename, h5file, g, electrodes, circuit=circ.config["cells"])

    write_neuron = writer_factory(circ, seg_counts)
    secIds = pd.DataFrame(data=f.values[:,1],index=f.values[:,0])


    for i, gid in enumerate(g):

        write_neuron(h5, h5file, gid, electrodes,secIds.loc[gid],i)

    h5file.close()

def repositionElectrode(probe, path_to_Blueconfig):

    '''
    Aligns probe with center of cortical column
    '''

    c = bp.Circuit(path_to_Blueconfig)
    somaPos = c.cells.get({'$target': 'hex0'},properties=[bp.Cell.X, bp.Cell.Y, bp.Cell.Z])
    center = np.mean(somaPos,axis=0).values


    pca = PCA(n_components=3)
    pca.fit(somaPos)
    main_axis = pca.components_[0]

    elevation = np.arctan2(np.sqrt(main_axis[0]**2+main_axis[1]**2),main_axis[2])
    azimuth = np.arctan2(main_axis[1],main_axis[0])

    probe.rotate([0,1,0],elevation*180/np.pi)
    probe.rotate([0,0,1],azimuth*180/np.pi)
    probe.move(center)

    return(probe)

if __name__=='__main__':


    probe_name = sys.argv[1]

    path_to_Blueconfig = sys.argv[3]
    inputfolder = sys.argv[4]
    outputfolder = sys.argv[5]

    probe = MEA.return_mea(probe_name)

    repositionElectrode(probe,path_to_Blueconfig)

    electrodePositions = probe.positions

    type = sys.argv[2] # Either EEG or LFP

    names = []
    types = []
    for i in range(len(electrodePositions)):
        names.append(probe_name+'_'+str(i))
        types.append(type)

    numFilesPerFolder = int(sys.argv[6]) # Number of files per folder in the cell position data folder

    sigma = 0.277 # Conductivity of brain. Used for LFP only
    path_to_fields = None # Path to finite elemnt results. Used for EEG only

    if len(sys.argv)>7:
        sigma = float(sys.argv[7])
        if len(sys.argv)>8:
            path_to_fields = sys.argv[8]


    writeH5File(path_to_Blueconfig,inputfolder,outputfolder,electrodePositions,names,types,numFilesPerFolder,sigma,path_to_fields)
