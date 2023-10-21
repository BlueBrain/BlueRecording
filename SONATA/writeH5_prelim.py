import numpy as np
import h5py
import os
import pandas as pd
import sys
import bluepysnap as bp
import json

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


def writer_factory(f, population_name):

    def write_all_neuron(h5, file, electrode_struc, sec_ids):

        file.create_dataset(h5.weights(population_name), data=np.zeros([len(f['gid'].values),len(electrode_struc.items())]))

        unique, counts = np.unique(f['gid'].values,return_counts=True)

        out_offsets = np.hstack((np.array([0]),np.cumsum(counts)[:-1]))

        file.create_dataset(h5.offsets(population_name), data=out_offsets)

    return write_all_neuron


def makeElectrodeDict(electrode_csv,type,region):

    electrode_df = pd.read_csv(electrode_csv,header=0,index_col=0)

    electrodes = {}

    for i in range(len(electrode_df.values)):

        if 'electrode' in electrode_df.columns:

            name = electrode_df['electrode'][i]

        else:

            name = str(i)

        position = np.array([electrode_df['x'][i],electrode_df['y'][i],electrode_df['z'][i]])

        if 'layer' in electrode_df.columns:

            layer = electrode_df['layer'][i]

        else:

            layer = "Not provided"

        electrodes[name] = {'position': position,'type': type,
        'region':region,'layer':layer}


    return electrodes

def writeH5File(path_to_simconfig,inputfolder,outputfile,electrode_csv,electrodeType,region):

    '''
    path_to_blueconfig refers to the BlueConfig from the 1-timestep simulation used to get the segment positions
    inputfile refers to the path to the pickle file containing the potential at each segment. This is the output of the interpolation script
    electrodePositions is a list containing the positions (in 3D cartesian space) of the recording and reference electrodes (if EEG)
    gidList is a list of the desired gids
    '''

    with open(path_to_simconfig) as f:

        circuitpath = json.load(f)['network']

    r1 = bp.Simulation(path_to_simconfig)
    r = r1.reports[list(r1.reports.keys())[0]]

    circuit = r1.circuit

    population_name = r.population_names[0]

    report = r[population_name]
    nodeIds = report.node_ids

    data = report.get(group=nodeIds,t_start=0,t_stop=r.dt)
    data.columns = data.columns.rename('gid',level=0)
    data.columns = data.columns.rename('section',level=1)


    f = data.columns.to_frame()
    f.index = range(len(f))


    electrodes = makeElectrodeDict(electrode_csv,electrodeType,region)

    h5file = h5py.File(outputfile,'w')

    h5id = h5file.id
    cc = h5id.get_mdc_config()
    cc.max_size = 1024*1024*124
    h5id.set_mdc_config(cc)

    h5 = ElectrodeFileStructure(outputfile, h5file, nodeIds, electrodes, population_name, circuit=circuitpath)

    write_all_neuron = writer_factory(f, population_name)
    secIds = pd.DataFrame(data=f.values[:,1],index=f.values[:,0])


    write_all_neuron(h5, h5file, electrodes,secIds)

    h5file.close()


if __name__=='__main__':

    electrode_csv = sys.argv[1]

    type = sys.argv[2]

    region = sys.argv[3]

    path_to_simconfig = sys.argv[4]

    segment_position_folder = sys.argv[5]
    outputfile = sys.argv[6]

    writeH5File(path_to_simconfig,segment_position_folder,outputfile,electrode_csv,type,region)
