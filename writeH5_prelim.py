import bluepy as bp
import numpy as np
import h5py
import pandas as pd
import sys

class ElectrodeFileStructure(object):

    '''
    This class writes datasets to the h5 file
    '''

    def __init__(self, h5, lst_gids, electrodes, **kwargs):

        '''
        fn: filename
        h5: h5 file returned by h5py.File(filename,'w')
        **kwargs: currently expected to take the circuit path
        '''

        dset = h5.create_dataset("neuron_ids", data=sorted(lst_gids))

        for k, v in kwargs.items():
            dset.attrs.create(k, v)

        ### Iterates through electrode dictionary to write metadata
        for k, electrode in electrodes.items(): # Iterates through electrodes
            for item in electrode.items(): # Iterates through metadata fields for each electrode
                h5.create_dataset("electrodes/" + k + '/' + item[0],
                              data=item[1])

        self._gids = np.array(lst_gids)


    def weights(self, gid):
        assert gid in self._gids

        return '/electrodes/electrode_grid/'+str(int(gid))


def write_neuron(h5, file, gid, electrode_struc,sec_ids):


    file.create_dataset(h5.weights(gid), data=np.ones([len(sec_ids.values),len(electrode_struc.items())+1])) # Initializes the coefficient dataset for the gid, with a matrix of ones of size (number_of_segments x number_of_electrodes+1 (the last column is a check; LFP reports from that electrode should always read 0 ) )


    file.create_dataset('sec_ids/'+str(gid),shape=len(sec_ids.values),data=sec_ids.values) # Creates dataset with section ids for the given neuron



def makeElectrodeDict(electrode_csv,type):

    electrode_df = pd.read_csv(electrode_csv,header=0,index_col=0)

    electrodes = {}

    for i in range(len(electrode_df.values)):

        name = electrode_df.index.values[i]
        position = np.array([electrode_df['x'].iloc[i],electrode_df['y'].iloc[i],electrode_df['z'].iloc[i]])

        if 'layer' in electrode_df.columns:
            layer = electrode_df['layer'].iloc[i]
        else:
            layer = "Not provided"

        if 'region' in electrode_df.columns:
            region = electrode_df['region'].iloc[i]
        else:
            region= "Not provided"


        electrodes[name] = {'position': position,'type': type,
        'region':region,'layer':layer}


    return electrodes

def getCellInfo(path_to_blueconfig):

    '''
    Returns the following:
    circuit: Path to the circuit used to generate the time steps. Gets written to the h5 file and is checked by neurodamus when and LFP simulation is run. LFP simulation will fail if it uses a different circuit than the one in the h5 file
    gids: list of gids for which segment coefficients will be written
    sectionIds: dataframe containing the gid and sectionId of each neuron
    '''

    sim = bp.Simulation(path_to_blueconfig)

    circ = sim.circuit


    rep = sim.report('Current')
    col = 'hex_O1'
    g = circ.cells.ids({'$target':col})

    data = rep.get(t_start=rep.t_start, t_end=rep.t_start + rep.t_step,gids=g)


    sectionIds = data.columns.to_frame() # Data frame containing gids and section ids
    sectionIds.index = range(len(sectionIds))


    return circ, g, sectionIds


def writeH5File(path_to_blueconfig,outputfile,electrode_csv,type):

    '''
    path_to_blueconfig refers to the BlueConfig from the 1-timestep simulation used to get the segment positions
    electrode_csv is a csv file containing the position, region, and layer of each electrode
    type is either EEG or LFP
    '''

    circ, gids, sectionIds = getCellInfo(path_to_blueconfig)

    electrodes = makeElectrodeDict(electrode_csv,type) # Dictionary containing metadata about the electrodes

    h5file = h5py.File(outputfile,'w') # Creates h5 file for coefficients

    ### This block sets memory parameters that make writing the H5 file faster
    h5id = h5file.id
    cc = h5id.get_mdc_config()
    cc.max_size = 1024*1024*124
    h5id.set_mdc_config(cc)
    #####


    h5 = ElectrodeFileStructure(h5file, gids, electrodes, circuit=circ.config["cells"]) # Initializes fields in h5 file

    secIds = pd.DataFrame(data=sectionIds.values[:,1],index=sectionIds.values[:,0]) # Data frame containing section ids, with gids as index


    for i, gid in enumerate(gids): # For each gid, initializes coefficient field in h5 file

        write_neuron(h5, h5file, gid, electrodes,secIds.loc[gid])

    h5file.close()

if __name__=='__main__':

    electrode_csv = sys.argv[1]

    type = sys.argv[2]

    path_to_simconfig = sys.argv[3]

    outputfile = sys.argv[4]


    writeH5File(path_to_simconfig,outputfile,electrode_csv,type)
