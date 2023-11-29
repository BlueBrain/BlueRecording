import pytest
import pandas as pd
import bluepy as bp
import numpy as np

from writeH5_prelim import makeElectrodeDict, count_segments

@pytest.fixture
def secCounts():

    '''
    Defines a data frame containing gids and section ids, equivalent to teh column indices from a voltage report
    '''

    sec_counts = pd.DataFrame(data=np.array([[1,1,1,1,1,1,2,2,2,2,2,2],[0,1,1,1,1,1,0,1,1,1,1,1]]).T,columns=('gid',bp.Section.ID))
    
    return sec_counts

def test_makeElectrodeDict():

    csv = '/gpfs/bbp.cscs.ch/project/proj83/tharayil/generationCode/create_lfp_weights_for_neurodamus/tests/data/electrode.csv'
    expected = {'name':{'position':np.array([1,2,3]),'type':'EEG','region':'Outside','layer':'Outside'}}

    print(makeElectrodeDict(csv,'EEG'))
    
    assert makeElectrodeDict(csv,'EEG') == expected

def test_count_segments(secCounts):
    
    gid = 1
    
    dataframe = secCounts.groupby("gid").apply(count_segments)
    
    count_dict = dataframe.loc[gid].values[0][0]
    count_dict = dict(zip(count_dict[0], count_dict[1]))
    
    assert count_dict == {0: 1, 1: 5}
