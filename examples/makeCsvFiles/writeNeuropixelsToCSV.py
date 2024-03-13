import MEAutility as MEA
import pandas as pd
import bluepysnap as bp
import sys
from bluerecording.utils import alignmentInfo, getAtlasInfo


def repositionElectrode(probe,center,azimuth,elevation):

    probe.rotate([0,1,0],elevation*180/np.pi)
    probe.rotate([0,0,1],azimuth*180/np.pi)
    probe.move(center)

    return(probe)


if __name__=='__main__':

    probe_name = sys.argv[1]
    path_to_simconfig = sys.argv[2]
    electrode_csv = sys.argv[3]

    probe = MEA.return_mea(probe_name)

    center, azimuth, elevation = alignentInfo(path_to_simconfig,'hex0')

    repositionElectrode(probe, center, azimuth, elevation)

    electrodePositions = probe.positions

    regionList, layerList = getAtlasInfo(path_to_simconfig, electrodePositions)

    electrodeData = pd.DataFrame(data=electrodePositions,columns=['x','y','z'])
    
    electrodeTypeList = []
    for p in electrodePositions:
        electrodeTypeList.append('LineSource')

    layerData = pd.DataFrame(data=layerList,columns=['layer'])

    regionData = pd.DataFrame(data=regionList,columns=['region'])
    
    electrodeTypeData = pd.DataFrame(data=electrodeTypeList,columns=['type'])

    data = pd.concat((electrodeData,layerData),axis=1)
    data = pd.concat((data,regionData),axis=1)
    data = pd.concat((data,electrodeTypeData),axis=1)

    data.to_csv(electrode_csv)
