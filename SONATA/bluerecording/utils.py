import bluepysnap as bp
import json

def getReport(path_to_simconfig):
    
    rSim = bp.Simulation(path_to_simconfig)
    r = rSim.reports[list(rSim.reports.keys())[0]] # We assume that the compartment report is the only report produced by the simulation

    circuit = rSim.circuit

    population_name = r.population_names[0]

    report = r[population_name]
    
    return report, population_name

def getSimulationInfo(path_to_simconfig, newidx=None):

    '''
    Returns the following:
    circuit: Path to the circuit used to generate the time steps. Gets written to the h5 file and is checked by neurodamus when and LFP simulation is run. LFP simulation will fail if it uses a different circuit than the one in the h5 file
    population_name: SONATA population name
    node_ids: list of ids for which segment coefficients will be written
    data: dataframe with a compartment report, whose columns are the node_id and sectionId of each neuron
    '''
    
    with open(path_to_simconfig) as f:

        circuitpath = json.load(f)['network']

    report, population_name = getReport(path_to_simconfig)
    
    nodeIds = report.node_ids
    
    if newidx is not None:

        try:
            ids = nodeIds[1000*newidx:1000*(newidx+1)]
        except:
            ids = nodeIds[1000*newidx:]
            
    else:
        
        ids = nodeIds


    data = report.get(group=ids,t_start=0,t_stop=r.dt)
    
    data.columns = data.columns.rename('id',level=0)
    data.columns = data.columns.rename('section',level=1)

    population = rSim.circuit.nodes[population_name]

    return report, circuitpath, population_name, nodeIds, data

