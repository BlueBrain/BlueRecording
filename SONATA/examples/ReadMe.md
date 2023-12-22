# Validation of EEG and LFP calculation

Here, we provide an example of an EEG and an LFP signal produced by a single layer 5 thick-tufted pyramidal cell.

## Instructions

### Dependencies
For each of the bash scripts described below, the dependencies are the same as described in the readme for the repository as a whole

### Calculating Segment Positions

As the same neuron is used in both examples, segment positions only need to be calculated once. First, in the **data/simulation** subfolder, the compartment report is produced for the single cell by running the script **launch.sh**. Next, in the the **data/getPositions** subfolder, the segment positions for this cell are extracted by running the script **GetPositions.sh**.

### Extracellular signal calculation

#### LFP

##### Electrode File

In the folder **LFP/electrodeFile**, the electrode csv file is created using the Jupyter notebook **createLfpCsc.ipynb**. The electrode weights h5 file then is created by running the scripts **WriteH5Prelim.sh** and **WriteH5.sh**. 

##### Online signal calculation

In the folder **LFP/simulation**, the simulation is launched by running the script **launch.sh**. 

##### Analysis
The figures in this subsection are generated notebook **LFP/simulation/MakeFigues.ipynb**

#### EEG

##### Electrode File

In the folder **LFP/electrodeFile**, the electrode csv file is already provided, having been created manually. The electrode weights h5 file then is created by running the scripts **WriteH5Prelim.sh** and **WriteH5.sh**. 

##### Online signal calculation

In the folder **EEG/simulation**, the simulation is launched by running the script **launch.sh**. 

##### Analysis
The figures in this subsection are generated notebook **EEG/simulation/MakeFigues.ipynb**

### Important note
The user should note that in the circuit and simulation configuration files, the absolute paths listed should be modified to match the user's system.
