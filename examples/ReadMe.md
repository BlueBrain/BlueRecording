# Examples

Here, we provide examples of extracellular signals produced by a single layer 5 thick-tufted pyramidal cell. 

## Instructions

### Dependencies
For each of the bash scripts described below, the dependencies are the same as described in the readme for the repository as a whole

### Calculating Segment Positions

As the same neuron is used in both examples, segment positions only need to be calculated once. First, in the **data/simulation** subfolder, the compartment report is produced for the single cell by running the script **launch.sh**. Next, in the the **data/getPositions** subfolder, the segment positions for this cell are extracted by running the script **GetPositions.sh**.

### Extracellular signal calculation

In the folder **compare-to-reference-solutions** we provide two examples of online calculations of extracellular signals. In each of the examples, the neuron is placed within a large homogeneous medium; the only difference between the two examples is the location of the recording electrode (the reference electrode is in the same position). The signal is calculated using the generalized and dipole-based reciprocity approaches, and using the point-source and line-source approximations.

In the following sections, the instructions are the same for both of the example folders

#### Electrode File

In subfolder **compare-to-reference-solutions/*/electrodeFile**, where the * refers to each of the two example folders, the the electrode weights h5 file is created by running the scripts **WriteH5Prelim.sh** and **WriteH5.sh**. 

#### Online signal calculation
In subfolder **compare-to-reference-solutions/*/electrodeFile**, where the * refers to each of the two example folders, the simulation is launched by running the script **launch.sh**. 

#### Analysis
The figures in this subsection are generated notebook **compare-to-reference-solutions/MakeFigues.ipynb**


### Important note
The user should note that in the circuit and simulation configuration files, the absolute paths listed should be modified to match the user's system.
