# Validation of EEG and LFP calculation

Here, we provide an example of an EEG and an LFP signal produced by a single layer 5 thick-tufted pyramidal cell.

## Instructions

### Dependencies
For each of the bash scripts described below, the dependencies are the same as described in the readme for the repository as a whole

### Segment Positions

In the **data/simulation** subfolder, the compartment report is produced for the single cell by running the script **launch.sh**, and in the the **data/getPositions** subfolder, the segment positions for this cell are extracted by running the script **GetPositions.sh**.

### Extracellular signal calculation

In each example subfolder, the electrode file is created in the subfolder **electrodeFile** by running the scripts **WriteH5Prelim.sh** and **WriteH5.sh**. In the subfolder **simulation**, the simulation is launched by running the script **launch.sh**. The figures in this subsection are generated in the subfolder **analysis** by running the script **MakeFigures.sh**

### Important note
The user should note that in the circuit and simulation configuration files, the absolute paths listed should be modified to match the user's system.
