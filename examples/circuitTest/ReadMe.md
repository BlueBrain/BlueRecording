# Examples

Here, we provide examples of extracellular signals produced by a network of 100 cells

## System requirements

We assume that you are running these examples on a Linux system with slurm. If this is not the case, you will have to modify the provided bash scripts accordingly.

## Instructions

### Dependencies
For each of the bash scripts described below, the dependencies are the same as described in the readme for the repository as a whole

### Download data
Download the file networks.zip from 10.5281/zenodo.10927050 and unzip it into the folder *data/simulation/configuration/networks*

### Calculating Segment Positions

First, in the **data/simulation** subfolder, the compartment report is produced for the network by running the script **launch.sh**. Next, in the the **data/getPositions** subfolder, the segment positions for the cells are extracted by running the script **GetPositions.sh**.

### Extracellular signal calculation

##### Electrode File

In subfolder **test/electrodeFile**, the the electrode weights h5 file is created by running the scripts **WriteH5Prelim.sh** and **WriteH5.sh**. 

##### Online signal calculation
In subfolder **test/simulation**, the simulation is launched by running the script **launch.sh**. 

## Important note
The user should note that in the circuit and simulation configuration files, the absolute paths listed should be modified to match the paths in the user's system. In the bash scripts, the slurm commands must be modified according to the user's system.
