#!/bin/bash -l
#SBATCH --job-name="EEG_2_CoordsV"
#SBATCH --partition=prod
#SBATCH --nodes=400
#SBATCH -C clx
#SBATCH --cpus-per-task=2
#SBATCH --time=24:00:00
##SBATCH --mail-type=ALL
#SBATCH --account=proj85
#SBATCH --no-requeue
#SBATCH --output=EEG_2_CoordsV.out
#SBATCH --error=EEG_2_CoordsV.err
#SBATCH --exclusive
#SBATCH --mem=0


spack env activate bluerecording-dev
source ~/bluerecording-dev/bin/activate

PATH_TO_SIMCONFIG='../data/simulation/simulation_config.json'
PATH_TO_POSITIONS_FOLDER='../data/getPositions/positions_all_new'
OUTPUT_FILE='coeffsEcog_EEG.h5'
NEURONS_PER_FILE=1000
FILES_PER_FOLDER=50
ELECTRODE_CSV='ecog_eeg.csv'
TISSUE_CONDUCTANCE=0.374556
 
srun -n 12000 python ../../scripts/run_write_weights.py '../data/simulation/simulation_config.json' '../data/getPositions/positions_all_new' 'coeffsEcog_EEG.h5' $NEURONS_PER_FILE $FILES_PER_FOLDER 'ecog_eeg.csv' $TISSUE_CONDUCTANCE '/gpfs/bbp.cscs.ch/project/proj85/scratch/ECoG.h5 /gpfs/bbp.cscs.ch/project/proj85/scratch/EEG.h5 /gpfs/bbp.cscs.ch/project/proj85/scratch/LFP.h5 /gpfs/bbp.cscs.ch/project/proj85/scratch/ECoG.h5 /gpfs/bbp.cscs.ch/project/proj85/scratch/EEG.h5 /gpfs/bbp.cscs.ch/project/proj85/scratch/LFP.h5'
