#!/bin/bash -l
#SBATCH --job-name="EEG_2_CoordsV"
#SBATCH --partition=prod
#SBATCH --nodes=10
#SBATCH -C clx
#SBATCH --cpus-per-task=2
#SBATCH --time=24:00:00
##SBATCH --mail-type=ALL
#SBATCH --account=proj45
#SBATCH --no-requeue
#SBATCH --output=EEG_S_CoordsV.out
#SBATCH --error=EEG_S_CoordsV.err
#SBATCH --exclusive
#SBATCH --mem=0


module purge

spack env activate writeCoefficientsEnv

ELECTRODE_TYPE='LFP' # Alternatively, EEG
PATH_TO_SIMCONFIG='arbitraryPath'
PATH_TO_POSITIONS_FOLDER='alsoArbitrary'
PATH_TO_ELECTRODE_CSV_FILE='electrodes.csv'
PATH_TO_ELECTRODE_COEFFICIENT_FILE='electrodeFile.h5'
FILES_PER_FOLDER=50 #Number of files in each subfolder in $PATH_TO_POSITIONS_FOLDER

srun -n 300 python writeH5.py $ELECTRODE_TYPE $PATH_TO_SIMCONFIG $PATH_TO_POSITIONS_FOLDER $PATH_TO_ELECTRODE_COEFFICIENT_FILE $FILES_PER_FOLDER

