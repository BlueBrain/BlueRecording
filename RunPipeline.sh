#!/bin/bash -l
#SBATCH --job-name="EEG_2_CoordsV"
#SBATCH --partition=prod
#SBATCH --nodes=160
#SBATCH -C clx
#SBATCH --cpus-per-task=2
#SBATCH --time=24:00:00
##SBATCH --mail-type=ALL
#SBATCH --account=proj45
#SBATCH --no-requeue
#SBATCH --output=EEG_2_CoordsV.out
#SBATCH --error=EEG_2_CoordsV.err
#SBATCH --exclusive
#SBATCH --mem=0


module purge


spack env activate writeCoefficientsEnv 

PATH_TO_SIMCONFIG='arbitraryPath'
PATH_TO_POSITIONS_FOLDER='alsoArbitrary'
PATH_TO_ELECTRODE_CSV_FILE='electrodes.csv'
PATH_TO_ELECTRODE_COEFFICIENT_FILE='electrodeFile.h5'
PATH_TO_POTENTIAL_FIELDS='potentialFieldsFromFEM' # Only applicable for Reciprocity, ommited for LineSource
FILES_PER_FOLDER=50 #Number of files in each subfolder in $PATH_TO_POSITIONS_FOLDER

NUMBER_OF_NEURONS=212000

NUMBER_OF_FILES=$(($NUMBER_OF_NEURONS/1000)) #Number of neurons in circuit divided by 1000

srun -n 1 python makeFolders.py $PATH_TO_SIMCONFIG $FILES_PER_FOLDER $PATH_TO_POSITIONS_FOLDER

srun -n $NUMBER_OF_FILES python getPositions.py $PATH_TO_SIMCONFIG $PATH_TO_POSITIONS_FOLDER $FILES_PER_FOLDER

srun -n 1 python writeH5_prelim.py $PATH_TO_ELECTRODE_CSV_FILE $PATH_TO_SIMCONFIG $PATH_TO_ELECTRODE_COEFFICIENT_FILE

srun -n 4240 python writeH5.py $PATH_TO_SIMCONFIG $PATH_TO_POSITIONS_FOLDER $PATH_TO_ELECTRODE_COEFFICIENT_FILE $FILES_PER_FOLDER $PATH_TO_ELECTRODE_CSV_FILE $PATH_TO_POTENTIAL_FIELDS