#!/bin/bash -l
#SBATCH --job-name="EEG_2_CoordsV"
#SBATCH --partition=prod
#SBATCH --nodes=1
#SBATCH -C clx
#SBATCH --cpus-per-task=2
#SBATCH --time=24:00:00
##SBATCH --mail-type=ALL
#SBATCH --account=proj45
#SBATCH --no-requeue
#SBATCH --output=EEG_3_CoordsV.out
#SBATCH --error=EEG_3_CoordsV.err
#SBATCH --exclusive
#SBATCH --mem=0

module purge

source ~/envForReqGeneration_NonSonata/bin/activate

ELECTRODE_TYPE='LFP' # Alternatively, EEG
PATH_TO_SIMCONFIG='arbitraryPath'
PATH_TO_POSITIONS_FOLDER='alsoArbitrary'
PATH_TO_ELECTRODE_CSV_FILE='electrodes.csv'
PATH_TO_ELECTRODE_COEFFICIENT_FILE='electrodeFile.h5'

srun -n 1 python writeH5_prelim.py $PATH_TO_ELECTRODE_CSV_FILE $ELECTRODE_TYPE $PATH_TO_SIMCONFIG $PATH_TO_POSITIONS_FOLDER $PATH_TO_ELECTRODE_COEFFICIENT_FILE
