#!/bin/bash -l
#SBATCH --job-name="EEG_2_CoordsV"
#SBATCH --partition=prod
#SBATCH --nodes=80
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
# SPDX-License-Identifier: GPL-3.0-or-later


spack env activate bluerecording-dev
source ~/bluerecording-dev/bin/activate

PATH_TO_SIMCONFIG='../data/simulation/simulation_config.json'
PATH_TO_POSITIONS_FOLDER='../data/getPositions/positions_all_new'
OUTPUT_FILE='coeffs_eeg.h5'
NEURONS_PER_FILE=1000
FILES_PER_FOLDER=50
ELECTRODE_CSV='eeg_csv.csv'
TISSUE_CONDUCTANCE=0.374556
 
srun -n 2400 python ../../scripts/run_write_weights.py $PATH_TO_SIMCONFIG $PATH_TO_POSITIONS_FOLDER $OUTPUT_FILE $NEURONS_PER_FILE $FILES_PER_FOLDER $TISSUE_CONDUCTANCE 'EEG.h5' 'objective_csd_array_indices 51:102 102:153 153:179 179:205 205:218 218:231'
