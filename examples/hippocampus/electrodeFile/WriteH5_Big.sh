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
# SPDX-License-Identifier: GPL-3.0-or-later


spack env activate bluerecording-dev
source ~/bluerecording-dev/bin/activate

PATH_TO_SIMCONFIG='../data/simulation/simulation_config_big.json'
PATH_TO_POSITIONS_FOLDER='../data/getPositions/positions_all_new_big'
OUTPUT_FILE='coeffs_big.h5'
NEURONS_PER_FILE=1000
FILES_PER_FOLDER=50
TISSUE_CONDUCTANCE=0.374556
 
srun -n 12000 python ../../scripts/run_write_weights.py '../data/simulation/simulation_config_big.json' '../data/getPositions/positions_all_new_big' 'coeffs_big.h5' $NEURONS_PER_FILE $FILES_PER_FOLDER $TISSUE_CONDUCTANCE 
