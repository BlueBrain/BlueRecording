#!/bin/bash -l
#SBATCH --job-name="EEG_2_CoordsV"
#SBATCH --partition=prod
#SBATCH --nodes=1
#SBATCH -C clx
#SBATCH --cpus-per-task=2
#SBATCH --time=24:00:00
##SBATCH --mail-type=ALL
#SBATCH --account=proj85
#SBATCH --no-requeue
#SBATCH --output=EEG_sonata_CoordsV.out
#SBATCH --error=EEG_sonata_CoordsV.err
#SBATCH --exclusive
#SBATCH --mem=0

# SPDX-License-Identifier: GPL-3.0-or-later

spack env activate bluerecording-dev
source ~/bluerecording-dev/bin/activate

ELECTRODE_FILE='electrodes.csv'
PATH_TO_SIMCONFIG='../data/simulation/simulation_config.json'
OUTPUT_FILE='coeffs.h5'

srun -n 1 python ../../scripts/run_initialize_h5.py $ELECTRODE_FILE $PATH_TO_SIMCONFIG $OUTPUT_FILE 
