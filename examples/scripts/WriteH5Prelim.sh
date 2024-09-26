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

ELECTRODE_FILE='eeg_csv.csv'
OUTPUT_FILE='coeffs_eeg.h5'

srun -n 1 python run_initialize_h5.py $ELECTRODE_FILE $PATH_TO_SIMCONFIG $OUTPUT_FILE 
