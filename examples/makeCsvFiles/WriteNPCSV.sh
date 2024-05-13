#!/bin/bash -l
#SBATCH --job-name="EEG_2_CoordsV"
#SBATCH --partition=prod
#SBATCH --nodes=1
#SBATCH -C clx
#SBATCH --cpus-per-task=2
#SBATCH --time=24:00:00
##SBATCH --mail-type=ALL
#SBATCH --account=proj83
#SBATCH --no-requeue
#SBATCH --output=EEG_2_CoordsV.out
#SBATCH --error=EEG_2_CoordsV.err
#SBATCH --exclusive
#SBATCH --mem=0

# SPDX-License-Identifier: GPL-3.0-or-later

spack env activate bluerecording-dev
source ~/bluerecording-dev/bin/activate

srun -n 1 python writeNeuropixelsToCSV.py 'Neuropixels-384' '../circuitTest/data/simulation/simulation_config.json' 'electrode_csv.csv'
