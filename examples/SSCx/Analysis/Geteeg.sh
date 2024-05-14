#!/bin/bash -l
#SBATCH --job-name="EEG_2_CoordsV"
#SBATCH --partition=prod
#SBATCH --nodes=14
#SBATCH -C clx
#SBATCH --cpus-per-task=2
#SBATCH --time=24:00:00
##SBATCH --mail-type=ALL
#SBATCH --account=proj85
#SBATCH --no-requeue
#SBATCH --output=EEG_0_CoordsV.out
#SBATCH --error=EEG_0_CoordsV.err
#SBATCH --exclusive
#SBATCH --mem=0

# SPDX-License-Identifier: GPL-3.0-or-later 

spack env activate bluerecording-dev

source ~/bluerecording-dev/bin/activate

mkdir ../sscxSimulation/pkls

srun -n 420 python geteeg.py

