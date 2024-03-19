#!/bin/bash -l
#SBATCH --job-name="EEG_2_CoordsV"
#SBATCH --partition=prod
#SBATCH --nodes=14
#SBATCH -C clx
#SBATCH --cpus-per-task=2
#SBATCH --time=2:00:00
##SBATCH --mail-type=ALL
#SBATCH --account=proj45
#SBATCH --no-requeue
#SBATCH --output=EEG_0_CoordsV.out
#SBATCH --error=EEG_0_CoordsV.err
#SBATCH --exclusive
#SBATCH --mem=0

source ~/bluerecording-dev/bin/activate
spack env activate bluerecording-dev

mkdir pkls

srun -n 420 python geteeg.py

