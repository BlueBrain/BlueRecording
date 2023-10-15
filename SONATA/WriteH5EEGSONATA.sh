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
#SBATCH --output=EEG_sonata_CoordsV.out
#SBATCH --error=EEG_sonata_CoordsV.err
#SBATCH --exclusive
#SBATCH --mem=0

module purge

mkdir eeg_sonata_new

source ~/probevenv/bin/activate

#module load unstable py-bluepy py-mpi4py

srun -n 1 python writeH5_MPI_prelim_EEG_SONATA_newspec.py 'C6' 'LFP' 'BlueConfig' 'positions0' 'eeg_sonata_new' 50 
