#!/bin/bash -l
#SBATCH --job-name="EEG_2_CoordsV"
#SBATCH --partition=prod
#SBATCH --nodes=10
#SBATCH -C clx
#SBATCH --cpus-per-task=2
#SBATCH --time=24:00:00
##SBATCH --mail-type=ALL
#SBATCH --account=proj45
#SBATCH --no-requeue
#SBATCH --output=EEG_S_CoordsV.out
#SBATCH --error=EEG_S_CoordsV.err
#SBATCH --exclusive
#SBATCH --mem=0


module purge

spack env activate h5mpi

cp neuropixels_full/coeffsNeuropixels-384.h5 neuropixels_full/coeffsneuropixels.h5

srun -n 300 python writeH5_MPI_MEA_full.py 'Neuropixels-384' 'LFP' 'BlueConfig' 'positions0' 'neuropixels_full/coeffsneuropixels.h5' 50

