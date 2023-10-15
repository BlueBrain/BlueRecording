#!/bin/bash -l
#SBATCH --job-name="EEG_2_CoordsV"
#SBATCH --partition=prod
#SBATCH --nodes=100
#SBATCH -C clx
#SBATCH --cpus-per-task=2
#SBATCH --time=24:00:00
##SBATCH --mail-type=ALL
#SBATCH --account=proj45
#SBATCH --no-requeue
#SBATCH --output=EEG_SONATA_CoordsV.out
#SBATCH --error=EEG_SONATA_CoordsV.err
#SBATCH --exclusive
#SBATCH --mem=0

#source ~/bluepy-env/bin/activate
#module load archive/2022-01
#module load unstable
#module load py-mpi4py
#source ~/sirio/bin/activate

module purge

spack env activate h5mpi

cp neuropixels_sonata_new/coeffsNeuropixels-384.h5 neuropixels_sonata_new/coeffsneuropixels.h5

srun -n 3000 python writeH5_MPI_MEA_SONATA_newspec.py 'Neuropixels-384' 'LFP' 'BlueConfig' 'positionsO1_new' 'neuropixels_sonata_new/coeffsneuropixels.h5' 50

