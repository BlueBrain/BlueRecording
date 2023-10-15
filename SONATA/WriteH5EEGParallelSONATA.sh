#!/bin/bash -l
#SBATCH --job-name="EEG_2_CoordsV"
#SBATCH --partition=prod
#SBATCH --nodes=160
#SBATCH -C clx
#SBATCH --cpus-per-task=2
#SBATCH --time=24:00:00
##SBATCH --mail-type=ALL
#SBATCH --account=proj45
#SBATCH --no-requeue
#SBATCH --output=EEG_2_CoordsV.out
#SBATCH --error=EEG_2_CoordsV.err
#SBATCH --exclusive
#SBATCH --mem=0

#source ~/bluepy-env/bin/activate
#module load archive/2022-01
#module load unstable
#module load py-mpi4py
#source ~/sirio/bin/activate

module purge

#spack load py-bluepy/jrzr2b
#module load unstable py-bluepy py-mpi4py

spack env activate h5mpi
#source ~/probevenv/bin/activate
module load unstable py-scipy

cp eeg_sonata_new/coeffsC6.h5 eeg_sonata_new/coeffs.h5

srun -n 4240 python writeH5_MPI_EEG_SONSATA_newspec.py 'C6' 'EEG' 'BlueConfig' 'positionsO1_new' 'eeg_sonata_new/coeffs.h5' 50
#10600
#360
#srun -n 6200 python writeH5_MPI_MEA.py 'Neuropixels-384' 'LFP' '../BlueConfig' '../positionsHex0' 'coeffsFull/coeffsNeuropixels-384Fixed.h5' 31 
