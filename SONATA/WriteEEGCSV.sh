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


module purge

source  ~/envForReqGenerationCsv/bin/activate

srun -n 1 python writeEEGToCSV.py 'C6' '/gpfs/bbp.cscs.ch/project/proj68/scratch/tharayil/sonata_circuits/newVPM/testing/full/testVPM/newConfig/174b9760-77b7-47de-8008-ce817f046920/0/simulation_config.json' '/gpfs/bbp.cscs.ch/project/proj83/data/atlas/S1/MEAN/P14-MEAN' 'electrode_csv.csv'
