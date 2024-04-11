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
#SBATCH --output=EEG_2_CoordsV.out
#SBATCH --error=EEG_2_CoordsV.err
#SBATCH --exclusive
#SBATCH --mem=0

spack env activate bluerecording-dev
source ~/bluerecording-dev/bin/activate

NEURONS_PER_FILE=1000
FILES_PER_FOLDER=50

srun -n 1 python ../../../scripts/run_write_weights.py '../../data/simulation/simulation_config.json' '../../data/getPositions/positions' 'coeffs.h5' $NEURONS_PER_FILE $FILES_PER_FOLDER 'electrodes.csv' '/gpfs/bbp.cscs.ch/project/proj83/tharayil/generationCode/create_lfp_weights_for_neurodamus/examples/compare-to-reference-solutions/close-electrodes/electrodeFile/Infinite_Close_HighRes_SmallSphere.h5 /gpfs/bbp.cscs.ch/project/proj83/tharayil/generationCode/create_lfp_weights_for_neurodamus/examples/compare-to-reference-solutions/close-electrodes/electrodeFile/Infinite_Close_HighRes_SmallSphere.h5'
