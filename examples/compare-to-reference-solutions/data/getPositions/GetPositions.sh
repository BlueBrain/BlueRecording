#!/bin/bash -l
#SBATCH --job-name="EEG_1_CoordsV"
#SBATCH --partition=prod
#SBATCH --nodes=1
#SBATCH -C clx
#SBATCH --cpus-per-task=2
#SBATCH --time=24:00:00
##SBATCH --mail-type=ALL
#SBATCH --account=proj85
#SBATCH --no-requeue
#SBATCH --output=EEG_1_CoordsV.out
##SBATCH --error=EEG_1_CoordsV.err
#SBATCH --exclusive
#SBATCH --mem=0

spack env activate bluerecording-dev
source ~/bluerecording-env/bin/activate

NEURONS_PER_FILE=1000

FILES_PER_FOLDER=50

for i in {0..0}
do

    folder="positions/$(($i/$CHUNK_SIZE))"
    mkdir -p $folder 2>/dev/null

done

srun -n 1 python ../../../scripts/run_get_positions.py "../simulation/simulation_config.json" "positions" $NEURONS_PER_FILE $FILES_PER_FOLDER

