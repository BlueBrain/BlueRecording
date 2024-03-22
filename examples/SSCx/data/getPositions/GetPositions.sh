#!/bin/bash -l
#SBATCH --job-name="EEG_1_CoordsV"
#SBATCH --partition=prod
#SBATCH --nodes=142
#SBATCH -C clx
#SBATCH --cpus-per-task=2
#SBATCH --time=24:00:00
##SBATCH --mail-type=ALL
#SBATCH --account=proj45
#SBATCH --no-requeue
#SBATCH --output=EEG_1_CoordsV.out
##SBATCH --error=EEG_1_CoordsV.err
#SBATCH --exclusive
#SBATCH --mem=0

spack env activate bluerecording-dev
source ~/bluerecording-env/bin/activate

CHUNK_SIZE=50

for i in {0..4235}
do

    folder="positions_all_new/$(($i/$CHUNK_SIZE))"
    mkdir -p $folder 2>/dev/null

done

srun -n 4235 python ../../../scripts/run_get_positions.py "../simulation_config.json" "positions_all_new" $CHUNK_SIZE

