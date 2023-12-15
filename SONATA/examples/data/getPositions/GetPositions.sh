#!/bin/bash -l
#SBATCH --job-name="EEG_1_CoordsV"
#SBATCH --partition=prod
#SBATCH --nodes=1
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

module purge

spack env activate getPositionsEnv

CHUNK_SIZE=50

for i in {0..0}
do

    folder="positions/$(($i/$CHUNK_SIZE))"
    mkdir -p $folder 2>/dev/null

done

srun -n 1 python getPositions.py "../simulation/simulation_config.json" "positions" $CHUNK_SIZE

