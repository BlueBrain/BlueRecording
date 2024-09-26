# Objective CSD calculation

BlueRecording permits the calculation of the "Objective CSD" metrics defined in the paper [iCSD produces spurious results in dense electrode arrays](). Two variants of the metric are defined in this paper: Objective Sphere CSD ($o_sCSD$) and Objective Disk CSD ($o_DCSD$). In addition, BlueRecording supports a third variant, "Objective Plane CSD", which is effectively $o_DCSD$ with a disk of infinite radius.

## Objective CSD electrodes in the input CSV file
In the CSV file used as an input to `bluerecording.writeH5_prelim.initializeH5File()`, electrodes using the objective CSD methods can be specified by setting the value of the `type` column to `ObjectiveCSD_Sphere`, `ObjectiveCSD_Disk`, or `ObjectiveCSD_Plane`. BlueRecording will use the following defaults:
- For the objective sphere method, the radius of the sphere will be set to 10 $\mu$m
- For the objective disk method, the radius of the disk will be set to 500 $\mu$m, and the thickness of the disk will be set equal to the distance between the electrodes (for each electrode, the disk will share a top and bottom surface with the disks for the electrodes immediately above and below). *Note that incorrect results may be produced if the electrodes are not equally spaced and in a single line*.
- For the objective plane method, the thickness of the plane will be set equal to the distance between the electrodes (for each electrode, the plane will share a top and bottom surface with the planes for the electrodes immediately above and below). *Note that incorrect results may be produced if the electrodes are not equally spaced and in a single line*.

You can also specify parametersby setting the value of the `type` column in the csv file as follows:
- `ObjectiveCSD_Sphere_X` will set the radius of the sphere to X $\mu$m
- `ObjectiveCSD_Disk_X` will set the radius of the disk to X $\mu$m; the thickness of the disk will be inferred from the electrode spacing, as before
- `ObjectiveCSD_Disk_X_Y` will set the radius of the disk to X $\mu$m and the thickness of the disk to Y $\mu$m
- `ObjectiveCSD_Plane_X` will set the thickness of the plane to X $\mu$m

## Calculating disk thickness from a subset of electrodes
It is possible that your electrodes csv file (and consequently your h5 weights file) will contain multiple electrode arrays from which you wish to calculate the objective disk or objective plane CSD. In this case, you can supply the argument `objective_csd_array_indices` to the function `bluerecording.writeH5.writeH5File()`. `objective_csd_array_indices` is a list of strings, with each string having the form `"a:b"`, where a and bv are the start and end indices in the csv file of one of the electrode arrays. If the thickness of the disks/planes is not specified as described in the section above, BlueRecording will estimate the thickness for each array separately based on the `objective_csd_array_indices` argmuent
