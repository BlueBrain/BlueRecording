# Examples

The notebook ComputationalComparison.ipynb estimates the time that would be required by LFPy and Bionet to run a simulation the size of the SSCx run with BlueRecording, assuming an equal number of cores.

The folder *compare_to_reference_electrodes* is an example of BlueRecording''s ability to calculate extracellular signals from a variety of recording modalities. It contains all of the code necessary to produce Figure 2 from the paper "BlueRecording: A Pipeline for the efficient calculation of extracellular recordings in large-scale neural circuit models"

The folder *SSCx* is an example of BlueRecording''s ability to produce signals from a large (~4.2M neurons) neural circuits. It contains the code to reproduce Figures 4-8 in the aforementioned paper. The data requried to run the simulations in that folder must be obtained from the authors.

The folder *makeCsvFile* contains an example of how to use BlueRecording to produce an electrode csv file for a Neuropixels probe aligned to the central column in the BBP SSCx model. This is not referenced in the paper. This example also requires the data mentioned above.
