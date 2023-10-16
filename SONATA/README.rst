======
Instructions for creating coefficient files
======

As before, run getPositions.py, then writeH5_prelim.py, then writeH5.py

Both writeH5_prelim.py and writeH5.py take a csv file containing, at minimum, the electrode positions, as well as, optionally, an electrode name, and the layer of each electrode.

The csv file has a header row with column names 'x','y','z', and optionally 'electrode' and 'layer', and an index column with indices from 0 to n, where n is the number of electrode contacts.
