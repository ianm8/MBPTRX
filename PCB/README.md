# BOM Notes

## Resistors
All SM resistors are 1206 thick film of at least 125mW except R77 which should be at least 500mW.

## Capacitors
All SM capacitors are 1206 NP0 or C0G except for the 22uF and 100uF (X5R) capacitors.

## Inductors
All SM inductors are 1206 for as high SRF as possible (ie, greater than 30MHz), except for L23 which has a specific part number.

# Build notes
The MMG3H21NT1 is now obsolete but you can use a GVA-84+ from Mini Circuits.

# PA Bias Adjustment
1. Make sure you have a dummy load connected
2. Set the bias trimpots to mid position
3. Apply power using a power supply set to 13.8V and current max 1.5A
4. Make sure the mode is set for SSB
5. Press PTT and adjust the trimpots for less than 1V on the gate of the MOSFETs
6. Note the current
7. Adjust one trimpot for an additional 250ma of current draw
8. Note the current
9. Adjust the other trimpot for another 250ma of current draw
10. No need to be exact
