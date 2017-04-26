int16 x
int16 y
bool ballChange  # True if these coordinates are for a different ball than the last message so can clear out memory for computing velocity
bool ballLost    # True if no target was found in the image