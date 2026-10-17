# Defining controller function

def controller(t, positions, references):
    y1 = positions[t][0]
    r1 = references[t]
    e1 = y1 - r1


    if t == 0:
        e0 = 0
    else:
        y0 = positions[0][t-1]
        r0 = references[t-1]
        e0 = y0 - r0

    u = (0.15*e1) + 0.6 * (e1 - e0)
    return u
        
