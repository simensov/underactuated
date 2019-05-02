import numpy as np 

###
### Drone specific
###
def printStates(x,y):
    ''' Both inputs are scalars
    '''
    return "q: [" + '{0:.2g}'.format(x) + "," + '{0:.2g}'.format(y) + "]"

def printInputs(u1,u2):
    ''' Both inputs are scalars
    '''
    return "u: [" + '{0:.3g}'.format(u1) + "," + '{0:.3g}'.format(u2) + "]"

def calculateFuel(u,dt):
    ''' u is a list of u1+u2 scalars, dt a scalar
    function implements naive integration to find area inder u vs time
    '''
    fuel = 0
    for i in range(len(u) - 1):
        f = ((max(u[i],u[i+1]) + 0.5 * np.fabs(u[i+1] - u[i])) * dt)**2
        fuel += f**2
    return fuel