# code snippets that I made, but has no use in differential_flatness.py yet, or possibly ever!

# attempt to use time as decision variables. didnt work since the rest of quadrotor2d was not compatible with it. see piazzapost from russ

if False:
 # Add time as cont var. Have to add to self.coeffs also?  
    k = 0; t = zpp.prog.NewContinuousVariables(1, "t_%d" % k)
    t_over_time = t

    # add time variables from t1 to tf-1. t0 shall be zero and t_f should be constrained
    for k in range(1,tf):
        t = zpp.prog.NewContinuousVariables(1, "t_%d" % k)
        t_over_time = np.vstack((t_over_time, t))

    ### Add constraint on t_0 = 0. Or set t=0 in add_constraint?
    ### Add constraint on t_i-1 <= t_i. "<=" allows the path to be done early
    ### Constrain final time instance to be == tf
    # zpp.prog.AddLinearConstraint(t_over_time[-1][0] == tf)
    # --- OR ---
    # Constrain final time instance to be <= tf, and add cost on it
    zpp.prog.AddLinearConstraint(t_over_time[0][0] == 0)
    
    timeslack = 0.01
    for i in range(0,tf-1):
        zpp.prog.AddLinearConstraint(t_over_time[i][0] <=\
                                     t_over_time[i+1][0] + timeslack)

    zpp.prog.AddLinearConstraint(t_over_time[-1][0] <= tf)
    zpp.prog.AddQuadraticCost( (t_over_time[-1,0])**2 )



# inefficient plotting technique
if False:
    stime = time.time()
    xplot = np.array(xplot)
    yplot = np.array(yplot)
    tck,u = interpolate.splprep( [xplot,yplot] ,s = 0)
    xnew,ynew = interpolate.splev( np.linspace( 0, 1, 200), tck,der = 0)
    ax.plot( xplot,yplot, xnew ,ynew )
    print time.time() - stime