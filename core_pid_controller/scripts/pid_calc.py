def get_PID(hover_thrust=0.55):
    wn = 1.3    # 0.6
    zeta = 1.0
    p = wn/2.0
    kp = wn**2+2.0*p*zeta*wn
    ki = p*wn**2
    kd = 2.0*zeta*wn+p
    
    C=hover_thrust/9.8
    kp *= C
    ki *= C
    kd *= C
    
    p1 = str(-p)
    
    if zeta>=1.0:
        p2 = "%.4f"%(-zeta*wn + wn*(zeta**2-1)**0.5)
        p3 = "%.4f"%(-zeta*wn - wn*(zeta**2-1)**0.5)

    else:
        p2r = -zeta*wn
        p2i = wn*(1-zeta**2)**0.5
        p2 = "%.4f"%(p2r)+"+"+"%.4f"%(p2i)+"j"
        p3 = "%.4f"%(p2r)+"-"+"%.4f"%(p2i)+"j"

    return (kp,ki,kd), (p1,p2,p3)

(p,i,d), (p1,p2,p3) = get_PID()
print("P: %.4f, I: %.4f, D: %.4f "%(p,i,d))
print("Poles: P1: %s, P2: %s, P3: %s"%(p1,p2,p3))