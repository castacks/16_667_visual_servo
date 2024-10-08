def pid_merge(px,ix,pv,iv):
    return px*pv+iv, pv*ix, pv

p,i,d=pid_merge(1.6,0.32,0.15,0.05)
print("P: %.4f, I: %.4f, D: %.4f"%(p,i,d))