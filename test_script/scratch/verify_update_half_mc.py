import numpy as np, time
lc=0.7; ap=0.7728; R1=2.164e-6; Q33=5.3e-6; h=0.034; R2=2.9e-4
I=np.eye(4)
F=np.array([[0,1,0,0],[0,0,1,0],[0,0,lc,0],[0,0,ap*(1-lc),1.0]])
g=np.array([0,0,-1.0,ap]); H1=np.array([1.0,0,0,0]); H2=h*np.array([0,0,0,1.0])
M=R1*(1-lc)*g; gn=(1-lc)*g; Q=Q33*np.outer(g,g)
def mc(seed,corrected,B=200,N=100000,burn=5000):
    rng=np.random.default_rng(seed)
    Fm=F-np.outer(gn,H1) if corrected else F
    Qm=Q-np.outer(M,M)/R1 if corrected else Q
    sig_t=np.sqrt(Q33-(1-lc)**2*R1)
    P=np.diag([1e-5]*3+[6e-6]); x=rng.multivariate_normal(np.zeros(4),P,size=B); xh=np.zeros((B,4))
    acc=0.0; cnt=0; Ee=np.zeros((4,4)); xh3_prev=np.zeros(B); res_prev=np.zeros(B); sumsq=0.0
    for k in range(N):
        if k>0:
            xh=xh@F.T+(np.outer(res_prev,gn) if corrected else 0.0)
            P=Fm@P@Fm.T+Qm
        nw=rng.normal(0,np.sqrt(R1),B); na=rng.normal(0,np.sqrt(R2),B); et=rng.normal(0,sig_t,B)
        y1=x[:,0]+nw; ey1=y1-xh[:,0]
        if k>=burn:
            prod=xh3_prev*ey1; acc+=prod.sum(); sumsq+=(prod**2).sum(); cnt+=B
        S1=P[0,0]+R1; l1=P[:,0]/S1; xh=xh+np.outer(ey1,l1); A1=I-np.outer(l1,H1); P=A1@P@A1.T+R1*np.outer(l1,l1)
        y2=h*x[:,3]+na; ey2=y2-xh@H2; S2=H2@P@H2+R2; l2=P@H2/S2; xh=xh+np.outer(ey2,l2); A2=I-np.outer(l2,H2); P=A2@P@A2.T+R2*np.outer(l2,l2)
        e=x-xh
        if k>=burn: Ee+=e.T@e
        xh3_prev=xh[:,2].copy(); res_prev=y1-xh[:,0]
        eps=(1-lc)*nw+et
        x=x@F.T+np.outer(eps,g)
    mean=acc/cnt; se=np.sqrt((sumsq/cnt-mean**2)/cnt)  # ignores serial corr (short, lc=0.7)
    return mean,se,Ee/cnt,P
for corrected in (False,True):
    t=time.time(); m,se,Pt,P=mc(11,corrected)
    print(f"corrected={corrected!s:5s} E[xh3[k-1] ey1[k]] = {m:+.3e} +- {se:.1e} (naive SE) | Ptrue33/P33-1={Pt[2,2]/P[2,2]-1:+.3e} "
          f"Ptrue31/P31-1={Pt[2,0]/P[2,0]-1:+.3e} Ptrue11/P11-1={Pt[0,0]/P[0,0]-1:+.3e} Ptrue44/P44-1={Pt[3,3]/P[3,3]-1:+.3e} [{time.time()-t:.0f}s]")
