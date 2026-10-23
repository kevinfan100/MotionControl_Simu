import numpy as np
np.set_printoptions(precision=6, linewidth=140)

lc=0.7; ap=0.7728; R1=2.164e-6; Q33=5.3e-6; h=0.034; R2=2.9e-4
I=np.eye(4)
F=np.array([[0,1,0,0],[0,0,1,0],[0,0,lc,0],[0,0,ap*(1-lc),1.0]])
g=np.array([0,0,-1.0,ap])
Q=Q33*np.outer(g,g)
H1=np.array([1.0,0,0,0]); H2=h*np.array([0,0,0,1.0])
M=R1*(1-lc)*g
gn=(1-lc)*g
Ft=F-np.outer(gn,H1)
Qt=Q-np.outer(M,M)/R1
zvec=np.array([0,0,ap,1.0])   # z = e4 + ap e3

def riccati_step(P,Fm,Qm):
    Pm=Fm@P@Fm.T+Qm
    S1=Pm[0,0]+R1; l1=Pm[:,0]/S1
    A1=I-np.outer(l1,H1)
    P1=A1@Pm@A1.T+R1*np.outer(l1,l1)
    S2=H2@P1@H2+R2; l2=P1@H2/S2
    A2=I-np.outer(l2,H2)
    P=A2@P1@A2.T+R2*np.outer(l2,l2)
    return P,Pm,P1,l1,l2,S1,S2

def run(Fm,Qm,N,P0):
    P=P0.copy(); out={}
    for k in range(1,N+1):
        P,Pm,P1,l1,l2,S1,S2=riccati_step(P,Fm,Qm)
        if k in (1600,10000,100000,N):
            out[k]=dict(l31=l1[2],l41=l1[3],lz=l1[3]+ap*l1[2],l42=l2[3],l32=l2[2],l12=l2[0],l11=l1[0],l21=l1[1],
                        P33=P[2,2],P44=P[3,3],P31=P[2,0],P34=P[2,3],P11=P[0,0],
                        Pz=zvec@P@zvec, Pm44=Pm[3,3], P1_14=P1[0,3], S1=S1,S2=S2)
    return P,out

P0=np.diag([1e-5,1e-5,1e-5,6e-6])
N=100000
print("=== Task 1: M-ignorant Riccati (F,Q) ===")
P_ign,out=run(F,Q,N,P0)
for k,v in out.items():
    print(f"k={k:7d} l31={v['l31']:.5f} l41={v['l41']:.5f} l41+ap*l31={v['lz']:+.3e} l42={v['l42']:.4e} "
          f"P33={v['P33']:.4e} P44={v['P44']:.4e} P31={v['P31']:.4e} Pz={v['Pz']:.3e} ap^2P33={ap**2*v['P33']:.4e}")
v=out[N]; print(f"  l11={v['l11']:.4f} l21={v['l21']:.4f} l32={v['l32']:.3e} l12={v['l12']:.3e} S1={v['S1']:.4e} P11={v['P11']:.4e} P34={v['P34']:.4e}")
print(f"  logged: l31=0.6576 l41=-0.5082 l42=1.5e-3 ; -ap*0.6576={-ap*0.6576:.4f}")
# y2 sourcing of Cov(z,e1): -(h^2 P1_14/S2) Cov(z,e4)
print(f"  y2-sourcing coefficient h^2*P1_14/S2 = {h**2*v['P1_14']/v['S2']:.3e}")

print("\n=== Task 3: corrected Riccati (Ft,Qt) ===")
P_cor,outc=run(Ft,Qt,N,P0)
for k,vc in outc.items():
    print(f"k={k:7d} l31={vc['l31']:.5f} l41={vc['l41']:.5f} l41+ap*l31={vc['lz']:+.3e} l42={vc['l42']:.4e} "
          f"P33={vc['P33']:.4e} P44={vc['P44']:.4e} P31={vc['P31']:.4e} Pz={vc['Pz']:.3e}")
vc=outc[N]
print("  ratios corrected/ignorant at k=N: l31 %.4f l41 %.4f l42 %.4f P33 %.4f P44 %.4f P31 %.4f P11 %.4f" %
      (vc['l31']/v['l31'],vc['l41']/v['l41'],vc['l42']/v['l42'],vc['P33']/v['P33'],vc['P44']/v['P44'],vc['P31']/v['P31'],vc['P11']/v['P11']))
vc16=outc[1600]; v16=out[1600]
print("  ratios at k=1600: P33 %.4f P44 %.4f l42 %.4f" % (vc16['P33']/v16['P33'],vc16['P44']/v16['P44'],vc16['l42']/v16['l42']))

print("\n=== Task 2: cross-moment recursion, M-ignorant gains, truth M!=0 ===")
def coupled(N,P0,text_version=False):
    P=P0.copy(); Pt=P0.copy(); C=np.zeros((4,4))
    # gains at step k are computed from filter P^- at k; E[xhat[k] w[k]^T] uses those
    l1_prev=None; l2_prev=None
    for k in range(1,N+1):
        Pm=F@P@F.T+Q
        S1=Pm[0,0]+R1; l1=Pm[:,0]/S1
        A1=I-np.outer(l1,H1)
        P1=A1@Pm@A1.T+R1*np.outer(l1,l1)
        S2=H2@P1@H2+R2; l2=P1@H2/S2
        A2=I-np.outer(l2,H2)
        Pnew=A2@P1@A2.T+R2*np.outer(l2,l2)
        # truth predict using previous-step gains (E[xhat[k-1] w[k-1]^T] = (I-l2H2) l1 M^T)
        if l1_prev is None:
            XW=np.zeros((4,4))
        else:
            lw = l1_prev if text_version else (I-np.outer(l2_prev,H2))@l1_prev
            XW=np.outer(lw,M)
        Ptm=F@Pt@F.T+Q-F@XW-XW.T@F.T
        Cm=F@C@F.T+F@XW
        C1=Cm@A1.T+np.outer(l1,H1@Ptm@A1.T)-R1*np.outer(l1,l1)
        Pt1=A1@Ptm@A1.T+R1*np.outer(l1,l1)
        C=C1@A2.T+np.outer(l2,H2@Pt1@A2.T)-R2*np.outer(l2,l2)
        Pt=A2@Pt1@A2.T+R2*np.outer(l2,l2)
        P=Pnew; l1_prev=l1; l2_prev=l2
        if k in (1600,10000,N):
            print(f"  k={k:7d} C32={C[2,1]:+.4e} C33={C[2,2]:+.4e} C31={C[2,0]:+.4e} C43={C[3,2]:+.4e} C44={C[3,3]:+.4e} | "
                  f"Pt33/P33-1={Pt[2,2]/P[2,2]-1:+.4e} Pt31/P31-1={Pt[2,0]/P[2,0]-1:+.4e} Pt11/P11-1={Pt[0,0]/P[0,0]-1:+.4e} Pt44/P44-1={Pt[3,3]/P[3,3]-1:+.3e}")
    return C,Pt,P
print(" corrected E[xhat w^T]=(I-l2H2) l1 M^T:")
C,Pt,P=coupled(N,P0)
print(" text version E[xhat w^T]=l1 M^T:")
Ct,_,_=coupled(N,P0,text_version=True)
print(f"  C32 difference text-corrected = {Ct[2,1]-C[2,1]:+.3e}")
print(f"  measured C32 = +1.5e-7 (SEM 0.4e-7)")
print("  full C (posterior, k=N):"); print(C)

np.save('/Users/kevin/.claude/jobs/dc9ca2b4/tmp/P_ign.npy',P_ign); np.save('/Users/kevin/.claude/jobs/dc9ca2b4/tmp/P_cor.npy',P_cor)
