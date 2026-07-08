XNT=0.;
Y=0.;
VM=3000.;
HEDEG = -20.;
TF=10.;
XNP=4.;
YD=-VM*HEDEG/57.3;
T=0.;
H=0.01;
S=0;
n=0;
while T <= (TF-1e-5)
    YOLD=Y;
    YDOLD=YD;
    STEP=1;
    FLAG=0;
    
    while STEP <= 1
        if FLAG ==1
            STEP = 2;
            Y = Y + H*YD;
            YD = YD + H*YDD;
            T=T+H;
        end
        TGO = TF - T+0.00001;
        XLAMD = (Y+YD*TGO)/(VT*TGO*TGO);
        XNC = XNP*VC*XLAMD;
        YDD = XNT-XNC;
        FLAG = 1;
    end
    FLAG=0;
    Y = 0.5*(YOLD+ Y + H*YD);
    YD = 0.5*(YDOLD + YD + H*YDD);
    S = S+H;
    if S >= 0.09999
        S=0;
        n = n+1;
        ArrayT(n) = T;
        ArrayY(n) = Y;
        ArrayYD(n) = YD;
        ArrayXNCG(n) = XNC / 9.81;
    end
end
figure
plot(ArrayT,ArrayXNCG),grid
xlabel("Time (s)")
ylabel("Missile Acceleration (g)")
clc
output=[ArrayT,ArrayY,ArrayYD,ArrayXNCG];
save datfil.txt output
disp ("Simulation Finished")
