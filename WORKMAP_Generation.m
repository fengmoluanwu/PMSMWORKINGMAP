function WORKMAP_Generation(~)
%This is the work map generation function.All points must work on the
%voltage curve.
global MotorParameters
global InverterParameters
global handle_figure
global CharacterristicPoints
global MapPoints
FirstSpeed = CharacterristicPoints.TransitionSpd;
LastSpeed = min(CharacterristicPoints.MaxSpd,InverterParameters.MaxSpeed);

FluxLinkage_Max = InverterParameters.Udc/(1.732*FirstSpeed/60*2*pi*MotorParameters.Pairs);
FluxLinkage_Min = InverterParameters.UdcMin/(1.732*LastSpeed/60*2*pi*MotorParameters.Pairs);
FluxLinkage_range = linspace(FluxLinkage_Max,FluxLinkage_Min,20);
SpeedRnge = double(int16(InverterParameters.Udc ./ FluxLinkage_range*60/(1.732*2*pi*MotorParameters.Pairs)));
CharacterristicPoints.SpdRange = SpeedRnge;
%Id = 0:-1:-InverterParameters.Imax;
Id_Vctr = 0:-1:-InverterParameters.Imax;
Iq_Vctr = 0:-1:-InverterParameters.Imax;
%Iq = 0:1:InverterParameters.Imax;
Vd_MAP = zeros(size(SpeedRnge,2),size(Iq_Vctr,2));
Vq_MAP = zeros(size(SpeedRnge,2),size(Iq_Vctr,2));
Te_MAP = zeros(size(SpeedRnge,2),size(Iq_Vctr,2));
MaxVleCnt_Map = zeros(size(SpeedRnge,2),3);
%RPM*2PI*Pairs/60 = we
n = 1;
figure(handle_figure);
hold on;
%linspace(FirstSpeed,LastSpeed,30)
for speed = SpeedRnge
    
    WE = speed*2*pi*MotorParameters.Pairs/60;
    i = 1;
    for Id = 0:-1:-InverterParameters.Imax
        Iq = 0;
        Vd = MotorParameters.Rs*Id - WE*MotorParameters.Lq*Iq;
        Vq = MotorParameters.Rs*Iq + WE*(MotorParameters.Ld*Id + MotorParameters.Phi);
        Vm = InverterParameters.Udc/1.732;
        Vpeak = sqrt(Vd^2 + Vq^2);
        Verr = Vpeak - Vm;
        while(Verr < -0.3)
            Iq = Iq + 0.01;
            if(sqrt(Iq^2+Id^2) >= InverterParameters.Imax)
%                 Iq = sqr(InverterParameters.Imax^2 - Id^2);
                break;
            end
            Vd = MotorParameters.Rs*Id - WE*MotorParameters.Lq*Iq;
            Vq = MotorParameters.Rs*Iq + WE*(MotorParameters.Ld*Id + MotorParameters.Phi);
            Vm = InverterParameters.Udc/1.732;
            Vpeak = sqrt(Vd^2 + Vq^2);
            Verr = Vpeak - Vm;
        end
        Id_Vctr(1,i) = Id;
        Iq_Vctr(1,i) = Iq;
        if(Id^2 + Iq^2 >= InverterParameters.Imax^2)
           Id_Vctr(1,i:end) = Id;
           Iq_Vctr(1,i:end) = Iq;
           break;
        end
        i = i + 1;
    
    end%current search
    Te_temp = 1.5*MotorParameters.Pairs*(MotorParameters.Phi*Iq_Vctr - (MotorParameters.Lq - MotorParameters.Ld)*Id_Vctr.*Iq_Vctr);
    [MaxVle,MaxVleCnt] = max(Te_temp);
    MaxVleCnt_Map(n,:) = [MaxVle,MaxVleCnt,speed];
    %fprintf('%f %d\n',MaxVle,MaxVleCnt);
    Vd_MAP(n,:) = Id_Vctr;
    Vd_MAP(n,MaxVleCnt:end) = Vd_MAP(n,MaxVleCnt);
    Vq_MAP(n,:) = Iq_Vctr;
    Vq_MAP(n,MaxVleCnt:end) = Vq_MAP(n,MaxVleCnt);
    
    Vd_Curve = Vd_MAP(n,1:MaxVleCnt);Vq_Curve = Vq_MAP(n,1:MaxVleCnt);
    if (abs(MotorParameters.Lq - MotorParameters.Ld) > 0.1e-6)
        for i = 1:MaxVleCnt

            if(Vd_Curve(1) <= -0.1)
               break; %that means MTPA does not intersect with voltage curve.

            end        
            Iq_MTPAPoint = interp1(CharacterristicPoints.MTPA_Id,CharacterristicPoints.MTPA_Iq,Vd_Curve(i),'linear','extrap');
            if(Vq_Curve(i) <= Iq_MTPAPoint)
                Iq_MTPA_interp = interp1(CharacterristicPoints.MTPA_Id,CharacterristicPoints.MTPA_Iq,Vd_Curve(1:i-1));
                Vq_MAP(n,1:i-1) = Iq_MTPA_interp;
                break;
            end
        end
    end
    Te_MAP(n,:) = 1.5*MotorParameters.Pairs*(MotorParameters.Phi*Vq_MAP(n,:) - (MotorParameters.Lq - MotorParameters.Ld)*Vd_MAP(n,:).*Vq_MAP(n,:));
    
    plot(Vd_MAP(n,:),Vq_MAP(n,:));
    text(Vd_MAP(n,end),Vq_MAP(n,end),num2str(speed));
    n = n + 1;
end%speed search 
MapPoints.D_MapPoints = Vd_MAP;
MapPoints.Q_MapPoints = Vq_MAP;
MapPoints.Te_MapPoints = Te_MAP;
MapPoints.MaxVlePoints = MaxVleCnt_Map;
end