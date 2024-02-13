function SpecialPoints_Caculation(~)
global MotorParameters
global InverterParameters
global handle_figure
global CharacterristicPoints


Id = CharacterristicPoints.MTPA_MAXId;
Iq = CharacterristicPoints.MTPA_MAXIq;
Vm = InverterParameters.Udc/1.732;

syms WE
Vd = MotorParameters.Rs*Id - WE*MotorParameters.Lq*Iq;
Vq = MotorParameters.Rs*Iq + WE*(MotorParameters.Ld*Id + MotorParameters.Phi);
eqn = Vm-sqrt(Vd^2 + Vq^2);
we_Trans = max(double(solve(eqn)));

CharacterristicPoints.TransitionSpd = we_Trans/6.2832/MotorParameters.Pairs*60;

Id = -InverterParameters.Imax;
Vd = MotorParameters.Rs*Id;
Vq = WE*(MotorParameters.Ld*Id + MotorParameters.Phi);
eqn = Vm-sqrt(Vd^2 + Vq^2);
we_peak = max(double(solve(eqn)));

CharacterristicPoints.MaxSpd = we_peak/6.2832/MotorParameters.Pairs*60;




end