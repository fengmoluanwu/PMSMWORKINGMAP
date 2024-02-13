function MTPA_Caculation(~)
global MotorParameters
global InverterParameters
global handle_figure
global CharacterristicPoints

if(CharacterristicPoints.CurrentStep < 1)
    CurrentStep = 1;
else
    CurrentStep = 5;
end

Is = 0:CurrentStep:InverterParameters.Imax;

MTPA_Id = zeros(size(Is));MTPA_Iq = zeros(size(Is));MTPA_Te = zeros(size(Is));

if (abs(MotorParameters.Lq - MotorParameters.Ld) > 0.1e-6)
    %caculation by equation
    MTPA_Id = 1/(4*MotorParameters.Lq - 4*MotorParameters.Ld)*(MotorParameters.Phi - sqrt(MotorParameters.Phi^2 + 8 * Is.^2*(MotorParameters.Lq - MotorParameters.Ld)^2));
    MTPA_Iq = sqrt(Is.^2 - MTPA_Id.^2);
    MTPA_Te = 1.5*MotorParameters.Pairs*(MotorParameters.Phi * MTPA_Iq + (MotorParameters.Ld - MotorParameters.Lq) * MTPA_Id .* MTPA_Iq);
        
    [value,index] = max(MTPA_Te);
    CharacterristicPoints.MTPA_MAXId = MTPA_Id(index);
    CharacterristicPoints.MTPA_MAXIq = MTPA_Iq(index);
    CharacterristicPoints.MTPA_MAXTe = value;
    CharacterristicPoints.MTPA_Id = MTPA_Id;
    CharacterristicPoints.MTPA_Iq = MTPA_Iq;
    figure(handle_figure);
    plot(MTPA_Id,MTPA_Iq,'-*');
    grid on;
    hold on;
%     MTPA_Id_COE = polyfit(MTPA_Te,MTPA_Id,4);
%     MTPA_Iq_COE = polyfit(MTPA_Te,MTPA_Iq,4);
%     Te = 0:0.1:max(MTPA_Te);
%     MTPA_Id_Fit = polyval(MTPA_Id_COE,Te,4);
%     MTPA_Iq_Fit = polyval(MTPA_Iq_COE,Te,4);
%     plot(MTPA_Id_Fit,MTPA_Iq_Fit,'*');
else
    %caculation by search
        i = 1;
    for Is = 0:CurrentStep:InverterParameters.Imax
        theta = 0:0.1:90;
        MTPA_Id_Temp = -Is*cosd(theta);
        MTPA_Iq_Temp =  Is*sind(theta);
        MTPA_Te_Temp = 1.5*MotorParameters.Pairs*(MotorParameters.Phi*MTPA_Iq_Temp + (MotorParameters.Ld - MotorParameters.Lq)*MTPA_Id_Temp.*MTPA_Iq_Temp);
        [value,index] = max(MTPA_Te_Temp);
        MTPA_Id(1,i) = MTPA_Id_Temp(index);
        MTPA_Iq(1,i) = MTPA_Iq_Temp(index);
        MTPA_Te(1,i) = value;
        i = i+ 1;
    end
    [value,index] = max(MTPA_Te);
    CharacterristicPoints.MTPA_MAXId = MTPA_Id(index);
    CharacterristicPoints.MTPA_MAXIq = MTPA_Iq(index);
    CharacterristicPoints.MTPA_MAXTe = value;
    CharacterristicPoints.MTPA_Id = MTPA_Id;
    CharacterristicPoints.MTPA_Iq = MTPA_Iq;
    figure(handle_figure);
    plot(MTPA_Id,MTPA_Iq,'-*');
    grid on;
    hold on;
end
end