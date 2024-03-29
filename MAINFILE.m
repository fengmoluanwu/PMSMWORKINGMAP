%This is the main file,is used to input motor parameters and inverter
%parameters.

clear,clc;
global MotorParameters
global InverterParameters
global handle_figure
global CharacterristicPoints
global MapPoints
MotorParameters = struct('Rs',0.025,...
                         'Ld',70e-6,...
                         'Lq',80e-6,...
                         'Pairs',4,...
                         'Phi',0.008);
InverterParameters = struct('Imax',120,...
                            'Udc',13,...
                            'UdcMin',9.5,...
                            'UdcMax',16,...
                            'MaxSpeed',9000);
CharacterristicPoints = struct('MTPA_MAXId',0,...
                               'MTPA_MAXIq',0,...
                               'MTPA_MAXTe',0,...
                               'TransitionSpd',0,...
                               'MaxSpd',0,...
                               'CurrentStep',0,...
                               'MTPA_Id',0,...
                               'MTPA_Iq',0,...
                               'SpdRange',0);
MapPoints = struct('D_MapPoints',0,...
                   'Q_MapPoints',0,...
                   'Te_MapPoints',0,...
                   'MaxVlePoints',0);
handle_figure = figure(1);
MTPA_Caculation();                        

SpecialPoints_Caculation();

WORKMAP_Generation();

WorkMap_Save();