function WorkMap_Save(~)
%This file is used for processing map data and save them.
%   此处显示详细说明
global MotorParameters
global InverterParameters
global handle_figure
global CharacterristicPoints
global MapPoints
LpCntr = size(CharacterristicPoints.SpdRange,2);
WorkMap_Id_temp = cell(LpCntr,1);
WorkMap_Iq_temp = cell(LpCntr,1);
WorkMap_Te_temp = cell(LpCntr,1);
WorkMap_Id = cell(LpCntr,1);
WorkMap_Iq = cell(LpCntr,1);
WorkMap_Te = cell(LpCntr,1);
for i = 1:LpCntr
    WorkMap_Id_temp{i,1} = MapPoints.D_MapPoints(i,1:MapPoints.MaxVlePoints(i,2));
    WorkMap_Iq_temp{i,1} = MapPoints.Q_MapPoints(i,1:MapPoints.MaxVlePoints(i,2));
    WorkMap_Te_temp{i,1} = MapPoints.Te_MapPoints(i,1:MapPoints.MaxVlePoints(i,2));

   FirstNonZeroIndex = 2;
   while(FirstNonZeroIndex<=size(WorkMap_Te_temp{i,1},2))
       if(WorkMap_Iq_temp{i,1}(1,2) > 0.05)
           WorkMap_Id{i,1} = WorkMap_Id_temp{i,1};
           WorkMap_Iq{i,1} = WorkMap_Iq_temp{i,1};
           WorkMap_Te{i,1} = WorkMap_Te_temp{i,1};
           break;
       end              
       if(WorkMap_Iq_temp{i,1}(1,FirstNonZeroIndex) > 0.001 && FirstNonZeroIndex > 2)
           WorkMap_Id{i,1} = WorkMap_Id_temp{i,1}(1,FirstNonZeroIndex-1:end);
           WorkMap_Iq{i,1} = WorkMap_Iq_temp{i,1}(1,FirstNonZeroIndex-1:end);
           WorkMap_Te{i,1} = WorkMap_Te_temp{i,1}(1,FirstNonZeroIndex-1:end);
           break;
       end
       FirstNonZeroIndex = FirstNonZeroIndex + 1;     
   end
end
MaxTeVle = max(MapPoints.MaxVlePoints(1,1));

LutVctr_SpdRPM = CharacterristicPoints.SpdRange;
LutVctr_TeNm = linspace(0,MaxTeVle,20);
SpdIntrpPoints = LpCntr;TeIntrpPoints = size(LutVctr_TeNm,2);
LutMAP_Id = zeros(SpdIntrpPoints,TeIntrpPoints);
LutMAP_Iq = zeros(SpdIntrpPoints,TeIntrpPoints);
for i = 1:LpCntr
    j = 1;
    while(j<=TeIntrpPoints)
        if(LutVctr_TeNm(j) <= max(WorkMap_Te{i,1}))
             LutMAP_Id(i,j) = interp1(WorkMap_Te{i,1},WorkMap_Id{i,1},LutVctr_TeNm(j));
             LutMAP_Iq(i,j) = interp1(WorkMap_Te{i,1},WorkMap_Iq{i,1},LutVctr_TeNm(j));
        else
             LutMAP_Id(i,j:end) = LutMAP_Id(i,j-1);
             LutMAP_Iq(i,j:end) = LutMAP_Iq(i,j-1);
            break;
        end
        j = j +1;
    end
end
LutVctr_TeMaxNm = MapPoints.MaxVlePoints(:,1);
figure(2);
plot(MapPoints.MaxVlePoints(:,3),MapPoints.MaxVlePoints(:,1));
save workpoints.mat LutVctr_SpdRPM LutVctr_TeNm LutMAP_Id LutMAP_Iq LutVctr_TeMaxNm

end

