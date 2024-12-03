function PlotBloc=Bloc(Cote,posblocf,avbloci,tfin)
CoteO2=Cote/2;
CoinsBlocNR=[-CoteO2 -CoteO2 -CoteO2
           -CoteO2  CoteO2 -CoteO2
            CoteO2  CoteO2 -CoteO2
            CoteO2 -CoteO2 -CoteO2
           -CoteO2 -CoteO2  CoteO2
           -CoteO2  CoteO2  CoteO2
            CoteO2  CoteO2  CoteO2
            CoteO2 -CoteO2  CoteO2];
for i=1:8
  CoinsBloc(i,:)=TourneCoin(avbloci*tfin,CoinsBlocNR(i,:))+posblocf;
end
%CoinsBloc=[-CoteO2+posblocf(1) -CoteO2+posblocf(2) -CoteO2+posblocf(3)
%           -CoteO2+posblocf(1)  CoteO2+posblocf(2) -CoteO2+posblocf(3)
%            CoteO2+posblocf(1)  CoteO2+posblocf(2) -CoteO2+posblocf(3)
%            CoteO2+posblocf(1) -CoteO2+posblocf(2) -CoteO2+posblocf(3)
%           -CoteO2+posblocf(1) -CoteO2+posblocf(2)  CoteO2+posblocf(3)
%           -CoteO2+posblocf(1)  CoteO2+posblocf(2)  CoteO2+posblocf(3)
%            CoteO2+posblocf(1)  CoteO2+posblocf(2)  CoteO2+posblocf(3)
%            CoteO2+posblocf(1) -CoteO2+posblocf(2)  CoteO2+posblocf(3)];
Face1x=[CoinsBloc(1,1) CoinsBloc(2,1) CoinsBloc(3,1) CoinsBloc(4,1)];
Face1y=[CoinsBloc(1,2) CoinsBloc(2,2) CoinsBloc(3,2) CoinsBloc(4,2)];
Face1z=[CoinsBloc(1,3) CoinsBloc(2,3) CoinsBloc(3,3) CoinsBloc(4,3)];
Face2x=[CoinsBloc(5,1) CoinsBloc(6,1) CoinsBloc(7,1) CoinsBloc(8,1)];
Face2y=[CoinsBloc(5,2) CoinsBloc(6,2) CoinsBloc(7,2) CoinsBloc(8,2)];
Face2z=[CoinsBloc(5,3) CoinsBloc(6,3) CoinsBloc(7,3) CoinsBloc(8,3)];
Face3x=[CoinsBloc(1,1) CoinsBloc(2,1) CoinsBloc(6,1) CoinsBloc(5,1)];
Face3y=[CoinsBloc(1,2) CoinsBloc(2,2) CoinsBloc(6,2) CoinsBloc(5,2)];
Face3z=[CoinsBloc(1,3) CoinsBloc(2,3) CoinsBloc(6,3) CoinsBloc(5,3)];
Face4x=[CoinsBloc(3,1) CoinsBloc(4,1) CoinsBloc(8,1) CoinsBloc(7,1)];
Face4y=[CoinsBloc(3,2) CoinsBloc(4,2) CoinsBloc(8,2) CoinsBloc(7,2)];
Face4z=[CoinsBloc(3,3) CoinsBloc(4,3) CoinsBloc(8,3) CoinsBloc(7,3)];
Face5x=[CoinsBloc(1,1) CoinsBloc(4,1) CoinsBloc(8,1) CoinsBloc(5,1)];
Face5y=[CoinsBloc(1,2) CoinsBloc(4,2) CoinsBloc(8,2) CoinsBloc(5,2)];
Face5z=[CoinsBloc(1,3) CoinsBloc(4,3) CoinsBloc(8,3) CoinsBloc(5,3)];
Face6x=[CoinsBloc(2,1) CoinsBloc(3,1) CoinsBloc(7,1) CoinsBloc(6,1)];
Face6y=[CoinsBloc(2,2) CoinsBloc(3,2) CoinsBloc(7,2) CoinsBloc(6,2)];
Face6z=[CoinsBloc(2,3) CoinsBloc(3,3) CoinsBloc(7,3) CoinsBloc(6,3)];
fill3(Face1x,Face1y,Face1z,[1 1 0])
fill3(Face2x,Face2y,Face2z,[1 0 1])
fill3(Face3x,Face3y,Face3z,[1 0 0])
fill3(Face4x,Face4y,Face4z,[0 0 1])
fill3(Face5x,Face5y,Face5z,[0 1 0])
fill3(Face6x,Face6y,Face6z,[1 1 1])
