function CoinBlocTourne=TourneCoin(RotationBlocf,CoinBloc)
%
%  Tourner le coin du bloc 
%  Omega  : Quaternion de rotation.
%
%  Deplacer les faces et les normales autour du centre de masse
%
angle=norm(RotationBlocf);
if angle == 0
  CoinBlocTourne=CoinBloc;
else
  direction=RotationBlocf/angle;
  qrot=[cos(angle/2) sin(angle/2)*direction];
  CoinBlocTourne=QVRotation(qrot,CoinBloc);
end
