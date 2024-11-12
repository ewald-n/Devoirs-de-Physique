%
% Devoir 3
% Lancer balle-boite
%
%
format long
clear;
CoteBloc=8/100;

% Valeurs à introduire selon le noméro de la simulation
%================================================================
% Tir 1
%vbloci = [ -2 -3  5 ];
%vballei = [ 5 2 0.642424 ];
%avbloci = [ 0 0 0 ];
%tl = 0.545454;

% Tir 2
%vbloci = [ -2 -3  5 ];
%vballei = [ 5 2 0.642424 ];
%avbloci = [ 0 0 15 ];
%tl = 0.545454;

% Tir 3
vbloci = [ 0 -6  3 ];
vballei = [ 7 0 0.40834 ];
avbloci = [ 0 0 0 ];
tl = 0.071429;

% Tir 4
%vbloci = [ 0 -6  3 ];
%vballei = [ 7 0 0.40834 ];
%avbloci = [ 0 0 15 ];
%tl = 0.071429;

% Tir 5
%vbloci = [ -2 -3  5 ];
%vballei = [ 5 2 0.642424 ];
%avbloci = [ -5 -5 0 ];
%tl = 0.6;

% Tir 6
%vbloci = [ -2 -3  5 ];
%vballei = [ 5 2 0.1 ];
%avbloci = [ 0 0 0 ];
%tl = 0.1;

%================================================================

[Resultat blocf ballef Post]=Devoir3(vbloci,avbloci,tl,vballei);
nel=length(Post);
tfin=Post(1,nel);
if Resultat == 0
  fprintf('La balle a touché le bloc\n \n');
elseif Resultat == 1
  fprintf('Le bloc a touché le sol en premier \n \n');
elseif Resultat==-1
  fprintf('La balle a touché le sol en premier \n \n');
else
  fprintf('erreur dans le résultat');
end;
posblocf=[Post(2,nel) Post(3,nel) Post(4,nel)];
posballef=[Post(5,nel) Post(6,nel) Post(7,nel)];

fprintf('Temps final: %8.5f s\n \n',tfin);
fprintf('Position finale de la balle: ( %7.4f , %7.4f , %7.4f )m \n', posballef(1), posballef(2), posballef(3));
fprintf('Position finale du bloc: ( %7.4f , %7.4f , %7.4f )m \n \n', posblocf(1), posblocf(2), posblocf(3));
fprintf('Vitesse de la balle juste avant la collision: ( %8.4f , %8.4f , %8.4f )m/s \n', ballef(1,1), ballef(2,1),ballef(3,1));
fprintf('Vitesse de la balle juste après la collision: ( %8.4f , %8.4f , %8.4f )m/s \n \n', ballef(1,2), ballef(2,2),ballef(3,2));
fprintf('Vitesse du bloc juste avant la collision: ( %8.4f , %8.4f , %8.4f )m/s \n', blocf(1,1), blocf(2,1),blocf(3,1));
fprintf('Vitesse du bloc juste après la collision: ( %8.4f , %8.4f , %8.4f )m/s \n \n', blocf(1,2), blocf(2,2),blocf(3,2));

fprintf('Vitesse angulaire du bloc juste avant la collision: ( %8.4f , %8.4f , %8.4f )rad/s \n', blocf(4,1), blocf(5,1),blocf(6,1));
fprintf('Vitesse angulaire du bloc juste après la collision: ( %8.4f , %8.4f , %8.4f )rad/s \n', blocf(4,2), blocf(5,2),blocf(6,2));


%
posbloci=[3 3 1];
posblocf=[Post(2,nel) Post(3,nel) Post(4,nel)];
hold
plot3(Post(2,:),Post(3,:),Post(4,:))
plot3(Post(5,:),Post(6,:),Post(7,:))
Bloc(CoteBloc,posblocf,avbloci,tfin)
Bloc(CoteBloc,posbloci,avbloci,0)
plot3(0,0,2,'o')


