%cas 1

AngRot = 0; % pas d'angle initial
omega = [0; 0; 0]; % pas de vitesse angulaire
forces = [11e6; 8.75e6; 8.75e6]; % tous les propulseurs sont allumés
posNL = [0; 0; 0]; % position initiale

%appels de la fonction
[pcmNL, INL, alphaNL] = Devoir1(AngRot, omega, forces, posNL);

disp('Position du centre de masse (pcmNL) :');
disp(pcmNL);

disp('Matrice d''inertie (INL) :');
disp(INL);

disp('Accélération angulaire (alphaNL) :');
disp(alphaNL);

%cas 2
AngRot = -pi/3; % Rotation de -π/3 radians autour de l'axe x
omega = [-0.54; 0; 0]; % Vecteur vitesse angulaire (rad/s)
forces = [11e6; 8.75e6; 0]; % Le propulseur droit est éteint
posNL = [0; -19.6075; 50]; % Position après l'accident

%appels de la fonction
[pcmNL, INL, alphaNL] = Devoir1(AngRot, omega, forces, posNL);

disp('Position du centre de masse (pcmNL) :');
disp(pcmNL);

disp('Matrice d''inertie (INL) :');
disp(INL);

disp('Accélération angulaire (alphaNL) :');
disp(alphaNL);




function [pcmNL, INL, alphaNL] = Devoir1(AngRot, omega, forces, posNL)
    % Entrées :
    %   AngRot - angle de rotation autour de l'axe x (en radians)
    %   omega  - vecteur vitesse angulaire (rad/s)
    %   forces - vecteur des forces [F_navette; F_propulseurG; F_propulseurD] (N)
    %   posNL  - position de la navette dans l'espace [x; y; z] (m)
    %
    % Sorties :
    %   pcmNL   - position du centre de masse du système navette-lanceur (m)
    %   INL     - tenseur d'inertie du système navette-lanceur (kg·m²)
    %   alphaNL - accélération angulaire du système navette-lanceur (rad/s²)

    %% Matrice de rotation autour de l'axe x
    Rx = [1, 0, 0;
          0, cos(AngRot), -sin(AngRot);
          0, sin(AngRot),  cos(AngRot)];

% ______________________________________________________________________________________________________________________
% NAVETTE
% ______________________________________________________________________________________________________________________
    hauteurNavetteCylindre = 27.93;
    rayonNavette = 3.5;
    hauteurNavetteCone = 9.31;
    masseTotaleNavette = 109000;

    volumeNavetteCylindre = pi * rayonNavette^2 * hauteurNavetteCylindre;
    volumeNavetteCone = (1/3) * pi * rayonNavette^2 * hauteurNavetteCone;
    volumeNavetteTotal = volumeNavetteCylindre + volumeNavetteCone;

    masseNavetteCylindre = masseTotaleNavette * (volumeNavetteCylindre / volumeNavetteTotal);
    masseNavetteCone = masseTotaleNavette - masseNavetteCylindre;

    % Centres de masse locaux avant rotation
    cmNavetteCylindreLocal = [0; 0; hauteurNavetteCylindre / 2];
    cmNavetteConeLocal = [0; 0; hauteurNavetteCylindre + hauteurNavetteCone / 4];

    % Centre de masse total de la navette

    cmNavette_loc = (masseNavetteCylindre*cmNavetteCylindreLocal + ...
                     masseNavetteCone*cmNavetteConeLocal)/masseTotaleNavette;

    % Moments d'inertie des composantes par rapport à leurs propres centres
    % de masse
    INavetteCylindre = calculerMomentInertieCylindrePlein(masseNavetteCylindre, rayonNavette, hauteurNavetteCylindre);
    INavetteCone = calculerMomentInertieConePlein(masseNavetteCone, rayonNavette, hauteurNavetteCone);

    % Calcul du moment d'inertie total de la navette après avoir ajusté les 
    % moments d'inertie des composantes par rapport au centre de masse de 
    % la navette
    INavette_loc = calculerMomentInertie(INavetteCylindre, masseNavetteCylindre, cmNavetteCylindreLocal, cmNavette_loc) + ...
                   calculerMomentInertie(INavetteCone, masseNavetteCone, cmNavetteConeLocal, cmNavette_loc);

% ______________________________________________________________________________________________________________________
% RESERVOIR
% ______________________________________________________________________________________________________________________
    hauteurReservoirCylindre = 39.1;
    rayonReservoir = 4.2;
    hauteurReservoirCone = 7.8;
    decalageYReservoir = rayonNavette + rayonReservoir;

    masseHydrogene = 108000;
    masseOxygene = 631000;
    masseTotaleReservoir = masseHydrogene + masseOxygene;

    hauteurCylindreOxygene = hauteurReservoirCylindre / 3;
    volumeCylindreOxygene = pi * rayonReservoir^2 * hauteurCylindreOxygene;
    volumeConeOxygene = (1/3) * pi * rayonReservoir^2 * hauteurReservoirCone;
    volumeTotalOxygene = volumeCylindreOxygene + volumeConeOxygene;

    masseCylindreOxygene = masseOxygene * (volumeCylindreOxygene / volumeTotalOxygene);
    masseConeOxygene = masseOxygene - masseCylindreOxygene;

    % Centres de masse des composantes du réservoir dans le
    % référentiel local 
    cmReservoirCylindreH2Local = [0; decalageYReservoir; (hauteurReservoirCylindre * 2 / 3) / 2];
    cmReservoirCylindreO2Local = [0; decalageYReservoir; hauteurReservoirCylindre * 2 / 3 + hauteurCylindreOxygene / 2];
    cmReservoirConeO2Local = [0; decalageYReservoir; hauteurReservoirCylindre + hauteurReservoirCone / 4];

    % Centre de masse total du réservoir dans le référentiel local 
    cmReservoir_loc = (masseHydrogene*cmReservoirCylindreH2Local + ...
                       masseCylindreOxygene*cmReservoirCylindreO2Local + ...
                       masseConeOxygene*cmReservoirConeO2Local)/masseTotaleReservoir;

    % Moments d'inertie
    IReservoirCylindreH2 = calculerMomentInertieCylindrePlein(masseHydrogene, rayonReservoir, hauteurReservoirCylindre * 2 / 3);
    IReservoirCylindreO2 = calculerMomentInertieCylindrePlein(masseCylindreOxygene, rayonReservoir, hauteurCylindreOxygene);
    IReservoirConeO2 = calculerMomentInertieConePlein(masseConeOxygene, rayonReservoir, hauteurReservoirCone);

    % Ajuster les moments d'inertie par rapport au centre de masse du reesrvoir
    IReservoir_loc = calculerMomentInertie(IReservoirCylindreH2, masseHydrogene, cmReservoirCylindreH2Local, cmReservoir_loc) + ...
                     calculerMomentInertie(IReservoirCylindreO2, masseCylindreOxygene, cmReservoirCylindreO2Local, cmReservoir_loc) + ...
                     calculerMomentInertie(IReservoirConeO2, masseConeOxygene, cmReservoirConeO2Local, cmReservoir_loc);

% ______________________________________________________________________________________________________________________
% PROPULSEURS D'APPOINT
% ______________________________________________________________________________________________________________________
    hauteurPropulseurCylindre = 39.9;
    rayonPropulseur = 1.855;
    hauteurPropulseurCone = 5.6;
    masseTotalePropulseur = 469000;
    decalageXPropulseur = rayonReservoir + rayonPropulseur;

    volumePropulseurCylindre = pi * rayonPropulseur^2 * hauteurPropulseurCylindre;
    volumePropulseurCone = (1/3) * pi * rayonPropulseur^2 * hauteurPropulseurCone;
    volumePropulseurTotal = volumePropulseurCylindre + volumePropulseurCone;

    massePropulseurCylindre = masseTotalePropulseur * (volumePropulseurCylindre / volumePropulseurTotal);
    massePropulseurCone = masseTotalePropulseur - massePropulseurCylindre;

    % Centres de masse des composantes des propulseurs dans le référentiel
    % local 
    % Propulseur gauche
    cmPropulseurGCylindreLocal = [-decalageXPropulseur; decalageYReservoir; hauteurPropulseurCylindre / 2];
    cmPropulseurGConeLocal = [-decalageXPropulseur; decalageYReservoir; hauteurPropulseurCylindre + hauteurPropulseurCone / 4];
    % Propulseur droit
    cmPropulseurDCylindreLocal = [decalageXPropulseur; decalageYReservoir; hauteurPropulseurCylindre / 2];
    cmPropulseurDConeLocal = [decalageXPropulseur; decalageYReservoir; hauteurPropulseurCylindre + hauteurPropulseurCone / 4];

    % Centres de masse totaux des propulseurs dans le référentiel local

    cmPropulseurG_loc = (massePropulseurCylindre * cmPropulseurGCylindreLocal + ...
                         massePropulseurCone * cmPropulseurGConeLocal) / masseTotalePropulseur;

    cmPropulseurD_loc = (massePropulseurCylindre * cmPropulseurDCylindreLocal + ...
                         massePropulseurCone * cmPropulseurDConeLocal) / masseTotalePropulseur;

    % Moments d'inertie
    IPropulseurCylindre = calculerMomentInertieCylindrePlein(massePropulseurCylindre, rayonPropulseur, hauteurPropulseurCylindre);
    IPropulseurCone = calculerMomentInertieConePlein(massePropulseurCone, rayonPropulseur, hauteurPropulseurCone);

    % Ajuster les moments d'inertie par rapport au centre de masse des propulseurs
    IPropulseurG_loc = calculerMomentInertie(IPropulseurCylindre, massePropulseurCylindre, cmPropulseurGCylindreLocal, cmPropulseurG_loc) + ...
                       calculerMomentInertie(IPropulseurCone, massePropulseurCone, cmPropulseurGConeLocal, cmPropulseurG_loc);
    IPropulseurD_loc = calculerMomentInertie(IPropulseurCylindre, massePropulseurCylindre, cmPropulseurDCylindreLocal, cmPropulseurD_loc) + ...
                       calculerMomentInertie(IPropulseurCone, massePropulseurCone, cmPropulseurDConeLocal, cmPropulseurD_loc);

% ______________________________________________________________________________________________________________________
% FUSEE
% ______________________________________________________________________________________________________________________
    %% MASSE TOTALE ET CENTRE DE MASSE GLOBAL
    masseTotale = masseTotaleNavette + masseTotaleReservoir + 2 * masseTotalePropulseur;
    
    %Position 
    pcmNL_loc = (masseTotaleNavette * cmNavette_loc + masseTotaleReservoir * cmReservoir_loc + ...
                 masseTotalePropulseur * cmPropulseurG_loc + masseTotalePropulseur * cmPropulseurD_loc) / masseTotale;

    %Passage au référentiel du laboratoire par application de la
    %translation et de la rotation
    pcmNL = posNL + Rx*pcmNL_loc;


    %% MOMENT D'INERTIE TOTAL
    % Ajuster les moments d'inertie par rapport au centre de masse global
    INavetteAdj = calculerMomentInertie(INavette_loc, masseTotaleNavette, cmNavette_loc, pcmNL_loc);
    IReservoirAdj = calculerMomentInertie(IReservoir_loc, masseTotaleReservoir, cmReservoir_loc, pcmNL_loc);
    IPropulseurGAdj = calculerMomentInertie(IPropulseurG_loc, masseTotalePropulseur, cmPropulseurG_loc, pcmNL_loc);
    IPropulseurDAdj = calculerMomentInertie(IPropulseurD_loc, masseTotalePropulseur, cmPropulseurD_loc, pcmNL_loc);

    INL_loc = INavetteAdj + IReservoirAdj + IPropulseurGAdj + IPropulseurDAdj;

    INL = Rx * INL_loc * Rx';

    %% CALCUL DES MOMENTS
    % Positions des forces
    posForceNavette = posNL; % Base de la navette
    posForcePropulseurG = posNL + Rx * [-decalageXPropulseur; decalageYReservoir; 0];
    posForcePropulseurD = posNL + Rx * [decalageXPropulseur; decalageYReservoir; 0];

    % Directions des forces (le long de l'axe z après rotation)
    directionForce = Rx * [0; 0; 1];

    % Vecteurs forces
    FNavette = forces(1) * directionForce;
    FPropulseurG = forces(2) * directionForce;
    FPropulseurD = forces(3) * directionForce;

    % Moments
    momentNavette = cross(posForceNavette - pcmNL, FNavette);
    momentPropulseurG = cross(posForcePropulseurG - pcmNL, FPropulseurG);
    momentPropulseurD = cross(posForcePropulseurD - pcmNL, FPropulseurD);

    momentTotal = momentNavette + momentPropulseurG + momentPropulseurD;

    %% acceleration angulaire
    alphaNL = INL \ (momentTotal - cross(omega, INL * omega));
end

%fonctions aides
%Fonction qui calcule le moment d'inertie d'un object composant dun objet composé avec la matrice de translation:
function inertieObjet = calculerMomentInertie(I_petit, massePetit, cmPetit, cmGrand)
    %param: (inertie objet composante, masse objet composant, centre masse objet composant, centre masse objet composé)
    d = cmPetit - cmGrand;  % Vecteur de translation
    T = [d(2)^2 + d(3)^2, -d(1)*d(2),    -d(1)*d(3);
        -d(1)*d(2),      d(1)^2 + d(3)^2, -d(2)*d(3);
        -d(1)*d(3),      -d(2)*d(3),      d(1)^2 + d(2)^2];

    inertieObjet = I_petit + (massePetit*T);
end

%Fonction qui calcule le moment d'inertie d'un cylindre plein:
function inertieCylindre = calculerMomentInertieCylindrePlein(masse, rayon, hauteur)
    % Calculer les moments d'inertie pour un cylindre
    Icxx = (1/12) * masse * (3 * rayon^2 + hauteur^2);
    Icyy = Icxx;
    Iczz = (1/2) * masse * rayon^2;

    % Créer la matrice d'inertie 3x3
    inertieCylindre = [Icxx, 0, 0;
                        0, Icyy, 0;
                        0, 0, Iczz];
end

%Fonction qui calcule le moment d'inertie d'un cone plein:
function inertieCone = calculerMomentInertieConePlein(masse, rayon, hauteur)
    % Calculer les moments d'inertie pour un cone
    Icxx = (1/80) * masse * (12 * rayon^2 + 3 * hauteur^2);
    Icyy = Icxx;
    Iczz = (3/10) * masse * rayon^2;
    % Créer la matrice d'inertie 3x3
    inertieCone = [Icxx, 0, 0;
                    0, Icyy, 0;
                    0, 0, Iczz];
end








