function [xi, yi, zi, face] = Devoir4(nout, nin, poso)
    % TODO
    N = 1000;
    M = 1000;

    ANG_POLAIRE_MIN = 0;
    ANG_POLAIRE_MAX = 90;
    ANG_AZIMUTAL_MIN = 0;
    ANG_AZIMUTAL_MAX = 90;


    anPol = genererVecteurLineaire(N, ANG_POLAIRE_MIN, ANG_POLAIRE_MAX);
    anAxi = genererVecteurLineaire(M, ANG_AZIMUTAL_MIN, ANG_AZIMUTAL_MAX);

    vecLum = calculerVecDirectionLum(anPol, anAxi);
end

function vec = genererVecteurLineaire(N, valeurMin, valeurMax)    
    % Calcul de l'incrément
    increment = (valeurMax - valeurMin) / (2 * N);
    
    % Génération du vecteur
    vec = valeurMin + increment * (1:2:N*2);
end

function vecLum = calculerVecDirectionLum(anPol, anAxi)
    % Calcul de la direction de la lumière
    vecLum = [sin(anPol) .* cos(anAxi); sin(anPol) .* sin(anAxi); cos(anPol)];
end