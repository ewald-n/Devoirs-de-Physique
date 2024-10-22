function [coup, vbf, ti, x, y, z] = Devoir2(option, rbi, vbi, wbi)
    % Constantes physiques
    g = 9.81; % gravité en m/s^2
    mb = 2.74e-3; % masse de la balle en kg
    Rb = 1.99e-2; % rayon de la balle en m

    % Paramètres du filet et de la table
    hz = 0.76; % hauteur de la table en m
    Lx = 2.74; % longueur de la table en m
    Ly = 1.525; % largeur de la table en m
    hf = 0.1525; % hauteur du filet en m

    % Initialisation des variables
    dt = 0.01; % pas de temps en secondes
    t = 0; % temps initial
    rb = rbi; % position initiale
    vb = vbi; % vitesse initiale
    omega_b = wbi; % vitesse angulaire constante
    x = []; y = []; z = []; ti = [];

    % Boucle de simulation
    while true
        % Enregistrer les positions pour la trajectoire
        x = [x; rb(1)];
        y = [y; rb(2)];
        z = [z; rb(3)];
        ti = [ti; t];

        % Calcul des forces selon l'option choisie
        Fg = [0, 0, -mb * g]; % force gravitationnelle
        Fv = [0, 0, 0]; % force de frottement
        Fm = [0, 0, 0]; % force de Magnus

        if option >= 2
            c = 0.1; % coefficient de frottement (arbitraire)
            Fv = -c * vb; % frottement visqueux
        end
        if option == 3
            lambda = 1e-4; % coefficient de Magnus (arbitraire)
            Fm = lambda * cross(omega_b, vb); % force de Magnus
        end

        % Accélération
        Ftot = Fg + Fv + Fm;
        ab = Ftot / mb;

        % Mise à jour de la vitesse et de la position
        vb = vb + ab * dt;
        rb = rb + vb * dt;
        t = t + dt;

        % Vérification des conditions d'arrêt
        if rb(3) <= 0 % touche le sol
            coup = 3; break;
        elseif rb(3) <= hz && rb(1) >= 0 && rb(1) <= Lx && rb(2) >= 0 && rb(2) <= Ly
            % touche la table
            if rb(1) < Lx / 2
                coup = 1; % côté du joueur
            else
                coup = 0; % coup réussi
            end
            break;
        elseif rb(1) == Lx / 2 && rb(3) <= hf % touche le filet
            coup = 2; break;
        end
    end

    % Résultats finaux
    vbf = vb;
end

