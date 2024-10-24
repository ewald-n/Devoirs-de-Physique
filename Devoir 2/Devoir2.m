disp("Simulation...");
function [coup, vbf, ti, x, y, z] = Devoir2(option, r_init, v_init, w_init)
    % Paramètres physiques
    g = 9.8;                      % Accélération gravitationnelle (m/s^2)
    m_balle = 2.74e-3;            % Masse de la balle (kg)
    r_balle = 1.99e-2;            % Rayon de la balle (m)
    rho_air = 1.2;                % Densité de l'air (kg/m^3)
    Cd = 0.5;                     % Coefficient de traînée
    C_M = 0.29;                   % Coefficient de Magnus

    % Surfaces et constantes
    A = pi * r_balle^2;           % Aire efficace de la balle
    k_visc = 0.5 * rho_air * Cd * A; % Coefficient de frottement visqueux
    S_magnus = 4 * pi * C_M * rho_air * r_balle^3; % Constante pour la force de Magnus

    % Initialisation des variables
    dt = 1e-4;                    % Pas de temps (s)
    t_max = 10;                   % Durée maximale de simulation (s)
    t = 0:dt:t_max;               % Vecteur temps
    n_steps = length(t);          % Nombre de pas de temps

    % Vecteurs pour stocker les positions et vitesses
    r = zeros(n_steps, 3);        % Position (x, y, z)
    v = zeros(n_steps, 3);        % Vitesse (vx, vy, vz)
    r(1,:) = r_init;              % Position initiale
    v(1,:) = v_init;              % Vitesse initiale

    % Compteur pour l'enregistrement des positions
    n_points = 500;               % Nombre maximum de points à enregistrer
    enregistrement_interval = floor(n_steps / n_points);
    compteur_enregistrement = 1;

    % Initialisation des variables de sortie
    ti = [];
    x = [];
    y = [];
    z = [];

    % Simulation du mouvement avec RK4
    for i = 1:n_steps-1
        % Calcul des coefficients k pour la position et la vitesse

        % k1
        a1 = acceleration(r(i,:), v(i,:), option, m_balle, g, k_visc, S_magnus, w_init);
        k1_v = a1 * dt;
        k1_r = v(i,:) * dt;

        % k2
        v_temp = v(i,:) + 0.5 * k1_v;
        r_temp = r(i,:) + 0.5 * k1_r;
        a2 = acceleration(r_temp, v_temp, option, m_balle, g, k_visc, S_magnus, w_init);
        k2_v = a2 * dt;
        k2_r = v_temp * dt;

        % k3
        v_temp = v(i,:) + 0.5 * k2_v;
        r_temp = r(i,:) + 0.5 * k2_r;
        a3 = acceleration(r_temp, v_temp, option, m_balle, g, k_visc, S_magnus, w_init);
        k3_v = a3 * dt;
        k3_r = v_temp * dt;

        % k4
        v_temp = v(i,:) + k3_v;
        r_temp = r(i,:) + k3_r;
        a4 = acceleration(r_temp, v_temp, option, m_balle, g, k_visc, S_magnus, w_init);
        k4_v = a4 * dt;
        k4_r = v_temp * dt;

        % Mise à jour de la vitesse et de la position
        v(i+1,:) = v(i,:) + (1/6)*(k1_v + 2*k2_v + 2*k3_v + k4_v);
        r(i+1,:) = r(i,:) + (1/6)*(k1_r + 2*k2_r + 2*k3_r + k4_r);

        % Enregistrement des positions pour le tracé
        if mod(i, enregistrement_interval) == 0 || i == 1
            ti(compteur_enregistrement) = t(i);
            x(compteur_enregistrement) = r(i,1);
            y(compteur_enregistrement) = r(i,2);
            z(compteur_enregistrement) = r(i,3);
            compteur_enregistrement = compteur_enregistrement + 1;
        end

        % Conditions d'arrêt
        % Balle touche le sol
        if r(i+1,3) - r_balle <= 0
            coup = 3;
            break;
        end

        % Balle touche le filet
        h_filet = 0.1525;         % Hauteur du filet (m)
        x_filet = 1.37;           % Position en x du filet (milieu de la table)
        l_filet = 1.83;           % Largeur du filet (m)
        y_filet_min = -0.1525;    % Le filet dépasse de chaque côté de 15.25 cm
        y_filet_max = 1.525 + 0.1525;

        if r(i+1,3) - r_balle <= h_filet && abs(r(i+1,1) - x_filet) <= r_balle && ...
           r(i+1,2) >= y_filet_min && r(i+1,2) <= y_filet_max
            coup = 2;
            break;
        end

        % Balle touche la table
        h_table = 0.76;           % Hauteur de la table (m)
        if r(i+1,3) - r_balle <= h_table && r(i+1,1) >= 0 && r(i+1,1) <= 2.74 && ...
           r(i+1,2) >= 0 && r(i+1,2) <= 1.525
            % Déterminer si le coup est réussi ou non
            if r(i+1,1) > x_filet
                coup = 0; % Le coup est réussi (balle atterrit du côté adverse)
            else
                coup = 1; % Coup raté (balle atterrit du côté du joueur)
            end
            break;
        end
    end

    % Résultats finaux
    vbf = v(i,:);                  % Vitesse finale

    % Si la simulation atteint la fin sans conditions d'arrêt
    if i == n_steps - 1
        coup = 3; % Considérer que la balle est sortie du jeu
    end

    % Enregistrement des dernières positions si nécessaire
    if length(ti) < n_points
        ti(compteur_enregistrement) = t(i+1);
        x(compteur_enregistrement) = r(i+1,1);
        y(compteur_enregistrement) = r(i+1,2);
        z(compteur_enregistrement) = r(i+1,3);
    end
end

% Fonction pour calculer l'accélération
function a = acceleration(r, v, option, m_balle, g, k_visc, S_magnus, w_init)
    % Force gravitationnelle
    F_grav = [0, 0, -m_balle * g];

    % Force de frottement visqueux (option 2 et 3)
    if option >= 2
        F_frott = -k_visc * norm(v) * v;
    else
        F_frott = [0, 0, 0];
    end

    % Force de Magnus (option 3)
    if option == 3
        F_magnus = S_magnus * cross(w_init, v);
    else
        F_magnus = [0, 0, 0];
    end

    % Somme des forces
    F_totale = F_grav + F_frott + F_magnus;

    % Accélération de la balle
    a = F_totale / m_balle;
end


%appels de la fonction sur les 4 essais
% Initialisation des paramètres pour les quatre essais
rbi_all = {
    [0.00, 0.50, 1.10],      % Essai 1
    [0.00, 0.40, 1.14],      % Essai 2
    [2.74, 0.50, 1.14],      % Essai 3
    [0.00, 0.30, 1.00]       % Essai 4
};

vbi_all = {
    [4.00, 0.00, 0.80],      % Essai 1
    [10.00, 1.00, 0.20],     % Essai 2
    [-5.00, 0.00, 0.20],     % Essai 3
    [10.00, -2.00, 0.20]     % Essai 4
};

wbi_all = {
    [0.00, -70.00, 0.00],        % Essai 1
    [0.00, 100.00, -50.00],      % Essai 2
    [0.00, 100.00, 0.00],        % Essai 3
    [0.00, 10.00, -100.00]       % Essai 4
};

options = [1, 2, 3]; % Les trois options de simulation

types_coup = {'Coup réussi (0)', 'Coup raté du côté du joueur (1)', 'Coup raté, balle touche le filet (2)', 'Coup raté, balle sort du jeu ou touche le sol (3)'};

% Boucle sur les essais
for essai = 1:4
    rbi = rbi_all{essai};
    vbi = vbi_all{essai};
    wbi = wbi_all{essai};

    fprintf("=== Résultats pour l'Essai %d ===\n", essai);

    % Boucle sur les options
    for opt = options
        [coup, vbf, ti, x, y, z] = Devoir2(opt, rbi, vbi, wbi);

        fprintf('Option %d:\n', opt);
        fprintf('  Coup: %d (%s)\n', coup, types_coup{coup+1});
        fprintf('  Vitesse finale (vbf): [%.4f, %.4f, %.4f] m/s\n', vbf);
        fprintf('  Temps final (tf): %.4f s\n', ti(end));
        fprintf('  Position finale (x, y, z): [%.4f, %.4f, %.4f] m\n\n', x(end), y(end), z(end));

    end
    fprintf('\n');
end



