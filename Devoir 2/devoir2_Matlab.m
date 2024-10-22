function [coup, vbf, ti, x, y, z] = Devoir2(option, r_init, v_init, w_init)
    % Paramètres physiques
    g = 9.81;                      % Accélération gravitationnelle (m/s^2)
    m_balle = 2.74e-3;             % Masse de la balle (kg)
    r_balle = 1.99e-2;             % Rayon de la balle (m)
    rho_air = 1.225;               % Densité de l'air (kg/m^3)
    Cd = 0.47;                     % Coefficient de traînée (froment)
    S_balle = pi * r_balle^2;      % Surface de la balle
    k_visc = 0.5 * rho_air * Cd * S_balle; % Coefficient de frottement visqueux
    S_magnus = 4/3 * pi * r_balle^3; % Coefficient de Magnus (à ajuster selon le problème)

    % Initialisation des variables
    dt = 0.01;                     % Pas de temps (s)
    t_max = 10;                    % Durée maximale de simulation (s)
    t = 0:dt:t_max;                % Vecteur temps
    n_steps = length(t);           % Nombre de pas de temps

    % Vecteurs pour stocker les positions et vitesses
    r = zeros(n_steps, 3);         % Position (x, y, z)
    v = zeros(n_steps, 3);         % Vitesse (vx, vy, vz)
    r(1,:) = r_init;               % Position initiale
    v(1,:) = v_init;               % Vitesse initiale

    % Simulation du mouvement
    for i = 1:n_steps-1
        % Force gravitationnelle
        F_grav = [0, 0, -m_balle * g];

        % Force de frottement visqueux (option 2 et 3)
        if option >= 2
            F_frott = -k_visc * norm(v(i,:)) * v(i,:);
        else
            F_frott = [0, 0, 0];
        end

        % Force de Magnus (option 3)
        %if option == 3
            %F_magnus = S_magnus * cross(w_init, v(i,:));
        %else
            F_magnus = [0, 0, 0];
        %end

        % Somme des forces
        F_totale = F_grav + F_frott + F_magnus;

        % Accélération de la balle
        a = F_totale / m_balle;

        % Mise à jour de la vitesse et de la position (intégration Euler)
        v(i+1,:) = v(i,:) + a * dt;
        r(i+1,:) = r(i,:) + v(i,:) * dt;

        % Conditions d'arrêt (filet, table ou sol)
        if r(i+1,3) <= 0  % La balle touche le sol
            coup = 3;
            break;
        elseif r(i+1,1) > 2.74 || r(i+1,1) < 0 % La balle est hors des limites en x
            coup = 3;
            break;
        elseif r(i+1,2) > 1.525 || r(i+1,2) < 0 % La balle est hors des limites en y
            coup = 3;
            break;
        elseif r(i+1,3) <= 0.1525 && r(i+1,1) >= 1.22 && r(i+1,1) <= 1.52 % La balle touche le filet
            coup = 2;
            break;
        elseif r(i+1,3) <= 0.76 && r(i+1,1) >= 0 && r(i+1,1) <= 2.74 % La balle touche la table
            % Déterminer si le coup est réussi ou non
            if r(i+1,1) > 1.37
                coup = 0; % Le coup est réussi
            else
                coup = 1; % Coup raté du côté du joueur
            end
            break;
        end
    end

    % Résultats finaux
    vbf = v(i,:);                  % Vitesse finale
    ti = t(1:i);                   % Temps jusqu'à l'arrêt
    x = r(1:i,1);                  % Positions en x
    y = r(1:i,2);                  % Positions en y
    z = r(1:i,3);                  % Positions en z

    % Coup raté si la balle sort ou touche le sol
    %{
    if r(i,3) <= 0
        coup = 3; % La balle touche le sol en dehors de la table
    elseif r(i,3) <= 0.1525 && r(i,1) >= 1.22 && r(i,1) <= 1.52
        coup = 2; % La balle touche le filet
    else
        coup = 0;
    end
    %}
end
