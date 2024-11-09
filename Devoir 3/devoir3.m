function [Resultat, blocf, ballef, Post] = Devoir3(vbloci, avbloci, t1, vballei)
    % Constantes
    g = 9.8;  % Accélération gravitationnelle (m/s^2)
    coeff_restitution = 0.8;  % Coefficient de restitution de la collision

    % Initialisation des propriétés du bloc
    mbloc = 1.2;           % Masse du bloc en kg
    Abloc = 0.08;          % Taille du côté du bloc en mètres
    pos_bloc = [3; 3; 1];  % Position initiale du bloc en mètres
    vel_bloc = vbloci;     % Vitesse initiale du bloc
    ang_vel_bloc = avbloci; % Vitesse angulaire initiale du bloc

    % Initialisation des propriétés de la balle
    m_balle = 0.05;        % Masse de la balle en kg
    R_balle = 0.02;        % Rayon de la balle en mètres
    pos_balle = [0; 0; 2]; % Position initiale de la balle
    vel_balle = vballei;   % Vitesse initiale de la balle
    ang_vel_balle = [0; 0; 0];  % Vitesse angulaire initiale de la balle (nulle)

    % Temps d’échantillonnage
    dt = 0.001;  % Intervalle de temps pour la simulation en secondes
    t = 0;       % Temps initial

    % Initialisation des sorties
    Post = [];
    collision_detectee = false;

    while ~collision_detectee
        % Mise à jour des positions et vitesses des objets
        pos_bloc = pos_bloc + vel_bloc * dt;
        vel_bloc = vel_bloc + [0; 0; -g] * dt;  % Effet de la gravité

        pos_balle = pos_balle + vel_balle * dt;
        vel_balle = vel_balle + [0; 0; -g] * dt;  % Effet de la gravité

        % Enregistrer les positions actuelles pour Post
        Post = [Post, [t; pos_bloc; pos_balle]];

        % Détection de la collision bloc-balle
        distance = norm(pos_bloc - pos_balle);
        if distance <= (Abloc / 2 + R_balle)
            collision_detectee = true;
            Resultat = 0;

            % Calcul des vitesses après collision
            vel_bloc_apres = (vel_bloc * (mbloc - coeff_restitution * m_balle) + ...
                              vel_balle * (1 + coeff_restitution) * m_balle) / ...
                              (mbloc + m_balle);

            vel_balle_apres = (vel_balle * (m_balle - coeff_restitution * mbloc) + ...
                               vel_bloc * (1 + coeff_restitution) * mbloc) / ...
                               (mbloc + m_balle);

            % Enregistrement des vitesses dans blocf et ballef
            blocf = [vel_bloc, vel_bloc_apres; ang_vel_bloc, ang_vel_bloc]; % vitesses initiale et finale
            ballef = [vel_balle, vel_balle_apres; ang_vel_balle, ang_vel_balle]; % vitesses initiale et finale
        elseif pos_bloc(3) <= 0 || pos_balle(3) <= 0
            % Collision avec le sol
            collision_detectee = true;
            if pos_balle(3) <= 0
                Resultat = -1;  % Balle touche le sol en premier
            else
                Resultat = 1;   % Bloc touche le sol en premier
            end

            % Enregistrer les vitesses avant impact au sol
            blocf = [vel_bloc, vel_bloc; ang_vel_bloc, ang_vel_bloc];
            ballef = [vel_balle, vel_balle; ang_vel_balle, ang_vel_balle];
        end

        % Avancer dans le temps
        t = t + dt;

        % Limiter la simulation à un temps raisonnable pour éviter les boucles infinies
        if t > 10
            warning('Simulation terminée sans collision ni contact avec le sol.');
            break;
        end
    end
end
