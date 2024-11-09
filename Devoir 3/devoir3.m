function [resultat_collision, vitesses_bloc, vitesses_balle, historique_positions] = Devoir3(vitesse_initiale_bloc, vitesse_angulaire_initiale_bloc, temps_lancer, vitesse_initiale_balle)
    % Constantes
    gravite = [0; 0; -9.8];  % Accélération gravitationnelle (m/s^2)
    coefficient_restitution = 0.8;  % Coefficient de restitution

    % Initialisation des propriétés du bloc
    masse_bloc = 1.2;                   % Masse du bloc en kg
    arete_bloc = 0.08;                  % Taille du côté du bloc en mètres
    position_initiale_bloc = [3; 3; 1]; % Position initiale du bloc
    vitesse_bloc = vitesse_initiale_bloc;         % Vitesse initiale du bloc
    vitesse_angulaire_bloc = vitesse_angulaire_initiale_bloc; % Vitesse angulaire initiale du bloc

    % Initialisation des propriétés de la balle
    masse_balle = 0.05;                 % Masse de la balle en kg
    rayon_balle = 0.02;                 % Rayon de la balle en mètres
    position_initiale_balle = [0; 0; 2]; % Position initiale de la balle
    vitesse_balle = vitesse_initiale_balle;      % Vitesse initiale de la balle
    vitesse_angulaire_balle = [0; 0; 0]; % Vitesse angulaire initiale de la balle (nulle)

    % Temps d’échantillonnage
    intervalle_temps = 0.001;   % Intervalle de temps pour la simulation en secondes
    temps_actuel = 0;           % Temps initial

    % Initialisation des sorties
    historique_positions = [];
    collision_detectee = false;

    while ~collision_detectee
        % Mise à jour des positions et vitesses (effet de la gravité)
        position_initiale_bloc = position_initiale_bloc + vitesse_bloc * intervalle_temps;
        vitesse_bloc = vitesse_bloc + gravite * intervalle_temps;
        position_initiale_balle = position_initiale_balle + vitesse_balle * intervalle_temps;
        vitesse_balle = vitesse_balle + gravite * intervalle_temps;

        % Enregistrer les positions actuelles pour historique_positions
        historique_positions = [historique_positions, [temps_actuel; position_initiale_bloc; position_initiale_balle]];

        % Détection de collision entre le bloc et la balle
        distance_centres = norm(position_initiale_bloc - position_initiale_balle);
        if distance_centres <= (arete_bloc / 2 + rayon_balle)
            collision_detectee = true;
            resultat_collision = 0;

            % Calcul de l'impulsion lors de la collision
            vitesse_relative = vitesse_balle - vitesse_bloc;
            impulsion_magnitude = (1 + coefficient_restitution) * (masse_balle * masse_bloc) / (masse_balle + masse_bloc) * ...
                                  dot(vitesse_relative, (position_initiale_balle - position_initiale_bloc)) / distance_centres;
            vecteur_impulsion = impulsion_magnitude * (position_initiale_balle - position_initiale_bloc) / distance_centres;

            % Mise à jour des vitesses après collision
            vitesse_bloc_apres_collision = vitesse_bloc + vecteur_impulsion / masse_bloc;
            vitesse_balle_apres_collision = vitesse_balle - vecteur_impulsion / masse_balle;

            % Enregistrement des vitesses avant et après collision pour les sorties
            vitesses_bloc = [vitesse_bloc, vitesse_bloc_apres_collision; vitesse_angulaire_bloc, vitesse_angulaire_bloc];
            vitesses_balle = [vitesse_balle, vitesse_balle_apres_collision; vitesse_angulaire_balle, vitesse_angulaire_balle];

        elseif position_initiale_bloc(3) <= 0 || position_initiale_balle(3) <= 0
            % Collision avec le sol
            collision_detectee = true;
            resultat_collision = -1 * (position_initiale_balle(3) <= 0) + 1 * (position_initiale_bloc(3) <= 0);

            % Enregistrer les vitesses avant impact au sol pour les sorties
            vitesses_bloc = [vitesse_bloc, vitesse_bloc; vitesse_angulaire_bloc, vitesse_angulaire_bloc];
            vitesses_balle = [vitesse_balle, vitesse_balle; vitesse_angulaire_balle, vitesse_angulaire_balle];
        end

        % Avancer dans le temps
        temps_actuel = temps_actuel + intervalle_temps;
        if temps_actuel > 10  % Limite de temps pour éviter des boucles infinies
            warning('Simulation terminée sans collision ni contact avec le sol.');
            break;
        end
    end
end

