function [Resultat, blocf, ballef, Post] = Devoir3(vbloc1, avbloc1, tl, vballe1)
    % Devoir3 - Simulation de la collision entre une balle et un bloc avec méthode de Runge-Kutta 4
    %
    % Entrées :
    %   vbloc1  - Vecteur de 3 éléments contenant la vitesse linéaire initiale du bloc
    %   avbloc1 - Vecteur de 3 éléments contenant la vitesse angulaire initiale du bloc
    %   tl      - Temps où la balle est lancée
    %   vballe1 - Vecteur de 3 éléments contenant la vitesse linéaire initiale de la balle
    %
    % Sorties :
    %   Resultat - 0 si collision a eu lieu, -1 si la balle touche le sol en premier,
    %              1 si le bloc touche le sol en premier
    %   blocf    - Matrice 6x2 des vitesses du bloc avant et après la collision
    %   ballef   - Matrice 6x2 des vitesses de la balle avant et après la collision
    %   Post     - Matrice contenant le temps et les positions du bloc et de la balle

    % Constantes
    g = [0; 0; -9.8];               % Accélération gravitationnelle (m/s^2)
    coefficient_restitution = 0.8;                      % Coefficient de restitution
    masse_bloc = 1.2;                   % Masse du bloc (kg)
    arete_bloc = 0.08;                  % Longueur du côté du bloc (m)
    masse_balle = 0.05;                 % Masse de la balle (kg)
    rayon_balle = 0.02;                 % Rayon de la balle (m)

    % Conditions initiales
    r_bloc = [3; 3; 1];             % Position initiale du bloc (m)
    v_bloc = vbloc1(:);             % Vitesse linéaire initiale du bloc (m/s)
    omega_bloc = avbloc1(:);        % Vitesse angulaire initiale du bloc (rad/s)

    r_balle = [0; 0; 2];            % Position initiale de la balle (m)
    v_balle = [0; 0; 0];            % Vitesse linéaire initiale de la balle (m/s)
    omega_balle = [0; 0; 0];        % Vitesse angulaire initiale de la balle (rad/s)

    % Moment d'inertie du bloc (cube autour de son centre)
    I_bloc = (1/6) * masse_bloc * arete_bloc^2 * eye(3);

    % Paramètres temporels
    dt = 0.0001;                    % Pas de temps (s)
    t = 0;                          % Temps initial
    t_max = 10;                     % Temps maximal de simulation (s)

    % Initialisation des sorties
    Post = [];
    collision_detectee = false;
    collision_sol = false;
    Resultat = [];

    % Variables pour stocker les vitesses avant et après la collision
    v_bloc_avant = [];
    v_bloc_apres = [];
    omega_bloc_avant = [];
    omega_bloc_apres = [];
    v_balle_avant = [];
    v_balle_apres = [];
    omega_balle_avant = [];
    omega_balle_apres = [];

    % Quaternion représentant l'orientation du bloc
    q_bloc = [1; 0; 0; 0];

    % États initiaux pour Runge-Kutta
    q_bloc_etat = [r_bloc; v_bloc];
    q_balle_etat = [r_balle; v_balle];
    balle_lancee = false;

    while t <= t_max
        % Mise à jour de l'état du bloc avec Runge-Kutta 4
        q_bloc_etat = SEDRK4t0(q_bloc_etat, t, dt, @eqMvtBloc, {g});
        r_bloc = q_bloc_etat(1:3);
        v_bloc = q_bloc_etat(4:6);
        % Mise à jour de l'orientation du bloc
        q_bloc = mettreAJourQuaternion(q_bloc, omega_bloc, dt);

        % Mise à jour de l'état de la balle avec Runge-Kutta 4 après le temps de lancement
        if t >= tl
            if ~balle_lancee && t >= tl
                balle_lancee = true;
                v_balle = vballe1(:);
                q_balle_etat(4:6) = v_balle;
            end

            q_balle_etat = SEDRK4t0(q_balle_etat, t, dt, @eqMvtBalle, {g});
            r_balle = q_balle_etat(1:3);
            v_balle = q_balle_etat(4:6);
        else
            % Avant le lancement, la balle reste immobile
            r_balle = r_balle;
            v_balle = v_balle;
        end

        % Enregistrement des positions et du temps
        Post = [Post, [t; r_bloc; r_balle]];

        % Vérification de la collision entre la balle et le bloc
        if ~collision_detectee && t >= tl
            if detectionCollisionSphereBloc(r_balle, rayon_balle, r_bloc, arete_bloc)
                collision_detectee = true;
                Resultat = 0;

                % Enregistrer les vitesses avant la collision
                v_bloc_avant = v_bloc;
                omega_bloc_avant = omega_bloc;
                v_balle_avant = v_balle;
                omega_balle_avant = omega_balle;

                % Calcul de la réponse à la collision
                [v_bloc_apres, omega_bloc_apres, v_balle_apres, omega_balle_apres] = ...
                    calculReponseCollision(r_bloc, v_bloc, omega_bloc, I_bloc, ...
                                           r_balle, v_balle, omega_balle, masse_bloc, masse_balle, coefficient_restitution, arete_bloc, rayon_balle);

                % Mise à jour des vitesses après la collision
                v_bloc = v_bloc_apres;
                omega_bloc = omega_bloc_apres;
                v_balle = v_balle_apres;
                omega_balle = omega_balle_apres;

                % Mise à jour des états pour Runge-Kutta
                q_bloc_etat(4:6) = v_bloc;
                q_balle_etat(4:6) = v_balle;

                % Simulation se termine après la collision, comme spécifié
                break;
            end
        end

        % Vérification de la collision avec le sol
        if r_bloc(3) <= 0 || (r_balle(3) <= 0 && t >= tl)
            collision_sol = true;
            if r_balle(3) <= 0 && (r_bloc(3) > 0 || t < tl)
                Resultat = -1;  % La balle touche le sol en premier
            else
                Resultat = 1;   % Le bloc touche le sol en premier
            end

            % Enregistrer les vitesses juste avant de toucher le sol
            v_bloc_avant = v_bloc;
            omega_bloc_avant = omega_bloc;
            v_balle_avant = v_balle;
            omega_balle_avant = omega_balle;

            % La simulation se termine à l'impact avec le sol
            break;
        end

        % Avancer le temps
        t = t + dt;
    end

    % Préparation des variables de sortie
    if isempty(Resultat)
        % Si aucune collision n'a été détectée et que le temps maximal est atteint
        Resultat = -2;  % Code d'erreur pour indiquer aucune collision ni impact avec le sol
        % Enregistrer les dernières vitesses connues
        v_bloc_avant = v_bloc;
        omega_bloc_avant = omega_bloc;
        v_balle_avant = v_balle;
        omega_balle_avant = omega_balle;
    end

    % Si collision a eu lieu
    if Resultat == 0
        % Vitesses avant et après la collision
        blocf = [v_bloc_avant, v_bloc_apres; omega_bloc_avant, omega_bloc_apres];
        ballef = [v_balle_avant, v_balle_apres; omega_balle_avant, omega_balle_apres];
    else
        % Pas de collision, les vitesses restent les mêmes
        blocf = [v_bloc_avant, v_bloc_avant; omega_bloc_avant, omega_bloc_avant];
        ballef = [v_balle_avant, v_balle_avant; omega_balle_avant, omega_balle_avant];
    end

    % S'assurer que les sorties sont de taille 6x2 sinon erreur octave
    blocf = reshape(blocf, [6, 2]);
    ballef = reshape(ballef, [6, 2]);
end

function dq = eqMvtBloc(q, t, params)
    % Équations du mouvement pour le bloc
    % q : état du bloc [position; vitesse]
    % dq : dérivée de l'état [vitesse; accélération]
    % params : paramètres supplémentaires (par exemple, accélération gravitationnelle)

    g = params{1};
    position = q(1:3);
    vitesse = q(4:6);

    % Pas de forces autres que la gravité sur le bloc avant collision
    acceleration = g;

    dq = [vitesse; acceleration];
end

function dq = eqMvtBalle(q, t, params)

    g = params{1};
    position = q(1:3);
    vitesse = q(4:6);

    % Pas de forces autres que la gravité sur la balle
    acceleration = g;

    dq = [vitesse; acceleration];
end

function qs = SEDRK4t0(q0, t0, DeltaT, g, params)
    % Solution des équations différentielles par méthode de Runge-Kutta 4
    % q0 : conditions initiales [q(t0)]
    % t0 : temps initial
    % DeltaT : intervalle de temps
    % g : fonction définissant l'équation différentielle dq/dt = g(q, t)
    % params : paramètres supplémentaires pour la fonction g

    k1 = feval(g, q0, t0, params);
    k2 = feval(g, q0 + k1 * DeltaT / 2, t0 + DeltaT / 2, params);
    k3 = feval(g, q0 + k2 * DeltaT / 2, t0 + DeltaT / 2, params);
    k4 = feval(g, q0 + k3 * DeltaT, t0 + DeltaT, params);
    qs = q0 + DeltaT * (k1 + 2 * k2 + 2 * k3 + k4) / 6;
end

function collision = detectionCollisionSphereBloc(r_sphere, rayon, r_boite, taille_boite)
    % Centre de la sphère : r_sphere, rayon
    % Centre de la boîte : r_boite, taille : taille_boite

    % Calculer les coordonnées min et max de la boîte
    min_boite = r_boite - taille_boite / 2;
    max_boite = r_boite + taille_boite / 2;

    % Calculer le point le plus proche sur la boîte du centre de la sphère
    point_le_plus_proche = max(min_boite, min(max_boite, r_sphere));

    % Calculer la distance entre le centre de la sphère et ce point
    distance = norm(point_le_plus_proche - r_sphere);

    % Vérifier si la distance est inférieure ou égale au rayon de la sphère
    collision = distance <= rayon;
end

function [v_bloc_apres, omega_bloc_apres, v_balle_apres, omega_balle_apres] = ...
    calculReponseCollision(r_bloc, v_bloc, omega_bloc, I_bloc, ...
                           r_balle, v_balle, omega_balle, masse_bloc, masse_balle, coefficient_restitution, arete_bloc, rayon_balle)
    % Calcul de la réponse à la collision entre la balle et le bloc

    % Calcul du point de collision
    % On suppose que la collision se produit au point le plus proche sur le bloc du centre de la balle
    min_boite = r_bloc - arete_bloc / 2;
    max_boite = r_bloc + arete_bloc / 2;
    point_collision = max(min_boite, min(max_boite, r_balle));

    % Vecteurs de position relatifs des centres de masse aux points de collision
    r_bloc_pc = point_collision - r_bloc;
    r_balle_pc = point_collision - r_balle;

    % Vitesse relative au point de contact
    v_bloc_pc = v_bloc + cross(omega_bloc, r_bloc_pc);
    v_balle_pc = v_balle + cross(omega_balle, r_balle_pc);

    v_rel = v_balle_pc - v_bloc_pc;

    % Calcul du vecteur normal au point de contact
    normal = (point_collision - r_balle) / norm(point_collision - r_balle);

    % Calcul du scalaire d'impulsion
    numerateur = -(1 + coefficient_restitution) * dot(v_rel, normal);
    denominateur = (1 / masse_balle) + (1 / masse_bloc) + ...
                   dot(normal, cross((I_bloc \ cross(r_bloc_pc, normal)), r_bloc_pc));

    J = numerateur / denominateur;

    % Vecteur d'impulsion
    impulsion = J * normal;

    % Mise à jour des vitesses linéaires
    v_bloc_apres = v_bloc + (impulsion / masse_bloc);      % Le bloc gagne de la quantité de mouvement
    v_balle_apres = v_balle - (impulsion / masse_balle);   % La balle perd de la quantité de mouvement

    % Mise à jour de la vitesse angulaire du bloc
    omega_bloc_apres = omega_bloc + I_bloc \ cross(r_bloc_pc, impulsion);

    % Moment d'inertie de la balle (sphère)
    I_balle = (2/5) * masse_balle * rayon_balle^2;
    % Mise à jour de la vitesse angulaire de la balle
    omega_balle_apres = omega_balle - (cross(r_balle_pc, impulsion) / I_balle);
end


function q = mettreAJourQuaternion(q, omega, dt)
    angle = norm(omega * dt);
    if angle > 0
        % Calcul du quaternion de rotation
        axe = omega / norm(omega);
        q_rot = [cos(angle/2); sin(angle/2) * axe];

        q = QRotation(q_rot, q);
        q = q / norm(q);
    end
end

