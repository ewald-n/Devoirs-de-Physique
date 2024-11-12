function [Resultat, blocf, ballef, Post] = Devoir3(vbloc1, avbloc1, tl, vballe1)
    % Devoir3 - Simulation de la collision entre une balle et un bloc avec méthode de Runge-Kutta 4

    % Constantes
    g = [0; 0; -9.8];               % Accélération gravitationnelle (m/s^2)
    coefficient_restitution = 0.8;  % Coefficient de restitution
    masse_bloc = 1.2;               % Masse du bloc (kg)
    arete_bloc = 0.08;              % Longueur du côté du bloc (m)
    masse_balle = 0.05;             % Masse de la balle (kg)
    rayon_balle = 0.02;             % Rayon de la balle (m)

    % Conditions initiales
    r_bloc = [3; 3; 1];             % Position initiale du bloc (m)
    %r_bloc = [0.3; 0; 1.96];             % Position initiale du bloc (m)
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

            q_balle_etat = SEDRK4t0(q_balle_etat, t , dt, @eqMvtBalle, {g});
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
            dimensions_bloc = [arete_bloc arete_bloc arete_bloc];
            [collision p_intersec] = detectionCollisionSphereBloc(r_balle, rayon_balle, r_bloc, dimensions_bloc, q_bloc);
            if collision
                collision_detectee = true;
                Resultat = 0;

                % Enregistrer les vitesses avant la collision
                v_bloc_avant = v_bloc;
                omega_bloc_avant = omega_bloc;
                v_balle_avant = v_balle;
                omega_balle_avant = omega_balle;

                [v_bloc_apres, omega_bloc_apres, v_balle_apres, omega_balle_apres] = ...
                    ApresCollision(p_intersec, r_balle, v_balle, r_bloc, v_bloc, omega_bloc);

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
        ballef = [v_balle_avant, v_balle_apres; omega_balle_avant, omega_balle_avant]; % omega_balle remains the same
    else
        % Pas de collision, les vitesses restent les mêmes
        blocf = [v_bloc_avant, v_bloc_avant; omega_bloc_avant, omega_bloc_avant];
        ballef = [v_balle_avant, v_balle_avant; omega_balle_avant, omega_balle_avant];
    end

    % S'assurer que les sorties sont de taille 6x2 sinon erreur octave
    blocf = reshape(blocf, [6, 2]);
    ballef = reshape(ballef, [6, 2]);
end

function [vitBloc, wBloc, vitBalle, wBalle] = ApresCollision(p_intersec, posBalle, vitBalle, posBloc, vitBloc, wBloc)
    restitution = 0.8;

    mBloc = 1.2;          % Masse du bloc (kg)
    coteBloc = 0.08;      % Longueur du côté du bloc (m)

    mBalle = 0.05;        % Masse de la balle (kg)
    rBalle = 0.02;        % Rayon de la balle (m)
    wBalle = [0; 0; 0];   % Vitesse angulaire de la balle (rad/s)

    % Moment d'inertie du bloc
    IBloc = (1/6) * mBloc * coteBloc^2 * eye(3);

    % Moment d'inertie de la balle
    IBalle = (2/5) * mBalle * rBalle^2 * eye(3);

    % Vecteurs de position relatifs des centres de masse aux points de collision
    rBlocp = p_intersec - posBloc;
    rBallep = p_intersec - posBalle;

    % Vitesse relative au point de contact
    vBlocp = vitBloc + cross(wBloc, rBlocp);
    vBallep = vitBalle + cross(wBalle, rBallep);

    % Vecteur normal au point de contact
    normale = (p_intersec - posBalle);
    normale = normale / norm(normale);

    % Vitesse relative dans la direction normale
    vr_ = dot(normale, (vBallep - vBlocp));

    % Calcul des termes pour l'impulsion
    GA = dot(normale, cross((IBalle \ cross(rBallep, normale)), rBallep));
    GB = dot(normale, cross((IBloc \ cross(rBlocp, normale)), rBlocp));

    alpha = 1 / ((1 / mBalle) + (1 / mBloc) + GA + GB);

    impulsionConstant = -alpha * (1 + restitution) * vr_;
    impulsionVector = normale * impulsionConstant;

    % Mise à jour des vitesses linéaires
    vitBalle = vitBalle + (impulsionVector / mBalle);
    vitBloc = vitBloc - (impulsionVector / mBloc);

          disp(wBloc)
    % Mise à jour des vitesses angulaires
    wBloc = wBloc + (IBloc \ cross(rBlocp, -impulsionVector));
    wBalle = wBalle + (IBalle \ cross(rBallep, impulsionVector));
        disp(wBloc)
end

function dq = eqMvtBloc(q, t, params)
    g = params{1};
    position = q(1:3);
    vitesse = q(4:6);
    acceleration = g;
    dq = [vitesse; acceleration];
end

function dq = eqMvtBalle(q, t, params)
    g = params{1};
    position = q(1:3);
    vitesse = q(4:6);
    acceleration = g;
    dq = [vitesse; acceleration];
end

function qs = SEDRK4t0(q0, t0, DeltaT, g, params)
    k1 = feval(g, q0, t0, params);
    k2 = feval(g, q0 + k1 * DeltaT / 2, t0 + DeltaT / 2, params);
    k3 = feval(g, q0 + k2 * DeltaT / 2, t0 + DeltaT / 2, params);
    k4 = feval(g, q0 + k3 * DeltaT, t0 + DeltaT, params);
    qs = q0 + DeltaT * (k1 + 2 * k2 + 2 * k3 + k4) / 6;
end

function q = mettreAJourQuaternion(q, omega, dt)
    angle = norm(omega * dt);
    if angle > 0
        axe = omega / norm(omega);
        q_rot = [cos(angle/2); sin(angle/2) * axe];
        q = QRotation(q_rot, q);
        q = q / norm(q);
    end
end

function q_out = QRotation(q1, q2)
    % Quaternion multiplication: q_out = q1 * q2
    w1 = q1(1); v1 = q1(2:4);
    w2 = q2(1); v2 = q2(2:4);
    w_out = w1 * w2 - dot(v1, v2);
    v_out = w1 * v2 + w2 * v1 + cross(v1, v2);
    q_out = [w_out; v_out];
end

