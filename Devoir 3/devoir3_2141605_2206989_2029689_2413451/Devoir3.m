function [Resultat, blocf, ballef, Post] = Devoir3(vbloc1, avbloc1, tl, vballe1)
    % Devoir3 - Simulation de la collision entre une balle et un bloc avec méthode de Runge-Kutta 4

    % Constantes
    F_GRAVITE = [0; 0; -9.8];       % Accélération gravitationnelle (m/s^2)
    COEF_RESTITUTION = 0.8;         % Coefficient de restitution
    ACC_ANG_BLOC = zeros(3, 1);     % Accélération angulaire du bloc (rad/s^2)
    m_bloc = 1.2;                   % Masse du bloc (kg)
    c_bloc = 0.08;                  % Longueur du côté du bloc (m)
    m_balle = 0.05;                 % Masse de la balle (kg)
    rayon_balle = 0.02;             % Rayon de la balle (m)

    % Indexes pour le vecteur q:
    qVITESSE = 1:3;
    qPOSITION = 4:6;
    qV_ANGULAIRE = 7:9;
    qQUATERNION = 10:13;

    % Moment d'inertie du bloc (cube autour de son centre)
    I_bloc = (1/6) * m_bloc * c_bloc^2 * eye(3);
    % Moment d'inertie de la balle
    I_balle = (2/5) * m_balle * rayon_balle^2 * eye(3);

    % Paramètres temporels
    dt = 0.0001;                    % Pas de temps (s)
    t = 0;                          % Temps initial
    t_max = 10;                     % Temps maximal de simulation (s)

    % Conditions initiales
    v_bloc = vbloc1(:);             % Vitesse linéaire initiale du bloc (m/s)
    r_bloc = [3; 3; 1];             % Position initiale du bloc (m)
    w_bloc = avbloc1(:);            % Vitesse angulaire initiale du bloc (rad/s)
    Rq_bloc = [1; 0; 0; 0];         % Quaternion représentant l'orientation du bloc
    r_balle = [0; 0; 2];            % Position initiale de la balle (m)
    v_balle = [0; 0; 0];            % Vitesse linéaire initiale de la balle (m/s)
    w_balle = [0; 0; 0];            % Vitesse angulaire initiale de la balle (rad/s)

    % États initiaux pour Runge-Kutta
    q_bloc_etat = [v_bloc; r_bloc; w_bloc; Rq_bloc];
    q_balle_etat = [v_balle; r_balle];
    balle_lancee = false;

    % Initialisation des sorties
    numIter = 1;
    nElementMax = ceil(t_max / dt) + numIter;
    Post = zeros(7, nElementMax);
    collision_detectee = false;
    collision_sol = false;
    Resultat = [];

    % Variables pour stocker les vitesses avant et après la collision
    v_bloc_avant = [];
    v_bloc_apres = [];
    w_bloc_avant = [];
    w_bloc_apres = [];
    v_balle_avant = [];
    v_balle_apres = [];
    w_balle_avant = w_balle;

    while t <= t_max
        % Mise à jour de l'état du bloc avec Runge-Kutta 4
        q_bloc_etat = SEDRK4t0(q_bloc_etat, t, dt, @eqMvtBloc, {});
        v_bloc = q_bloc_etat(qVITESSE);
        r_bloc = q_bloc_etat(qPOSITION);
        w_bloc = q_bloc_etat(qV_ANGULAIRE);
        Rq_bloc = q_bloc_etat(qQUATERNION); % TODO: normalize quaternion

        % Mise à jour de l'état de la balle avec Runge-Kutta 4 après le temps de lancement
        if t >= tl
            if ~balle_lancee
                balle_lancee = true;
                v_balle = vballe1(:);
                q_balle_etat(qVITESSE) = v_balle;
            end

            q_balle_etat = SEDRK4t0(q_balle_etat, t , dt, @eqMvtBalle, {});
            v_balle = q_balle_etat(qVITESSE);
            r_balle = q_balle_etat(qPOSITION);
        end

        % Enregistrement des positions et du temps
        Post(:, numIter) = [t; r_bloc; r_balle];
        % Vérification de la collision entre la balle et le bloc
        if ~collision_detectee && t >= tl
            dimensions_bloc = [c_bloc c_bloc c_bloc];
            [collision, p_intersec] = detectionCollisionSphereBloc(r_balle, rayon_balle, r_bloc, dimensions_bloc, Rq_bloc);
            if collision
                collision_detectee = true;
                Resultat = 0;

                % Enregistrer les vitesses avant la collision
                v_bloc_avant = v_bloc;
                w_bloc_avant = w_bloc;
                v_balle_avant = v_balle;
                w_balle_avant = w_balle;

                [v_bloc_apres, w_bloc_apres, v_balle_apres] = ...
                    ApresCollision(p_intersec, r_balle, v_balle, r_bloc, v_bloc, w_bloc);

                % Mise à jour des vitesses après la collision
                v_bloc = v_bloc_apres;
                w_bloc = w_bloc_apres;
                v_balle = v_balle_apres;

                % Mise à jour des états pour Runge-Kutta
                q_bloc_etat(qVITESSE) = v_bloc;
                q_bloc_etat(qV_ANGULAIRE) = w_bloc;
                q_balle_etat(qVITESSE) = v_balle;

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
            w_bloc_avant = w_bloc;
            v_balle_avant = v_balle;
            w_balle_avant = w_balle;

            % La simulation se termine à l'impact avec le sol
            break;
        end

        % Avancer le temps
        t = t + dt;
        numIter = numIter + 1;
    end

    % Préparation des variables de sortie
    if isempty(Resultat)
        % Si aucune collision n'a été détectée et que le temps maximal est atteint
        Resultat = -2;  % Code d'erreur pour indiquer aucune collision ni impact avec le sol
        % Enregistrer les dernières vitesses connues
        v_bloc_avant = v_bloc;
        w_bloc_avant = w_bloc;
        v_balle_avant = v_balle;
        w_balle_avant = w_balle;
    end

    % Si collision a eu lieu
    if Resultat == 0
        % Vitesses avant et après la collision
        blocf = [v_bloc_avant, v_bloc_apres; w_bloc_avant, w_bloc_apres];
        ballef = [v_balle_avant, v_balle_apres; w_balle_avant, w_balle_avant];
    else
        % Pas de collision, les vitesses restent les mêmes
        blocf = [v_bloc_avant, v_bloc_avant; w_bloc_avant, w_bloc_avant];
        ballef = [v_balle_avant, v_balle_avant; w_balle_avant, w_balle_avant];
    end

    % S'assurer que les sorties sont de taille 6x2 sinon erreur octave
    blocf = reshape(blocf, [6, 2]);
    ballef = reshape(ballef, [6, 2]);

    % Reduire la taille de Post a son miminum
    Post = Post(:, 1:numIter);


    % Fonctions:

    function [vitBloc, wBloc, vitBalle] = ApresCollision(p_intersec, posBalle, vitBalle, posBloc, vitBloc, wBloc)
        % Vecteurs de position relatifs des centres de masse aux points de collision
        rBlocp = p_intersec - posBloc;
        rBallep = p_intersec - posBalle;

        % Vitesse relative au point de contact
        vBlocp = vitBloc + cross(wBloc, rBlocp);
        vBallep = vitBalle + cross(w_balle, rBallep);

        % Vecteur normal au point de contact
        normale = (posBalle - p_intersec);
        normale = normale / norm(normale);

        % Vitesse relative dans la direction normale
        vr_ = dot(normale, (vBallep - vBlocp));

        % Calcul des termes pour l'impulsion
        GA = dot(normale, cross((I_balle \ cross(rBallep, normale)), rBallep));
        GB = dot(normale, cross((I_bloc \ cross(rBlocp, normale)), rBlocp));

        alpha = 1 / ((1 / m_balle) + (1 / m_bloc) + GA + GB);

        impulsionConstant = -alpha * (1 + COEF_RESTITUTION) * vr_;
        impulsionVector = normale * impulsionConstant;

        % Mise à jour des vitesses linéaires
        vitBalle = vitBalle + (impulsionVector / m_balle);
        vitBloc = vitBloc - (impulsionVector / m_bloc);

        % Mise à jour des vitesses angulaires
        wBloc = wBloc + (I_bloc \ cross(rBlocp, -impulsionVector));
    end

    function dq = eqMvtBloc(q, t, param)
        acceleration = F_GRAVITE;
        vitesse = q(qVITESSE);
        dR_quat = Deriv_Quaternion(q(qQUATERNION), q(qV_ANGULAIRE));
        dq = [acceleration; vitesse; ACC_ANG_BLOC; dR_quat];
    end

    function dq = eqMvtBalle(q, t, param)
        acceleration = F_GRAVITE;
        vitesse = q(qVITESSE);
        dq = [acceleration; vitesse];
    end

    function qs = SEDRK4t0(q0, t0, DeltaT, g, params)
        k1 = feval(g, q0, t0, params);
        k2 = feval(g, q0 + k1 * DeltaT / 2, t0 + DeltaT / 2, params);
        k3 = feval(g, q0 + k2 * DeltaT / 2, t0 + DeltaT / 2, params);
        k4 = feval(g, q0 + k3 * DeltaT, t0 + DeltaT, params);
        qs = q0 + DeltaT * (k1 + 2 * k2 + 2 * k3 + k4) / 6;
    end

    function dR = Deriv_Quaternion(R_quat, w)
        % Selon (2.95) du manuel de l'étudiant
        w_quat = [0; w];
        dR = QProduit(R_quat, w_quat) / 2;
    end
end
