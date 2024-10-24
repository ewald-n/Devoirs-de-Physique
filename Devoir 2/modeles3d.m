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



%3d////////////////////////////////////////////////////////////////////////////////////////////
%//////////////////////////////////////////////////////////////////////////////////////////////
% Essai 4
rbi = [0.00, 0.30, 1.00];
vbi =  [10.00, -2.00, 0.20];
wbi = [0.00, 10.00, -100.00];

% Simulations pour les trois options
options = [1, 2, 3];
couleurs = {'r', 'b', 'k'}; % Rouge, Bleu, Noir
trajectoires = cell(1, 3); % Pour stocker les trajectoires de chaque option

for idx = 1:length(options)
    opt = options(idx);
    [coup, vbf, ti, x, y, z] = Devoir2(opt, rbi, vbi, wbi);
    trajectoires{idx} = struct('x', x, 'y', y, 'z', z, 'couleur', couleurs{idx});
end


global longueur largeur hauteur hfilet lfilet dir ;
longueur = 2.74;    % Longueur de la table en mètres
largeur = 1.525;    % Largeur de la table en mètres
hauteur = 0.76;     % Hauteur de la table en mètres
hfilet = 0.1525;    % Hauteur du filet en mètres
lfilet = 1.83;      % Largeur du filet en mètres
dir = 1;            % Direction (peut-être une variable utilisée ailleurs)
bandes=5/100;
epaisseur=10/100;
lo2=longueur/2;
xoff=0.0;
xcp=3*longueur/8;
ycp=3*largeur/8;
ba2=bandes/2;
la2=largeur/2;
yoff=0.0;
epp=3/100;
df=(lfilet-largeur)/2;
LigneX1=[xoff    xoff+longueur    xoff+longueur   xoff   xoff;
         -ba2+la2   -ba2+la2    ba2+la2    ba2+la2   -ba2+la2 ;
          hauteur     hauteur     hauteur     hauteur      hauteur ];
LigneX2=[xoff    xoff+longueur    xoff+longueur   xoff   xoff;
         yoff   yoff   yoff+bandes   yoff+bandes    yoff ;
          hauteur     hauteur     hauteur     hauteur      hauteur ];
LigneX3=[xoff    xoff+longueur    xoff+longueur   xoff   xoff;
          yoff+largeur-bandes    yoff+largeur-bandes   yoff+largeur   yoff+largeur    yoff+largeur-bandes ;
          hauteur     hauteur     hauteur     hauteur      hauteur ];
LigneY1=[xoff    xoff+bandes   xoff+bandes   xoff   xoff;
         yoff   yoff    yoff+largeur    yoff+largeur   yoff ;
          hauteur     hauteur     hauteur     hauteur      hauteur ];
LigneY2=[xoff+longueur-bandes    xoff+longueur     xoff+longueur   xoff+longueur-bandes   xoff+longueur-bandes;
         yoff   yoff    yoff+largeur    yoff+largeur   yoff ;
          hauteur     hauteur     hauteur     hauteur      hauteur ];
TableX1=[xoff+bandes    xoff+longueur-bandes    xoff+longueur-bandes   xoff+bandes   xoff+bandes;
         yoff+bandes   yoff+bandes    -ba2    -ba2    yoff+bandes ;
          hauteur     hauteur     hauteur     hauteur      hauteur ];
TableX2=[xoff+bandes    xoff+longueur-bandes    xoff+longueur-bandes   xoff+bandes   xoff+bandes;
          ba2    ba2    yoff+largeur-bandes    yoff+largeur-bandes    ba2 ;
          hauteur     hauteur     hauteur     hauteur      hauteur ];
CoteX1=[xoff    xoff+longueur    xoff+longueur   xoff   xoff;
         yoff   yoff   yoff   yoff   yoff ;
          hauteur-epaisseur     hauteur-epaisseur     hauteur     hauteur      hauteur-epaisseur ];
CoteY1=[xoff    xoff   xoff   xoff   xoff;
         yoff    yoff+largeur    yoff+largeur   yoff   yoff ;
          hauteur-epaisseur   hauteur-epaisseur     hauteur     hauteur      hauteur-epaisseur ];
Patte1F=[-xcp+epp+lo2 -xcp-epp+lo2 -xcp-epp+lo2 -xcp+epp+lo2 -xcp+epp+lo2;
         -ycp+la2-epp -ycp+la2-epp -ycp+la2-epp -ycp+la2-epp -ycp+la2-epp;
         0 0 hauteur hauteur 0];
Patte1C=[-xcp-epp+lo2 -xcp-epp+lo2 -xcp-epp+lo2 -xcp-epp+lo2 -xcp-epp+lo2;
         -ycp+la2-epp -ycp+la2+epp -ycp+la2+epp -ycp+la2-epp -ycp+la2-epp;
         0 0 hauteur hauteur 0];
Patte2F=[ xcp+epp+lo2  xcp-epp+lo2  xcp-epp+lo2  xcp+epp+lo2  xcp+epp+lo2;
         -ycp+la2-epp -ycp+la2-epp -ycp+la2-epp -ycp+la2-epp -ycp+la2-epp;
         0 0 hauteur hauteur 0];
Patte2C=[ xcp-epp+lo2  xcp-epp+lo2  xcp-epp+lo2  xcp-epp+lo2  xcp-epp+lo2;
         -ycp+la2-epp -ycp+la2+epp -ycp+la2+epp -ycp+la2-epp -ycp+la2-epp;
         0 0 hauteur hauteur 0];
Patte3F=[-xcp+epp+lo2 -xcp-epp+lo2 -xcp-epp+lo2 -xcp+epp+lo2 -xcp+epp+lo2;
          ycp+la2-epp  ycp+la2-epp  ycp+la2-epp  ycp+la2-epp  ycp+la2-epp;
         0 0 hauteur hauteur 0];
Patte3C=[-xcp-epp+lo2 -xcp-epp+lo2 -xcp-epp+lo2 -xcp-epp+lo2 -xcp-epp+lo2;
          ycp+la2-epp  ycp+la2+epp  ycp+la2+epp  ycp+la2-epp  ycp+la2-epp;
         0 0 hauteur hauteur 0];
Patte4F=[ xcp+epp+lo2  xcp-epp+lo2  xcp-epp+lo2  xcp+epp+lo2  xcp+epp+lo2;
          ycp+la2-epp  ycp+la2-epp  ycp+la2-epp  ycp+la2-epp  ycp+la2-epp;
         0 0 hauteur hauteur 0];
Patte4C=[ xcp-epp+lo2  xcp-epp+lo2  xcp-epp+lo2  xcp-epp+lo2  xcp-epp+lo2;
          ycp+la2-epp  ycp+la2+epp  ycp+la2+epp  ycp+la2-epp  ycp+la2-epp;
         0 0 hauteur hauteur 0];
Filet=[0+lo2 0+lo2 0+lo2 0+lo2 0+lo2;
       yoff-df yoff-df yoff+largeur+df yoff+largeur+df yoff-df;
        hauteur hauteur+hfilet hauteur+hfilet hauteur hauteur];


% Création d'une nouvelle figure
figure;
hold on;

% Tracé de la table
fill3(LigneX1(1,:), LigneX1(2,:), LigneX1(3,:), [1 1 1]);
fill3(LigneX2(1,:), LigneX2(2,:), LigneX2(3,:), [1 1 1]);
fill3(LigneX3(1,:), LigneX3(2,:), LigneX3(3,:), [1 1 1]);
fill3(LigneY1(1,:), LigneY1(2,:), LigneY1(3,:), [1 1 1]);
fill3(LigneY2(1,:), LigneY2(2,:), LigneY2(3,:), [1 1 1]);
fill3(TableX1(1,:), TableX1(2,:), TableX1(3,:), [0.5 1 0.5]);
fill3(TableX2(1,:), TableX2(2,:), TableX2(3,:), [0.5 1 0.5]);
fill3(CoteX1(1,:), CoteX1(2,:), CoteX1(3,:), [0.5 1 0.5]);
fill3(CoteY1(1,:), CoteY1(2,:), CoteY1(3,:), [0.5 1 0.5]);
fill3(Patte1F(1,:), Patte1F(2,:), Patte1F(3,:), [0 0 0]);
fill3(Patte1C(1,:), Patte1C(2,:), Patte1C(3,:), [0 0 0]);
fill3(Patte2F(1,:), Patte2F(2,:), Patte2F(3,:), [0 0 0]);
fill3(Patte2C(1,:), Patte2C(2,:), Patte2C(3,:), [0 0 0]);
fill3(Patte3F(1,:), Patte3F(2,:), Patte3F(3,:), [0 0 0]);
fill3(Patte3C(1,:), Patte3C(2,:), Patte3C(3,:), [0 0 0]);
fill3(Patte4F(1,:), Patte4F(2,:), Patte4F(3,:), [0 0 0]);
fill3(Patte4C(1,:), Patte4C(2,:), Patte4C(3,:), [0 0 0]);
fill3(Filet(1,:), Filet(2,:), Filet(3,:), [1 1 0]);

% Tracé des trajectoires
legend_entries = {'Option 1', 'Option 2', 'Option 3'};
legend_handles = zeros(1, 3);

for idx = 1:length(trajectoires)
    traj = trajectoires{idx};
    h = plot3(traj.x, traj.y, traj.z, 'Color', traj.couleur, 'LineWidth', 1, 'DisplayName', legend_entries{idx});
    legend_handles(idx) = h;
end

% Configuration des axes et de la vue
axis equal;
xlabel('Position en X (m)');
ylabel('Position en Y (m)');
zlabel('Position en Z (m)');
title("Trajectoires de la balle pour l'Essai 1");
grid on;
view(3);

% Ajouter la légende
legend(legend_handles, legend_entries);

