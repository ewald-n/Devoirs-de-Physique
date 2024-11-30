function [xi, yi, zi, face] = Devoir4(nout, nin, poso)
    poso = poso(:);

    N = 1000;
    M = 1000;

    ANG_POLAIRE_MIN = 0;
    ANG_POLAIRE_MAX = 90;
    ANG_AZIMUTAL_MIN = 0;
    ANG_AZIMUTAL_MAX = 90;

    cm   = [4,4,11]; %--- centre de masse ellipsoïde
    rad    = 3; %--- x^2/(rad^2), y^2/(rad^2)
    bval   = 9; %--- z^2/(bval^2)

    dirPosVersCentre = cm - poso;
    [angPolaireCentre, angAzimutalCentre] = calculerAnglesCentre(dirPosVersCentre);

    anPolList = genererVecteurLineaire(N, ANG_POLAIRE_MIN + angPolaireCentre, ANG_POLAIRE_MAX + angPolaireCentre);
    anAxiList = genererVecteurLineaire(M, ANG_AZIMUTAL_MIN + angAzimutalCentre, ANG_AZIMUTAL_MAX + angAzimutalCentre);

    vecLumList = calculerVecDirectionLum(anPolList, anAxiList);

    [vecLumList, ptIntersection] = findLinesIntersectEllipsoid(poso, vecLumList, cm, [rad, rad, bval]);
    normales = calculerNormalesEllipsoide(ptIntersection, cm, [rad, rad, bval]); % TODO: unitaires?

    vecJ = cross(vecLumList, normales);
    vecJ = vecJ ./ vecnorm(vecJ);

    vecK = cross(normales, vecJ);

    st = nin / nout * dot(vecLumList, vecK);
    maskRefraction = st < 1;

    st = st(maskRefraction);
    normales = normales(:, maskRefraction);
    vecK = vecK(:, maskRefraction);
    % vecJ = vecJ(:, maskRefraction);
    vecLumList = vecLumList(:, maskRefraction);

    ut = -normales .* sqrt(1 - st.^2) + vecK .* st;



    
end

function [anPol; anAxi] = calculerAnglesCentre(dirPosVersCentre)
    % Cette fonction calcule l'angle polaire et azimutal du vecteur
    % dirPosVersCentre par rapport à l'axe z.

    % Calcul de l'angle polaire
    anPol = acos(dirPosVersCentre(3, :));

    % Calcul de l'angle azimutal
    anAxi = atan2(dirPosVersCentre(2, :), dirPosVersCentre(1, :));
end

function vec = genererVecteurLineaire(N, valeurMin, valeurMax)    
    % Calcul de l'incrément
    increment = (valeurMax - valeurMin) / (2 * N);
    
    % Génération du vecteur
    vec = valeurMin + increment * (1:2:N*2);
end

function matLum = calculerVecDirectionLum(anPol, anAxi)
    % Calcul de la direction de la lumière
    X = sin(anPol)' * cos(anAxi);
    X = reshape(X,1,[]) ;
    Y = sin(anPol)' * sin(anAxi);
    Y = reshape(Y,1,[]) ;
    Z = cos(anPol);
    Z = repmat(Z, 1, size(anAxi));

    matLum = [X; Y; Z];
end

% IA
function intersects = doesLineIntersectEllipsoid(linePoint, lineDir, ellipsoidCenter, ellipsoidRadii)
    % Cette fonction détermine si une droite traverse un ellipsoïde.
    % linePoint: un point sur la droite [x0, y0, z0]
    % lineDir: vecteur directionnel de la droite [dx, dy, dz]
    % ellipsoidCenter: centre de l'ellipsoïde [cx, cy, cz]
    % ellipsoidRadii: rayons de l'ellipsoïde [rx, ry, rz]

    % Déplacer la droite pour que l'ellipsoïde soit centré à l'origine
    p = linePoint - ellipsoidCenter;

    % Paramètres de l'ellipsoïde
    a = ellipsoidRadii(1);
    b = ellipsoidRadii(2);
    c = ellipsoidRadii(3);

    % Coefficients de l'équation quadratique
    A = (lineDir(1)^2 / a^2) + (lineDir(2)^2 / b^2) + (lineDir(3)^2 / c^2);
    B = 2 * ((p(1) * lineDir(1) / a^2) + (p(2) * lineDir(2) / b^2) + (p(3) * lineDir(3) / c^2));
    C = (p(1)^2 / a^2) + (p(2)^2 / b^2) + (p(3)^2 / c^2) - 1;

    % Discriminant de l'équation quadratique
    discriminant = B^2 - 4 * A * C;

    % La droite traverse l'ellipsoïde si le discriminant est positif ou nul
    intersects = discriminant >= 0;
end

function [lineDirList, intersectionPoint] = findLinesIntersectEllipsoid(linePoint, lineDirList, ellipsoidCenter, ellipsoidRadii)
    % Cette fonction détermine si une droite traverse un ellipsoïde.
    % linePoint: un point sur la droite [x0, y0, z0]
    % lineDirList: liste de vecteurs directionnels de la droite [dx, dy, dz]
    % ellipsoidCenter: centre de l'ellipsoïde [cx, cy, cz]
    % ellipsoidRadii: rayons de l'ellipsoïde [rx, ry, rz]

    % Déplacer la droite pour que l'ellipsoïde soit centré à l'origine
    p = linePoint - ellipsoidCenter;

    % Paramètres de l'ellipsoïde
    a = ellipsoidRadii(1);
    b = ellipsoidRadii(2);
    c = ellipsoidRadii(3);

    % Coefficients de l'équation quadratique
    A = (lineDirList(1, :).^2 / a^2) + (lineDirList(2, :).^2 / b^2) + (lineDirList(3, :).^2 / c^2);
    B = 2 * ((p(1) * lineDirList(1, :) / a^2) + (p(2) * lineDirList(2, :) / b^2) + (p(3) * lineDirList(3, :) / c^2));
    C = (p(1)^2 / a^2) + (p(2)^2 / b^2) + (p(3)^2 / c^2) - 1;

    % Discriminant de l'équation quadratique
    discriminants = B.^2 - 4 * A * C;

    % La droite traverse l'ellipsoïde si le discriminant est positif ou nul
    maskTouche = discriminants >= 0;

    lineDirList = lineDirList(:, maskTouche);
    A = A(maskTouche);
    B = B(maskTouche);
    discriminants = discriminants(maskTouche);

    % Calculer les distances d'intersection d1 et d2
    d1 = (-B + sqrt(discriminant)) ./ (2 * A);
    d2 = (-B - sqrt(discriminant)) ./ (2 * A);

    % Choisir la plus petite d positive
    d = min(d1, d2);
    % if t < 0
    %     t = max(t1, t2);
    % end

    % if t < 0
    %     intersects = false;
    %     intersectionPoint = [];
    %     return;
    % end

    % Calculer le point d'intersection
    intersectionPoint = linePoint + d .* lineDirList;
end

function vecNormale = calculerNormalesEllipsoide(ptIntersection, ellipsoidCenter, ellipsoidRadii)
    % Cette fonction calcule la normale à un ellipsoïde à un point donné.    
    vecNormale = (ptIntersection - ellipsoidCenter) ./ (ellipsoidRadii.^2);
end
