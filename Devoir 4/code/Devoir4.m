function [xi, yi, zi, face] = Devoir4(nout, nin, poso)
    poso = poso(:);

    N = 1000;
    M = 1000;

    ANG_POLAIRE_MIN = -45;
    ANG_POLAIRE_MAX = 45;
    ANG_AZIMUTAL_MIN = -45;
    ANG_AZIMUTAL_MAX = 45;

    OUTSIDE = true;
    INSIDE = false;

    cm   = [4;4;11]; %--- centre de masse ellipsoïde
    rad    = 3; %--- x^2/(rad^2), y^2/(rad^2)
    bval   = 9; %--- z^2/(bval^2)

    finalResults = [];

    dirPosVersCentre = cm - poso;
    [angPolaireCentre, angAzimutalCentre] = calculerAnglesCentre(dirPosVersCentre);

    anPolList = genererVecteurLineaire(N, ANG_POLAIRE_MIN + angPolaireCentre, ANG_POLAIRE_MAX + angPolaireCentre);
    anAxiList = genererVecteurLineaire(M, ANG_AZIMUTAL_MIN + angAzimutalCentre, ANG_AZIMUTAL_MAX + angAzimutalCentre);

    vecLumList = calculerVecDirectionLum(anPolList, anAxiList);

    [vecLumList, ptIntersection, distancesParcourues] = findLinesIntersectEllipsoid(poso, vecLumList, cm, [rad, rad, bval], OUTSIDE);
    normales = calculerNormalesEllipsoide(ptIntersection, cm, [rad, rad, bval]);

    vecJ = cross(vecLumList, normales);
    vecJ = vecJ ./ vecnorm(vecJ);

    vecK = cross(normales, vecJ);

    st = nin / nout * dot(vecLumList, vecK);
    maskRefraction = st < 1;

    st = st(maskRefraction);
    normales = normales(:, maskRefraction);
    vecLumList = vecLumList(:, maskRefraction);
    distancesParcourues = distancesParcourues(maskRefraction);
    ptIntersection = ptIntersection(:, maskRefraction);
    vecK = vecK(:, maskRefraction);
    % vecJ = vecJ(:, maskRefraction);

    finalResults = zeros(3, length(st));
    nfinalResults = 0;
    face = zeros(1, length(st));

    ut = -normales .* sqrt(1 - st.^2) + vecK .* st;
    ut = ut ./ vecnorm(ut); % TODO: necessaire unitaires?

    [areIntersecting, distancesParcouruesTouchee, facesTemp] = doRaysIntersectPrism(ptIntersection, ut, distancesParcourues);

    finalResults(:, (nfinalResults + 1):(nfinalResults + length(distancesParcouruesTouchee))) = vecLumList(:, areIntersecting) .* distancesParcouruesTouchee + poso;
    face((nfinalResults + 1):(nfinalResults + length(distancesParcouruesTouchee))) = facesTemp;
    nfinalResults = nfinalResults + length(distancesParcouruesTouchee);
    
    ui = ut(:, ~areIntersecting);

    nReflexInterne = 0;
    while nReflexInterne < 100
        vecLumList = vecLumList(:, ~areIntersecting);
        distancesParcourues = distancesParcourues(~areIntersecting);
        ptIntersection = ptIntersection(:, ~areIntersecting);

        [allo, ptIntersection, distancesParcouruesTemp] = findLinesIntersectEllipsoid(ptIntersection, ui, cm, [rad, rad, bval], INSIDE);
        distancesParcourues = distancesParcourues + distancesParcouruesTemp;
        normales = -calculerNormalesEllipsoide(ptIntersection, cm, [rad, rad, bval]); % TODO: unitaires?

        vecJ = cross(vecLumList, normales);
        vecJ = vecJ ./ vecnorm(vecJ);

        vecK = cross(normales, vecJ);

        st = nout / nin * dot(vecLumList, vecK);
        maskReflexion = st > 1;

        ui = ui(:, maskReflexion);
        normales = normales(:, maskReflexion);
        ptIntersection = ptIntersection(:, maskReflexion);
        distancesParcourues = distancesParcourues(maskReflexion);

        if isempty(ui)
            break;
        else
            nReflexInterne = nReflexInterne + 1;
        end

        ur = ui - 2 * normales .* dot(ui, normales);

        [areIntersecting, distancesParcouruesTouchee, facesTemp] = doRaysIntersectPrism(ptIntersection, ur, distancesParcourues);

        if ~isempty(distancesParcouruesTouchee)
            finalResults(:, (nfinalResults + 1):(nfinalResults + length(distancesParcouruesTouchee))) = vecLumList(:, areIntersecting) .* distancesParcouruesTouchee + poso;
            face((nfinalResults + 1):(nfinalResults + length(distancesParcouruesTouchee))) = facesTemp;
            nfinalResults = nfinalResults + length(distancesParcouruesTouchee);
        end

        ui = ur(:, ~areIntersecting);
    end

    xi = finalResults(1, 1:nfinalResults);
    yi = finalResults(2, 1:nfinalResults);
    zi = finalResults(3, 1:nfinalResults);
    face = face(1:nfinalResults);
end

function [anPol, anAxi] = calculerAnglesCentre(dirPosVersCentre)
    % Cette fonction calcule l'angle polaire et azimutal du vecteur
    % dirPosVersCentre par rapport à l'axe z.

    dirPosVersCentre = dirPosVersCentre / norm(dirPosVersCentre);

    % Calcul de l'angle polaire
    anPol = acos(dirPosVersCentre(3));
    anPol = rad2deg(anPol);

    % Calcul de l'angle azimutal
    anAxi = atan2(dirPosVersCentre(2), dirPosVersCentre(1));
    anAxi = rad2deg(anAxi);
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
    Z = repmat(Z, 1, length(anAxi));

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

function [lineDirList, intersectionPoint, d] = findLinesIntersectEllipsoid(linePoint, lineDirList, ellipsoidCenter, ellipsoidRadii, isOutside)
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
    B = 2 * ((p(1, :) .* lineDirList(1, :) / a^2) + (p(2, :) .* lineDirList(2, :) / b^2) + (p(3, :) .* lineDirList(3, :) / c^2));
    C = (p(1, :).^2 / a^2) + (p(2, :).^2 / b^2) + (p(3, :).^2 / c^2) - 1;

    % Discriminant de l'équation quadratique
    discriminants = B.^2 - 4 * A .* C;

    if isOutside
        % La droite traverse l'ellipsoïde si le discriminant est positif ou nul
        maskTouche = discriminants >= 0;

        lineDirList = lineDirList(:, maskTouche);
        A = A(maskTouche);
        B = B(maskTouche);
        discriminants = discriminants(maskTouche); 
    end

    % Calculer les distances d'intersection d1 et d2
    d1 = (-B + sqrt(discriminants)) ./ (2 * A);
    d2 = (-B - sqrt(discriminants)) ./ (2 * A);

    if isOutside
        % Choisir la plus petite d positive
        d = min(d1, d2); 
        maskPositif = d > 0;

        d = d(maskPositif);
        lineDirList = lineDirList(:, maskPositif);
    else
        % Choisir la plus grande d positive
        d = max(d1, d2);
    end

    % Calculer le point d'intersection
    intersectionPoint = linePoint + d .* lineDirList;
end

function vecNormale = calculerNormalesEllipsoide(ptIntersection, ellipsoidCenter, ellipsoidRadii)
    % Cette fonction calcule la normale à un ellipsoïde à un point donné.    
    vecNormale = (ptIntersection - ellipsoidCenter) ./ (ellipsoidRadii'.^2);
    vecNormale = vecNormale ./ vecnorm(vecNormale);
end

% IA
function [maskIntersect, distancesParcouruesTouchee, facesTouchees] = doRaysIntersectPrism(raysOrigin, raysDir, distancesParcourues)
    % Cette fonction détermine si un rayon touche un prisme rectangulaire.
    % raysOrigin: origine du rayon [x0, y0, z0]
    % raysDir: direction du rayon [dx, dy, dz]
    % intersects: booléen indiquant si le rayon intersecte le prisme

    LAME = [3 4; 3 5; 12 17];

    % Initialiser les paramètres tmin et tmax
    tmin = (LAME(:, 1) - raysOrigin) ./ raysDir;
    tmax = (LAME(:, 2) - raysOrigin) ./ raysDir;

    % Assurer que tmin est toujours le plus petit et tmax le plus grand
    [t1, indexMin] = min(cat(3, tmin, tmax), [], 3);
    t2 = max(tmin, tmax);

    % Trouver les plus grandes valeurs de t1 et les plus petites valeurs de t2
    [t_enter, indexDim] = max(t1);
    t_exit = min(t2);

    % Le rayon intersecte le prisme si t_enter <= t_exit et t_enter >= 0
    maskIntersect = (t_enter <= t_exit) & (t_enter >= 0);


    % Calculer les distances parcourues
    distancesParcouruesTouchee = distancesParcourues(:, maskIntersect) + t_enter(maskIntersect);

    % Calculer les faces touchées
    indexDim = indexDim(maskIntersect);
    indexMin = indexMin(:, maskIntersect);
    facesTouchees = 2 * indexDim + indexMin(indexDim) - 2;
end
