function [collision, p_intersec] = detectionCollisionSphereBloc(r_sphere, rayon, r_boite, taille_boite, q_bloc)
    %methode du cours 5
    collision = false;
    p_intersec = [];

    a = taille_boite(1);
    b = taille_boite(2);
    c = taille_boite(3);

    r_boite = r_boite(:);
    half_sizes = [a/2; b/2; c/2];
    local_corners = [...
        half_sizes .* [ 1;  1;  1], ...
        half_sizes .* [ 1;  1; -1], ...
        half_sizes .* [ 1; -1;  1], ...
        half_sizes .* [ 1; -1; -1], ...
        half_sizes .* [-1;  1;  1], ...
        half_sizes .* [-1;  1; -1], ...
        half_sizes .* [-1; -1;  1], ...
        half_sizes .* [-1; -1; -1] ...
    ];


    R = quat2rotm(q_bloc);

    world_corners = r_boite + R * local_corners;

    faces = [
        1, 2, 4, 3; % +X face
        5, 6, 8, 7; % -X face
        1, 2, 6, 5; % +Y face
        3, 4, 8, 7; % -Y face
        1, 3, 7, 5; % +Z face
        2, 4, 8, 6  % -Z face
    ];

    %% collision faces du bloc
    for i = 1:size(faces, 1)
        face_indices = faces(i, :);
        face_vertices = world_corners(:, face_indices);

        % face normal
        v1 = face_vertices(:,2) - face_vertices(:,1);
        v2 = face_vertices(:,3) - face_vertices(:,1);
        normal = cross(v1, v2);
        normal = normal / norm(normal);

        % distance sphere centre face
        d = dot(normal, (r_sphere - face_vertices(:,1)));

        if abs(d) > rayon
            continue;
        end

        projection = r_sphere - d * normal;

        if pointInPolygon3D(projection, face_vertices)
            collision = true;
            p_intersec = projection;
            return;
        end
    end

    % collision aretes
    edges = [
        1,2; 2,4; 4,3; 3,1; % aretes +Y
        5,6; 6,8; 8,7; 7,5; % aretes -Y
        1,5; 2,6; 3,7; 4,8  % aretes connenctant
    ];

    for i = 1:size(edges, 1)
        edge_start = world_corners(:, edges(i,1));
        edge_end = world_corners(:, edges(i,2));

        % point le plus proche
        edge_vector = edge_end - edge_start;
        t = dot(r_sphere - edge_start, edge_vector) / dot(edge_vector, edge_vector);
        t = max(0, min(1, t));
        closest_point = edge_start + t * edge_vector;

        dist = norm(r_sphere - closest_point);

        if dist <= rayon
            collision = true;
            p_intersec = closest_point;
            return;
        end
    end

    % collisions coins
    for i = 1:size(world_corners, 2)
        corner = world_corners(:, i);
        dist = norm(r_sphere - corner);

        if dist <= rayon
            collision = true;
            p_intersec = corner;
            return;
        end
    end
end

function R = quat2rotm(q)
    q = q / norm(q);
    w = q(1);
    x = q(2);
    y = q(3);
    z = q(4);
    R = [1 - 2*(y^2 + z^2),     2*(x*y - z*w),     2*(x*z + y*w);
             2*(x*y + z*w), 1 - 2*(x^2 + z^2),     2*(y*z - x*w);
             2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x^2 + y^2)];
end

function inside = pointInPolygon3D(point, vertices)
    N = size(vertices, 2);
    inside = true;

    v1 = vertices(:,2) - vertices(:,1);
    v2 = vertices(:,3) - vertices(:,1);
    normal = cross(v1, v2);
    normal = normal / norm(normal);

    [~, idx] = max(abs(normal));
    idxs = setdiff(1:3, idx);

    projected_vertices = vertices(idxs, :);
    projected_point = point(idxs);

    % https://stackoverflow.com/questions/8452078/integer-winding-number-algorithm-with-edge-cases
    winding_number = 0;

    for i = 1:N
        j = mod(i, N) + 1;

        vi = projected_vertices(:, i);
        vj = projected_vertices(:, j);

        if vi(2) <= projected_point(2)
            if vj(2) > projected_point(2)
                if isLeft(vi, vj, projected_point)
                    winding_number = winding_number + 1;
                end
            end
        else
            if vj(2) <= projected_point(2)
                if ~isLeft(vi, vj, projected_point)
                    winding_number = winding_number - 1;
                end
            end
        end
    end

    if winding_number == 0
        inside = false;
    end
end

function left = isLeft(p0, p1, p2)
    left = ((p1(1) - p0(1)) * (p2(2) - p0(2)) - ...
            (p2(1) - p0(1)) * (p1(2) - p0(2))) > 0;
end


