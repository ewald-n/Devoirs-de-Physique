function qc = QConjugue(q)
%
% Calcul du conjugué du quaternion q
%
% Entrée :
%   q - Quaternion sous la forme d'un vecteur colonne [q0; q1; q2; q3]
%
% Sortie :
%   qc - Conjugué du quaternion q

qc = [q(1); -q(2); -q(3); -q(4)];
end

