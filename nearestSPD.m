function Ahat = nearestSPD(A)
    % nearestSPD - the nearest (in Frobenius norm) Symmetric Positive Definite matrix to A
    % usage: Ahat = nearestSPD(A)
    % from Higham: "The nearest symmetric positive semidefinite matrix in the
    % Frobenius norm to an arbitrary real matrix A is shown to be (B + H)/2,
    % where H is the symmetric polar factor of B=(A + A')/2."

    % ensure symmetry
    B = (A + A') / 2;

    % Compute the symmetric polar factor of B. Call it H.
    [U, S, V] = svd(B);
    H = V * S * V';

    % The nearest SPD matrix to A is (B + H)/2
    Ahat = (B + H) / 2;

    % Ensure symmetry
    Ahat = (Ahat + Ahat') / 2;

    % Test that Ahat is in fact PD. If it is not, then tweak it just a bit.
    p = 0;
    [~, p] = chol(Ahat);
    k = 0;
    while p ~= 0
        % Ahat failed the chol test. It must have been just a hair off,
        % due to floating point trash, so it is simplest now just to
        % tweak by adding a tiny multiple of the identity.
        mineig = min(eig(Ahat));
        Ahat = Ahat + (-mineig * k.^2 + eps(mineig)) * eye(size(A));
        [~, p] = chol(Ahat);
        k = k + 1;
        if k > 10
            error('nearestSPD: Cannot compute nearest SPD matrix.');
        end
    end
end
