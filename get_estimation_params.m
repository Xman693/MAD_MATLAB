function EstimationParams = get_estimation_params()
A = zeros(4,4);
A(1,3) = 1; A(2,4) = 1;
Q = 1e-3 * eye(4);
EstimationParams.ABT.A = A;
EstimationParams.ABT.Q = Q;
end
