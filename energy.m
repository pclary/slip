function E = energy(states, model_params)
% Utility function that calculates total energy of SLIP system

GPE = model_params.mass*model_params.gravity*states(2);
KE = 1/2*model_params.mass*(states(3).^2 + states(4).^2);
SPE = 1/2*model_params.stiffness*(states(7) - states(6)).^2;
E = GPE + KE + SPE;
