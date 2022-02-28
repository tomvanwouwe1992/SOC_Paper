

import casadi.*
nrStochSims = 100;
listing = dir();

for j = 1:length(listing)
    j/length(listing)
    if contains(listing(j).name,'_noVelCtr.mat')
        name = listing(j).name;
        load(name);
        if ~isfield(result,'X_stoch_unperturbed')
            functions = generateFunctions_OCP_6muscles_FF_FB(result.auxdata);
            
            EE_stoch_unperturbed = NaN(4,result.auxdata.N+1,nrStochSims);
            X_stoch_unperturbed = NaN(result.auxdata.nStates,result.auxdata.N+1,nrStochSims);
            
            EE_stoch_perturbed = NaN(4,result.auxdata.N+1,nrStochSims);
            X_stoch_perturbed = NaN(result.auxdata.nStates,result.auxdata.N+1,nrStochSims);
                        P_init = result.Pmat(:,:,1);

            perturbation = [0;0];
            parfor i = 1:nrStochSims
                [X_stoch_local,EE_stoch_local] = stochasticForwardSim(result.X(:,1),result.e_ff',result.EE_ref,result.K,result.auxdata,functions,perturbation,10,P_init);
                EE_stoch_unperturbed(:,:,i) = EE_stoch_local;
                X_stoch_unperturbed(:,:,i) = X_stoch_local;
            end
            
            perturbation = -[2;0.5];
            for i = 1:nrStochSims
                [X_stoch_local,EE_stoch_local] = stochasticForwardSim(result.X(:,1),result.e_ff',result.EE_ref,result.K,result.auxdata,functions,perturbation,10,P_init);
                EE_stoch_perturbed(:,:,i) = EE_stoch_local;
                X_stoch_perturbed(:,:,i) = X_stoch_local;
            end
            

            
            result.EE_stoch_unperturbed = EE_stoch_unperturbed;
            result.X_stoch_unperturbed = X_stoch_unperturbed;
            result.EE_stoch_perturbed = EE_stoch_perturbed;
            result.X_stoch_perturbed = X_stoch_perturbed;
            save(name,'result')
        end
    end
end
