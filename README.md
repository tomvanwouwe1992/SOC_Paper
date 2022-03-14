# SOC_Paper
Code for simulations of our paper (in revision): "An approximate stochastic optimal control framework to simulate nonlinear neuro-musculoskeletal models in the presence of noise"

https://www.biorxiv.org/content/10.1101/2021.08.13.456221v1


You will need to install CasADi (https://web.casadi.org/) to solve the defined optimal control problems.
 - 'mumps' is the standard linear solver used within IPOPT optionssol.ipopt.linear_solver = 'mumps'
 - in some cases the linear solver 'ma57' is more performant, but it is not open-source


For the muscle driven reaching simulations we provide two implementations.
 - The standard one: the covariance matrix is propagated in time using a shooting formulation
 - DCversion: the covariance matrix is propagated in time using a direct collocation formulation
