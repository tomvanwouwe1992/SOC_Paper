# SOC_Paper
Code for simulations of our paper: "An approximate stochastic optimal control framework to simulate nonlinear neuro-musculoskeletal models in the presence of noise"

https://journals.plos.org/ploscompbiol/article?id=10.1371/journal.pcbi.1009338


You will need to install CasADi (https://web.casadi.org/) to solve the defined optimal control problems.
 - 'mumps' is the standard linear solver used within IPOPT optionssol.ipopt.linear_solver = 'mumps'
 - in some cases the linear solver 'ma57' is more performant, but it is not open-source


For the muscle driven reaching simulations we provide two implementations.
 - The standard one: the covariance matrix is propagated in time using a shooting formulation
 - DCversion: the covariance matrix is propagated in time using a direct collocation formulation
