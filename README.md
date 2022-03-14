# SOC_Paper
Code for simulations of our paper (in revision): "An approximate stochastic optimal control framework to simulate nonlinear neuro-musculoskeletal models in the presence of noise"


You will need to install CasADi (https://web.casadi.org/) to solve the defined optimal control problems.
 - 'mumps' is the standard linear solver used within IPOPT optionssol.ipopt.linear_solver = 'mumps'
 - in some cases the linear solver 'ma57' is more performant, but it is not open-source


In the folder with implentations for the model driven by four Hill-type muscles we provide two implementations.
 - The standard one: the covariance matrix is propagated in time using a shooting formulation
 - DC: the covariance matrix is propagated in time using a shooting formulation
