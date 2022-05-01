

    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    # Trajactory Adaptation
    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    # Adaptive impedance Control
    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    # Constant system paramters
    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    # Constant system paramters
    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%

generationTime = 3000 # in milliseconds

K2Max = 1000
dTMax = 5000
M2Max = 1000
Kc0Max = 1000
Mu2Max = 1000
v2Max = 1000
KdMax = 1000

dTFull = 5000
K2Full = 0.00000001
M2Full = 1
Kc0Full = 0.00000001
KdFull = 0.00000001

Mu2Full = 0.001
v2Full = 0.001

imp = {
    "dT": 50, # sample time in milliseconds
    
    "M1": 1, # paramter to adjust Alpha1
    "K1": 1, # parameter to regulate the position error e
    "Mu1": 1, # parameter to regulate the forgetting rate
    "v1": 1, # parameter to adjust the adaptation of the interaction force Fh with Mu1

    "K2": 1, # paramter to adjust the weight of the tracking error e
    "M2": 1, # parameter to adjust Alpha2
    "Mu2": 1, # paramter to regulate the forgetting rate
    "v2": 1, # paramter to adjust the adaptation of the interaction force

    "Kd": 1, # parameter of damping can be variable too if we want
    "Kc0": 1 # paramter of nominal stiffness, must be designed according to the experimental results
}

# global variables
WIDTH = 500
HEIGHT = 600