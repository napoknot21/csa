from pylab import *
from matplotlib import pyplot
import numpy as np
import scipy as sp
import scipy.linalg 


# Wrist position in Cartesian coordinates
# From a given theta computes the handle position
# cf equation (1)
# Inputs: h     = problem data (arm length)
#         theta = variables (joint angles)
# Output: f     = handle position (Cartesian coordinates)
def pos(h,theta):

    f=np.array([0.,0.,0.]);
    
    f[0] = cos(theta[0])*(h[1]*cos(theta[1])-h[2]*sin(theta[1]+theta[2]));
    f[1] = sin(theta[0])*(h[1]*cos(theta[1])-h[2]*sin(theta[1]+theta[2]));
    
    f[2] = h[2]*cos(theta[1]+theta[2]) + h[1]*sin(theta[1]);
    
    return(f)


# Jacobian of function fvec.
# Inputs: h     = problem data (arm length)
#         theta = variables (joint angles)
# Output: J     = Jacobian of the previous function
def jacobienne(h,theta): 

    J=np.zeros((3,3));

    J[0,0]  = -sin(theta[0])*(h[2]*cos(theta[1])-h[2]*sin(theta[1]+theta[2]));
    J[0,1] = cos(theta[0])*(-h[1]*sin(theta[1])-h[2]*cos(theta[1]+theta[2]));
    J[0,2] = -h[2]*cos(theta[0])*cos(theta[1]+theta[2]);

    J[1,0] = cos(theta[0])*(h[1]*cos(theta[1])-h[2]*sin(theta[1]+theta[2]));
    J[1,1] = sin(theta[0])*(-h[1]*sin(theta[1])-h[2]*cos(theta[1]+theta[2]));
    J[1,2] = sin(theta[0])*(-h[2]*cos(theta[1]+theta[2]));

    J[2,0] = 0;
    J[2,1] = -h[2]*sin(theta[1]+theta[2])+h[1]*cos(theta[1]);
    J[2,2] = -h[2]*sin(theta[1]+theta[2]);

    return (J)



# Function that one seeks to cancel.
# Define f(theta) (cf equation (2))
# Represents the difference between the wrist position and the target.
# Inputs: h        = problem data (arm length)
#         theta    = variables (joint angles)
#         xcible   = target position
# Output: f        = difference between calculated position and target position
def fvec(h,theta,xcible):
    # initialization of f
    f=np.array([0.,0.,0.]); 

    # compute f(theta)
    current_pos = pos(h, theta)

    f = np.subtract(current_pos, xcible)
    
    return(f)


# Take one step of Newton's iteration
# Entry:  theta        = current estimate (theta^(k))
#         h and xcible = problem data
# Output: thetanext    = estimated new (theta^(k+1))
def newton_step(h,xcible,theta):
    """
    # initialization
    y=np.array([0.,0.,0.]); 
    d_theta=np.array([0.,0.,0.]);

    # on commence par évaluer la jacobienne
    J=jacobienne(h,theta);
    f=fvec(h,theta ,xcible); # Composantes du second membre (f(x))
    
    # Compute d_theta using the LU deomposition 
    P, L, U = scipy.linalg.lu(J); # utilisation de la decomposition LU de python (be careful with P)
    # Python gives P.L.U=A
    # We have A.theta=-f, P.L.U=-f but we solve L.U=P^{T}.(-f)
    #   P transpose = P inverse
    
    # Résolution du système:
    # firtly we need to invert the line of f considering matrix P
    F2 = -np.dot(np.transpose(P),f) #F2=-P*f;
    
    # We need to inverse L.U.theta = F2 to find d_theta
    # find the solution of the two systems Lz = b and then Ux=z
    # Solve the system Lz = F2 for z
    z = scipy.linalg.solve(L, F2)

    # Now solve U.d_theta = z for d_theta
    d_theta = scipy.linalg.solve(U, z)
    
    # incrémentation of d_theta
    thetanext = theta + d_theta;        
    
    return(thetanext)
    """
    # Evaluate the Jacobian
    J = jacobienne(h, theta)
    f = fvec(h, theta, xcible)  # Components of the right-hand side (f(x))
    
    # Compute d_theta using LU decomposition
    P, L, U = scipy.linalg.lu(J)  # Use Python's LU decomposition (be careful with P)
    
    # We solve L.U.d_theta = -P.T*f
    F2 = -np.dot(np.transpose(P), f)  # F2 = -P*f
    
    # Solve the system Lz = F2 for z
    z = scipy.linalg.solve(L, F2)
    
    # Now solve U.d_theta = z for d_theta
    d_theta = scipy.linalg.solve(U, z)
    
    # Increment theta with d_theta
    thetanext = theta + d_theta
    
    return thetanext

    
# Problem data: h and xcible
# Output: theta = estimated angle
def newton(h,xcible,theta0):
    """
    # Set the tolerance level
    epsilon = 1e-12

    #initialization
    theta = np.copy(theta0);
       
    # include a criteria in the loop
    
    # Initialize the increment to a large value
    increment = np.inf
    # Counter for iterations
    iteration = 0
    
    # Newton's method loop
    while increment > epsilon:

        # Calculate the new estimate for theta
        thetanext = newton_step(h, xcible, theta)
        
        # Calculate the increment
        increment = scipy.linalg.norm(thetanext - theta)
        
        # Update theta
        theta = thetanext
        
        # Update the iteration counter
        iteration += 1
        
        # Optional: print iteration number and increment
        print(f"Iteration {iteration}: Increment = {increment}")
    
    # When the increment is sufficiently small, return theta
    return theta
    """
    # Set the tolerance level
    epsilon = 1e-12

    # Initialize theta with the initial guess
    theta = np.copy(theta0)
    
    # Initialize the increment to a large value
    increment = np.inf
    # Counter for iterations
    iteration = 0
    
    # Newton's method loop
    while increment > epsilon:
        # Calculate the new estimate for theta
        thetanext = newton_step(h, xcible, theta)
        
        # Calculate the increment as the norm of the difference between the new and old theta
        increment = scipy.linalg.norm(thetanext - theta)
        
        # Update theta with the new estimate
        theta = thetanext
        
        # Update the iteration counter
        iteration += 1
        
        # Optional: print iteration number and increment
        print(f"Iteration {iteration}: Increment = {increment}")
    
    # When the increment is sufficiently small, return the final estimate of theta
    return theta



# arms length
h=np.array([1.,1.,1.]);

# target
xcible =np.array([0.2,0.3,0.4]);# to modify 

# Initialization
theta0 =np.array([0.,0.,0.]);# to modify or not

# solving from theta0:
theta=newton(h,xcible ,theta0)

# We obtain the following solution : 
print('We obtain the following solution : theta = ',theta)