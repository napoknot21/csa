from pylab import *
from matplotlib import pyplot
import numpy as np
import scipy as sp
import scipy.linalg


# Initialization of the variable n, k and b
# n := number of circuits
n=10

# k := Resistor values (vector: (n,1))
k=10.0*np.ones((n,1))

# u := Voltage values (vector: (n,1))
u=1.0*np.ones((n,1))

# b := Calculation of the vector b (values of the tensions)
b=1.0*np.zeros((n,1))

# e := value for the stopping criterion (to be specified)
epsilon = 1.0e-8;





def matrice_circuit (k) :
    n = size(k)

    A = np.zeros((n,n))

    # On remplit la première lighen
    A[0][0] = k[0]
    A[0][1] = -k[0]


    #for in in range(1, n-1) :

    #On remplit la dernière ligne
    A[n-1][n-2] = -k[n-1]
    A[n-1][n-1] = -k[n-1]
    """
    for i in range (1, n-1) :
        A[i][] =
    """
    return A


print(matrice_circuit(k))



def gauss_seidel_iter (A, b, x) :

    n=size(b)

    for i in range (n) :
        """
        for j in ranger (i+1, n) : # j=1...n-1 (case j>1)
            sum1 = A[i][j] * k[j]
        """
        sum_jsupi = np.sum( A[i, i+1:] * x[i+1:] )
        sum_jinfi = np.sum ( A[i, :i] * x[:i])

        x[i] = (b[i] - sum_jsupi - sum_jinfi) / A[i][i]

    return x # x_k+1



def jacobi_iter (A, b, x) :

    x1 = np.copy(x)
    n = size(b)



def gauss_seidel (A, b, epsilon) :

    incr = 10
    x = np.array(b, copy=True)
    print('x=', x)
    
    while incr > epsilon :

        x_old = np.array(x, copy=True)

        print('x_old=', x_old)
        x=gauss_seidel_iter (A,b,x)

        incr = np.linalg.norm(np.substract(x, x_old))
        print('incr=', incr)

    return (x)