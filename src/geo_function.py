"""
This script implement usefull math function
"""

import numpy as np
import pandas as pd

def norm(x):
    return np.sqrt(x.dot(x.T))

def scalar_product(x,y):
    return x.dot(y.T)

def scalar_product_pandas(serie1,serie2):
    X = np.array(list(serie1.values))
    Y = np.array(list(serie2.values))
    X = pd.DataFrame(X, columns = ['x','y','z'])
    Y = pd.DataFrame(Y, columns = ['x','y','z'])
    
    return (X['x']*Y['x'] + X['y']*Y['y'] + X['z']*Y['z'])
def scalar_product_vect_pandas(vect,serie):
    X = np.array(list(serie.values))
    X = pd.DataFrame(X, columns = ['x','y','z'])
    return (X['x']*vect[0] + X['y']*vect[1]  + X['z']*vect[2])
    
def norm_pandas(serie):
    return scalar_product_pandas(serie,serie).apply(np.sqrt)