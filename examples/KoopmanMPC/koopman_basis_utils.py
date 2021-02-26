import numpy as np
import sympy as sp
from sympy.polys.monomials import itermonomials
from sympy.polys.orderings import monomial_key
from scipy.special import factorial


def GetMonomials(vars, deg):
    mon = sorted(itermonomials(vars,deg,0), key=monomial_key('grlex', vars))
    mon[0], mon[-1] = mon[-1], mon[0]
    return mon


def CreateObservableBasisPolynomial(self, vars, deg):
    nx = vars.shape[0]
    N = factorial(nx + deg) / (factorial(nx) * factorial(deg))

    basis = GetMonomials(vars, deg)