from pydrake.solvers import MathematicalProgram, Solve
import numpy as np

# Plan in joint space for the arm, cartesian for the base


# Set up initial guess


# Declare decision variables

prog = MathematicalProgram()
p1 = prog.NewContinuousVariables(9, "x")

def cost_fun(z, start_pos, end_pos):
    return cos_z**2 + cos_z + sin_z
# Add the cost evaluated with x[0] and x[1].
cost1 = prog.AddCost(cost_fun, vars=[x[0], x[1]])

# Add the constraint that p1 is on the unit circle centered at (0, 2)
prog.AddConstraint(
    lambda z: [z[0]**2 + (z[1]-2)**2],
    lb=np.array([1.]),
    ub=np.array([1.]),
    vars=p1)

# Add the constraint that p2 is on the curve y=x*x
prog.AddConstraint(
    lambda z: [z[1] - z[0]**2],
    lb=[0.],
    ub=[0.],
    vars=p2)

# Set the value of p1 in initial guess to be [0, 1]
prog.SetInitialGuess(p1, [0., 1.])
# Set the value of p2 in initial guess to be [1, 1]
prog.SetInitialGuess(p2, [1., 1.])