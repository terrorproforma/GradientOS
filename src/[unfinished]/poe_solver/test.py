import numpy as np, ik_solver as ik

home = np.zeros(6)
g_home = ik.fk(home)
sols   = ik.ik(g_home)      # should return the eight nominal zeros
print(len(sols), sols[0].th)

# Jacobian at arbitrary posture
J = ik.jacobian([.1,-.5,.3,.7,-.2,1.0])
print(J)
