
import control
import math

KA = 0.887
KT = 0.72
JE = 7e-4
BE = 0.00612
KE = 20 / (2 * math.pi)
T_S = 0.0002

num = [KA * KT * KE]
den = [JE, BE, 0.0]
Gs = control.TransferFunction(num, den)
Gz = control.c2d(Gs, T_S, method='zoh')

print("Gz coefficients:")
print(Gz)
