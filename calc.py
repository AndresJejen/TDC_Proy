import math 

ud = 1
y2 = 23.496
y1 = 19.175
t2 = 97.475
t1 = 96.875

Tu = 99.8 - 95.925

ypp = 25.630 - 17.829
e_h = 2
a = ypp/2

ku = 4*ud/(math.pi*math.sqrt(a**2 - e_h**2))
print("ku:", ku)
w_u = 2*math.pi/Tu
print("w_u:", w_u)

b = ypp / ((t2 - t1) * ud)
print("b:", b)
tau = math.sqrt( ( (ku*b)**2 - (w_u)**2 ) / (w_u)**2 )
print("tau:", tau)
L = ((math.pi/2) - math.atan(tau*w_u)) / w_u

print("*"*50)
print("ZIEGLER-NICHOLS")
kp = 0.6*ku
print("kp:", kp)
ki = kp/(0.5*Tu)
print("ki:", ki) 
kd = 0.125*Tu*kp
print("kd:", kd)

print("*"*50)
print("AMSTRONG")
kp = (0.37/(b*L)) + 0.02*(tau/(b*L**2))
print("kp:", kp)
ki = (0.03/(b*L)) + 0.0012*(tau/(b*L**3))
print("ki:", ki)
kd = (0.16/b) + 0.28*(tau/(b*L))
print("kd:", kd)



