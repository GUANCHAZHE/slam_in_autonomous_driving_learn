# Test
import numpy as np

a = [1 , 1, 1]
x = a[0]
y = a[1]
z = a[2]

print("x{x}, y{y}, z{z}")
rang = x * x + y * y

a = np.arctan2(y, x)
e = np.arctan2(z, rang)

print("range {rang}, a{a}, e{e}")