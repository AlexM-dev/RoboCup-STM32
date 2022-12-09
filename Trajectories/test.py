import math
r = 20
step = 1
a = 90
c = 0
S = 0.1 * 3

def f(x):
    return -(r - math.sqrt(-(x * x - 2 * x * r)))
def g(x):
    return -(-r + math.sqrt(-2 * x * r - x * x))

print("{")
while a >= 0:
    x = r * math.cos(a / 57.3)
    print("{", round(x, 3), ",", round(f(x), 3) , "}", "," "{", round(g(f(x)), 3), ",", round(f(g(f(x))), 3) , "}", ",")
    c+=1
    a-=(S * 180) / (math.pi * r)
    
print("}", c)