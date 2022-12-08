import math
r = 20
step = 1
c = 0

def f(x):
    return r - math.sqrt(-(x * x - 2 * x * r))

# print("{")
# while y > r:
#     print("{", 0, ",", round(y, 3), "}", ",")
#     y -= step
y = r
print("{")
while y > r - r * math.sqrt(2) / 2:
    print("{", round(f(y), 3), ",", -round(y, 3), "}", ",")
    c+=1
    y -= step / 10
    
while round(y, 3) > 0:
    print("{", round(f(y), 3), ",", -round(y, 3), "}", ",")
    c+=1
    y = 0.005   * (200*r   - math.sqrt(40000*y*y     - 80000*r*y     + 40000*r*r     + 8000   *math.sqrt(2*y*r - y*y) - 400)) #step = 0.1
    #y = 0.00005 * (20000*r - math.sqrt(400000000*y*y - 800000000*r*y + 400000000*r*r + 8000000*math.sqrt(2*y*r - y*y) - 40000)) #step = 0.01
    #sqrt(-(yold^2-2*yold*r)) + sqrt(-(y^2-2*y*r)) = step / 10(or /100)
print("{", round(r, 3), ",", round(0, 3), "}", ",")
c+=1
print("}", c)