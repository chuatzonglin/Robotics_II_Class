import sympy

x = sympy.Symbol('x')
y = sympy.Symbol('y')

print(sympy.solve(x**2 - 3 * x + 2))
# [1, 2]

print(sympy.solve(x**2 + x + 1))
# [-1/2 - sqrt(3)*I/2, -1/2 + sqrt(3)*I/2]

expr = x + y**2 - 4

print(sympy.solve(expr, x))
# [-y**2 + 4]

print(sympy.solve(expr, y))
# [-sqrt(-x + 4), sqrt(-x + 4)]

expr1 = 3 * x + 5 * y - 29
expr2 = x + y - 7

print(sympy.solve([expr1, expr2]))
# {x: 3, y: 4}
