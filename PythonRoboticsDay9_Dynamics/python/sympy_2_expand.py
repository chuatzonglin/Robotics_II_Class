import sympy

x = sympy.Symbol('x')
y = sympy.Symbol('y')

expr = (x + 1)**2

print(expr)
# (x + 1)**2

expr_ex = sympy.expand(expr)

print(expr_ex)
# x**2 + 2*x + 1

expr_factor = sympy.factor(expr_ex)

print(expr_factor)
# (x + 1)**2

print(sympy.factor(x**3 - x**2 - 3 * x + 3))
# (x - 1)*(x**2 - 3)

print(sympy.factor(x * y + x + y + 1))
# (x + 1)*(y + 1)
