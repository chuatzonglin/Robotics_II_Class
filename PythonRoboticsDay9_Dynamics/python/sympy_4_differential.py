import sympy

x = sympy.Symbol('x')
y = sympy.Symbol('y')

theta_1 = sympy.Symbol("theta_1")
theta_2 = sympy.Symbol("theta_2")

theta_1_d = sympy.Symbol("theta_1_d")
theta_2_d = sympy.Symbol("theta_2_d")




print(sympy.diff(x**3 + 2 * x**2 + x))
# 3*x**2 + 4*x + 1

expr = x**3 + y**2 - y

print(sympy.diff(expr, x))
# 3*x**2

print(sympy.diff(expr, y))
# 2*y - 1

print(sympy.integrate(3 * x**2 + 4 * x + 1))
# x**3 + 2*x**2 + x

print(sympy.diff(sympy.cos(x)))
# -sin(x)

print(sympy.diff(sympy.exp(x)))
# exp(x)

print(sympy.diff(sympy.log(x)))
# 1/x

print(sympy.integrate(sympy.cos(x)))
# sin(x)

print(sympy.integrate(sympy.exp(x)))
# exp(x)

print(sympy.integrate(sympy.log(x)))
# x*log(x) - x

print(sympy.integrate(expr,x))

