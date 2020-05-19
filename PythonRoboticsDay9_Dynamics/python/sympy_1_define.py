import sympy

x = sympy.Symbol('x')
y = sympy.Symbol('y')

print(x, type(y))
#x <class 'sympy.core.symbol.Symbol'>

expr = x**2 + y +1

print(expr)
# x**2 + y + 1

print(expr.subs(x, 1))
# y + 2

print(expr.subs(x, y))
# y**2 + y + 1

print(expr.subs([(x, 1), (y, 2)]))
# 4
