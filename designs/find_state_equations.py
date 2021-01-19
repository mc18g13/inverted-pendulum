from sympy.physics.mechanics import *
from sympy import Matrix
from sympy import init_printing
from sympy import *

def add_space():
  print()
  print()
  print()
  print()
  print()

init_printing(use_unicode=True)

# q1 angular arm displacement
# q2 angular pendulum displacement

q1, q2 = dynamicsymbols('q1 q2')
q1d, q2d = dynamicsymbols('q1 q2', 1)
q1d2, q2d2 = dynamicsymbols('q1 q2', 2)
l1, l2, m1, m2, g, K, R, V = symbols('l1, l2, m1, m2, g, K, R, V')


half = (1/2)

kinetic_energy_for_arm = half * 1/3 * m1 * (l1 ** 2) * (q1d ** 2)
kinetic_energy_for_pendulum = (
  half * (1/12) * m2 * (l2 ** 2) * (q2d ** 2) + 
  half * m2 * (
    (
      q1d * l1 - 
      half * l2 * q2d * cos(q2)
    ) ** 2 +
    (
      -half * l2 * q2d * sin(q2) ** 2
    )
  )
)

potential_energy_for_system = m2 * g * half * l2 * cos(q2)

L = kinetic_energy_for_arm + kinetic_energy_for_pendulum - potential_energy_for_system

L1 = simplify(diff(diff(L, q1d), 't') - diff(L, q1))
# pprint(L1)
# add_space()

L2 = simplify(diff(diff(L, q2d), 't') - diff(L, q2))
# pprint(L2)

motor_torque = K * V / R - (K ** 2) *  q1d / R

lagrange_solution = simplify(solve([L1 -  motor_torque, L2], (q1d2, q2d2)))


""" 
convert to first order ode's

x = [x1, x2, x3, x4] = [q1, q2, q1d, q2d]
xd = [x1d, x2d, x3d, x4d] = [q1d, q2d, q1d2, q2d2] = [f1, f2, f3, f4]

"""

f1 = q1d
f2 = q2d
f3 = lagrange_solution[q1d2]
f4 = lagrange_solution[q2d2]

A11 = diff(f1, q1)
A12 = diff(f1, q2)
A13 = diff(f1, q1d)
A14 = diff(f1, q2d)

A21 = diff(f2, q1)
A22 = diff(f2, q2)
A23 = diff(f2, q1d)
A24 = diff(f2, q2d)

A31 = diff(f3, q1)
A32 = diff(f3, q2)
A33 = diff(f3, q1d)
A34 = diff(f3, q2d)

add_space()
add_space()
add_space()
add_space()
add_space()

A41 = diff(f4, q1)
A42 = diff(f4, q2)
A43 = diff(f4, q1d)
A44 = diff(f4, q2d)

A = Matrix ([
  [A11, A12, A13, A14],
  [A21, A22, A23, A24],
  [A31, A32, A33, A34],
  [A41, A42, A43, A44]
])

A = simplify(A.subs({q1: 0, q2: 0, q1d: 0, q2d: 0, V: 0}))
print(A)
add_space()

B1 = diff(f1, V)
B2 = diff(f2, V)
B3 = diff(f3, V)
B4 = diff(f4, V)

B = Matrix ([
  B1, B2, B3, B4
])


B = simplify(B.subs({q1: 0, q2: 0, q1d: 0, q2d: 0, V: 0}))


add_space()
print(B)