import numpy as np
import math

# Linear Velocity of Robot
d1 = 105
d5 = 150
a2 = 105
a3 = 100
theta1 = 1.50098
theta2 = 2.26893
theta3 = 2.18166
theta4 = 1.0472

Oo = np.array([[0], [0], [0]])
O1 = np.array([[0], [0], [d1]])
O2 = np.array(
    [
        [a2 * math.cos(theta1) * math.cos(theta2)],
        [a2 * math.sin(theta1) * math.cos(theta2)],
        [a2 * math.sin(theta2) + d1],
    ]
)
O3 = np.array(
    [
        [
            a3 * math.cos(theta1) * math.cos(theta2 + theta3)
            + a2 * math.cos(theta1) * math.cos(theta2)
        ],
        [
            a3 * math.sin(theta1) * math.cos(theta2 + theta3)
            + a2 * math.sin(theta1) * math.cos(theta2)
        ],
        [a3 * math.sin(theta2 + theta3) + a2 * math.sin(theta2) + d1],
    ]
)
O4 = np.array(
    [
        [
            a3 * math.cos(theta1) * math.cos(theta2 + theta3)
            + a2 * math.cos(theta1) * math.cos(theta2)
        ],
        [
            a3 * math.sin(theta1) * math.cos(theta2 + theta3)
            + a2 * math.sin(theta1) * math.cos(theta2)
        ],
        [a3 * math.sin(theta2 + theta3) + a2 * math.sin(theta2) + d1],
    ]
)

O5 = np.array(
    [
        [
            d5 * math.cos(theta1) * math.sin(theta2 + theta3 + theta4)
            + a3 * math.cos(theta1) * math.cos(theta2 + theta3)
            + a2 * math.cos(theta1) * math.cos(theta2)
        ],
        [
            d5 * math.sin(theta1) * math.sin(theta2 + theta3 + theta4)
            + a3 * math.sin(theta1) * math.cos(theta2 + theta3)
            + a2 * math.sin(theta1) * math.cos(theta2)
        ],
        [
            -d5 * math.cos(theta2 + theta3 + theta4)
            + a3 * math.sin(theta2 + theta3)
            + a2 * math.sin(theta2)
            + d1
        ],
    ]
)

# Angular Velocity of the Robots

z0 = np.array([[0], [0], [1]])
z1 = np.array([[math.sin(theta1)], [-math.cos(theta1)], [0]])
z3 = z2 = z1
z4 = np.array(
    [
        [math.cos(theta1) * math.sin(theta2 + theta3 + theta4)],
        [math.sin(theta1) * math.sin(theta2 + theta3 + theta4)],
        [-math.cos(theta2 + theta3 + theta4)],
    ]
)

J11 = np.multiply(z0, (O5 - Oo))
J12 = np.multiply(z1, (O5 - O1))
J13 = np.multiply(z2, (O5 - O2))
J14 = np.multiply(z3, (O5 - O3))
J15 = np.multiply(z4, (O5 - O4))
Jacobian_Linear = np.hstack([J11, J12, J13, J14, J15])

Jm1 = [
    0,
    math.sin(theta1),
    math.sin(theta1),
    math.sin(theta1),
    math.cos(theta1) * math.sin(theta2 + theta3 + theta4),
]

Jm2 = [
    0,
    -math.cos(theta1),
    -math.cos(theta1),
    -math.cos(theta1),
    math.sin(theta1) * math.sin(theta2 + theta3 + theta4),
]

Jm3 = [1, 0, 0, 0, -math.cos(theta2 + theta3 + theta4)]

Jacobian_Angular = np.zeros((3, 5))
Jacobian_Angular[0] = Jm1
Jacobian_Angular[1] = Jm2
Jacobian_Angular[2] = Jm3


Jacobian_Matrix = np.vstack([Jacobian_Linear, Jacobian_Angular])
psi = np.array([[theta1], [theta2], [theta3], [theta4]])


Jacob_Inverse = np.linalg.pinv(Jacobian_Matrix)
