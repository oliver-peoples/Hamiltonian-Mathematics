import math
import copy
import numpy as np


class Complex:

    # No initialization is needed as this is merely a class of functions, as Python already has a fairly robust
    # complex number library that was well worth using, as its basis is in one of the c languages

    # Returns the norm (magnitude) of the complex number - tested in complex_norms_test.py
    @staticmethod
    def norm(c=complex):
        return math.sqrt((c.real ** 2 + c.imag ** 2))

    # Returns the normed (unit) version of the compelx number - tested in complex_norms_test.py
    @staticmethod
    def normed(c=complex):
        norm = Complex.norm(c)
        return complex(c.real / norm, c.imag / norm)

    # Returns the same complex number but with the imaginary component negated - tested in complex_conjugate_test.py
    @staticmethod
    def conjugate(c=complex):
        return complex(c.real, c.imag * -1)


# Two Dimensional Vectors - the mentioned R2 coordinate space.

class Vector2:

    def __init__(self, a, b):
        self.a = a  # value along x axis
        self.b = b  # value alogn y axis

    # Multiplies the vector by a scalar - evaluated in vector2_scale_test.py
    def scale(self, scalar):
        return Vector2(self.a * scalar, self.b * scalar)

    # Returns the norm (magnitude) of the vector - tested in vector2_norms_test.py
    @property
    def norm(self):
        return math.sqrt((self.a ** 2 + self.b ** 2))

    # Returns the normed (unit) version of the vector - tested in vector2_norms_test.py
    @property
    def normed(self):
        norm = self.norm
        return Vector2(self.a / norm, self.b / norm)

    # Prints the vector in <i, j> form
    @property
    def string(self):
        return f'<{self.a}, {self.b}>'

    # Returns the sum of n vectors - tested in vector2_sum_difference_test.py
    @classmethod
    def sum(cls, *args):
        v = copy.copy(args[0])

        for j in args[1:]:
            a = v.a + j.a
            b = v.b + j.b
            v.a, v.b = a, b  # Not too sure why this final line is needed; answer was incorrect otherwise
        return v

    # Returns the difference of n vectors - tested in vector2_sum_difference_test.py
    @classmethod
    def difference(cls, *args):
        v = copy.copy(args[0])

        for j in args[1:]:
            a = v.a - j.a
            b = v.b - j.b
            v.a, v.b = a, b  # Not too sure why this final line is needed; answer was incorrect otherwise
        return v

    # Returns the dot product of two vectors - tested in vector2_dot_test.py
    @classmethod
    def dot_product(cls, v1, v2):
        return v1.a * v2.a + v1.b * v2.b

    # This function determines the angle between two vectors using the dot product of the normed inputs and acos. Output
    # is in radians. - tested in vector2_angle_test.py
    @classmethod
    def angle(cls, v1, v2):
        v1 = v1.normed
        v2 = v2.normed
        dot = cls.dot_product(v1, v2)
        return math.acos(dot)


# Three Dimensional Vectors - the mentioned R3 coordinate space.

class Vector3:

    # Initialization

    def __init__(self, a, b, c):
        self.a = a  # value along x axis
        self.b = b  # value along y axis
        self.c = c  # value along z axis

    # Multiplies the vector by a scalar - evaluated in vector3_scale_test.py
    def scale(self, scalar):
        return Vector3(self.a * scalar, self.b * scalar, self.c * scalar)

    # Properties

    # Returns the norm (magnitude) of the vector - tested in vector3_norms_test.py
    @property
    def norm(self):
        return math.sqrt((self.a ** 2 + self.b ** 2 + self.c ** 2))

    # Returns the normed (unit) version of the vector - tested in vector3_norms_test.py
    @property
    def normed(self):
        return self.scale(scalar=(1 / self.norm))

    # Prints the vector in <i, j, k> form
    @property
    def string(self):
        return f'<{self.a}, {self.b}, {self.c}>'

    # Class Methods

    # Returns the sum of n vectors - tested in vector3_sum_difference_test.py
    @classmethod
    def sum(cls, *args):
        v = copy.copy(args[0])

        for j in args[1:]:
            a = v.a + j.a
            b = v.b + j.b
            c = v.c + j.c
            v.a, v.b, v.c = a, b, c  # Not too sure why this final line is needed; answer was incorrect otherwise
        return v

    # Returns the difference of n vectors - tested in vector3_sum_difference_test.py
    @classmethod
    def difference(cls, *args):
        v = copy.copy(args[0])

        for j in args[1:]:
            a = v.a - j.a
            b = v.b - j.b
            c = v.c - j.c
            v.a, v.b, v.c = a, b, c  # Not too sure why this final line is needed; answer was incorrect otherwise
        return v

    # Returns the vector orthogonal to both input vectors - tested in vector3_cross_test.py
    @classmethod
    def cross_product(cls, v1, v2):
        v3a = v1.b * v2.c - v1.c * v2.b
        v3b = v1.c * v2.a - v1.a * v2.c
        v3c = v1.a * v2.b - v1.b * v2.a
        return Vector3(v3a, v3b, v3c)

    # Returns the dot product of two vectors - tested in vector3_dot_test.py
    @classmethod
    def dot_product(cls, v1, v2):
        return v1.a * v2.a + v1.b * v2.b + v1.c * v2.c

    # This function determines the angle between two vectors using the dot product of the normed inputs and acos. Output
    # is in radians - evaluated in vector3_angle_test.py
    @classmethod
    def angle(cls, v1, v2):
        v1 = v1.normed
        v2 = v2.normed
        dot = cls.dot_product(v1, v2)
        return math.acos(dot)


# Quaternions

class Quaternion:

    # Initialization

    def __init__(self, a, b, c, d):
        self.a = a  # the scalar component
        self.b = b  # quaternionic i component
        self.c = c  # quaternionic j component
        self.d = d  # quaternionic k component

    # Returns a scaled version of the quaternion, where every element has been multiplied by some scalar variable
    # - tested in quaternion_scale_test.py
    def scale(self, scalar):
        return Quaternion(self.a * scalar, self.b * scalar, self.c * scalar, self.d * scalar)

    # Properties

    # Return the quaternion equivalent of a complex conjugate - tested in quaternion_pure_and_rotations_test.py
    @property
    def conjugate(self):
        return Quaternion(self.a, -1 * self.b, -1 * self.c, -1 * self.d)

    # Return the magnitude of a quaternion
    @property
    def norm(self) -> float:
        return math.sqrt((self.a ** 2 + self.b ** 2 + self.c ** 2 + self.d ** 2))

    # Return the a quaternion with magnitude 1, thus an element of SU(2) and member of SO(3) - tested in quaternion_pure_and_rotations_test.py
    @property
    def normed(self):
        return self.scale(scalar=(1 / self.norm))

    # Class Methods

    # Sums n quaternions
    @classmethod
    def sum(cls, *args):
        q = copy.copy(args[0])

        for j in args[1:]:
            a = q.a + j.a
            b = q.b + j.b
            c = q.c + j.c
            d = q.d + j.d
            q.a, q.b, q.c, q.d = a, b, c, d  # Not too sure why this final line is needed; answer was incorrect otherwise
        return q

    # Returns the difference of n quaternions
    @classmethod
    def difference(cls, *args):
        q = copy.copy(args[0])

        for j in args[1:]:
            a = q.a - j.a
            b = q.b - j.b
            c = q.c - j.c
            d = q.d - j.d
            q.a, q.b, q.c, q.d = a, b, c, d  # Not too sure why this final line is needed; answer was incorrect otherwise
        return q

    # Returns the quaternionic product of n quaternions - tested in quaternion_pure_and_rotations_test.py
    @classmethod
    def product(cls, *args):
        q = copy.copy(args[0])

        for j in args[1:]:
            a = (q.a * j.a) - (q.b * j.b) - (q.c * j.c) - (q.d * j.d)
            b = (q.a * j.b) + (q.b * j.a) + (q.c * j.d) - (q.d * j.c)
            c = (q.a * j.c) - (q.b * j.d) + (q.c * j.a) + (q.d * j.b)
            d = (q.a * j.d) + (q.b * j.c) - (q.c * j.b) + (q.d * j.a)
            q.a, q.b, q.c, q.d = a, b, c, d  # Not too sure why this final line is needed; answer was incorrect otherwise
        return q

    # Return a quaternion form of a vector in R3 - tested in quaternion_pure_and_rotations_test.py
    @classmethod
    def pure_qt(cls, vector=Vector3):
        return Quaternion(0, vector.a, vector.b, vector.c)

    # Rotates a point using a quaternion - tested in quaternion_pure_and_rotations_test.py
    @classmethod
    def rotate(cls, q, p=Vector3):
        if q.norm < 1.00000001 and q.norm > 0.99999999:  # manual control over float error
            qd = q.conjugate
            p = cls.pure_qt(p)
            qpqd = cls.product(q, p, qd)
            return Vector3(qpqd.b, qpqd.c, qpqd.d)
        else:
            raise ArithmeticError

    # Generates a rotation about an axis; based off the syntax from point_rotation, which was tested in quaternion_pure_and_rotations_test.py
    @classmethod
    def axis_rotation(cls, angle=float, axis=Vector3):
        angle *= 0.5
        axis = axis.normed
        axis = axis.scale(math.sin(angle))
        q = Quaternion(math.cos(angle), axis.a, axis.b, axis.c)
        return q

    # Generates the rotation quaternion that will rotate point p1 to point p2
    @classmethod
    def point_rotation(cls, p1=Vector3, p2=Vector3):
        angle = Vector3.angle(p1, p2) * 0.5
        axis = Vector3.cross_product(p1, p2).normed
        axis = axis.scale(math.sin(angle))
        q = Quaternion(math.cos(angle), axis.a, axis.b, axis.c)
        return q


# Complex Quaternions

class QtComplex:

    # Initialization

    def __init__(self, c1, c2, c3, c4):
        self.c1 = c1  # first complex element
        self.c2 = c2  # second complex element
        self.c3 = c3  # third complex element
        self.c4 = c4  # fourth complex element

    # Returns the complex linear combination (CLC) notation for the complex quaternion
    @property
    def CLC(self):
        string = f'{self.c1, self.c2, self.c3, self.c4}'
        return string

    # Returns the real linear combination (RLC) notation for the complex quaternion
    @property
    def RLC(self):
        string = f'{self.c1.real, self.c1.imag, self.c2.real, self.c3.real, self.c4.real, self.c2.imag, self.c3.imag, self.c4.imag}'
        return string

    # Returns a complex quaternion whose quaternionic elements are negated; in CLC, these are the last three elements,
    # in RLC it is the last six
    @property
    def quaternion_conjugate(self):
        return QtComplex(self.c1, self.c2 * -1, self.c3 * -1, self.c4 * -1)

    # Returns a complex quaternion whose complex i values are negated; this is the imaginary component of each element
    # in CLC notation, and the 2nd - 5th element in RLC notation
    @property
    def complex_conjugate(self):
        c1 = Complex.conjugate(self.c1)
        c2 = Complex.conjugate(self.c2)
        c3 = Complex.conjugate(self.c3)
        c4 = Complex.conjugate(self.c4)
        return QtComplex(c1, c2, c3, c4)

    # Returns the both the complex and quaternionic conjugate in one complex quaternion
    @property
    def hermitian_conjugate(self):
        c1 = Complex.conjugate(self.c1)
        c2 = Complex.conjugate(self.c2) * -1
        c3 = Complex.conjugate(self.c3) * -1
        c4 = Complex.conjugate(self.c4) * -1
        return QtComplex(c1, c2, c3, c4)

    # Class Methods

    # Allows definition of a complex quaternion through a real linear combination
    @classmethod
    def real_linear(cls, a, b, i, j, k, ii, ij, ik):
        c1 = complex(a, b)
        c2 = complex(i, ii)
        c3 = complex(j, ij)
        c4 = complex(k, ik)
        return QtComplex(c1, c2, c3, c4)

    # The complex quaternion multiplicative product
    @classmethod
    def product(cls, *args):
        cq = copy.copy(args[0])

        for j in args[1:]:
            c1 = (cq.c1 * j.c1) - (cq.c2 * j.c2) - (cq.c3 * j.c3) - (cq.c4 * j.c4)
            c2 = (cq.c1 * j.c2) + (cq.c2 * j.c1) + (cq.c3 * j.c4) - (cq.c4 * j.c3)
            c3 = (cq.c1 * j.c3) - (cq.c2 * j.c4) + (cq.c3 * j.c1) + (cq.c4 * j.c2)
            c4 = (cq.c1 * j.c4) + (cq.c2 * j.c3) - (cq.c3 * j.c2) + (cq.c4 * j.c1)
            cq.c1, cq.c2, cq.c3, cq.c4 = c1, c2, c3, c4

        return cq


class DualQuaternion:
    def __init__(self, a, b, c, d, e, ei, ej, ek):
        self.a = a
        self.b = b
        self.c = c
        self.d = d
        self.e = e
        self.ei = ei
        self.ej = ej
        self.ek = ek

    @property
    def ELC(self):
        string = f'{self.a, self.b, self.c, self.d, self.e, self.ei, self.ej, self.ek}'
        return string

    @property
    def QTA(self):
        string = f'{self.a, self.b, self.c, self.d}'
        return

    @property
    def QTB(self):
        string = f'{self.e, self.ei, self.ej, self.ek}'
        return

    @property
    def dualQuaternionConjugate(self):
        a = self.a
        b = -1 * self.b
        c = -1 * self.c
        d = -1 * self.d
        e = self.e
        ei = -1 * self.ei
        ej = -1 * self.ej
        ek = -1 * self.ek
        return DualQuaternion(a, b, c, d, e, ei, ej, ek)

    @property
    def dualNumberConjugate(self):
        a = self.a
        b = self.b
        c = self.c
        d = self.d
        e = -1 * self.e
        ei = -1 * self.ei
        ej = -1 * self.ej
        ek = -1 * self.ek
        return DualQuaternion(a, b, c, d, e, ei, ej, ek)

    @property
    def fullConjugate(self):
        a = self.a
        b = -1 * self.b
        c = -1 * self.c
        d = -1 * self.d
        e = -1 * self.e
        ei = self.ei
        ej = self.ej
        ek = self.ek
        return DualQuaternion(a, b, c, d, e, ei, ej, ek)

    # Sums n dual quaternions
    @classmethod
    def sum(cls, *args):
        q = copy.copy(args[0])

        for j in args[1:]:
            a = q.a + j.a
            b = q.b + j.b
            c = q.c + j.c
            d = q.d + j.d
            e = q.e + j.e
            ei = q.ei + j.ei
            ej = q.ej + j.ej
            ek = q.ek + j.ek
            q.a, q.b, q.c, q.d, q.e, q.ei, q.ej, q.ek = a, b, c, d, e, ei, ej, ek
        return q

    # Returns the difference of n dual quaternions
    @classmethod
    def difference(cls, *args):
        q = copy.copy(args[0])

        for j in args[1:]:
            a = q.a - j.a
            b = q.b - j.b
            c = q.c - j.c
            d = q.d - j.d
            e = q.e - j.e
            ei = q.ei - j.ei
            ej = q.ej - j.ej
            ek = q.ek - j.ek
            q.a, q.b, q.c, q.d, q.e, q.ei, q.ej, q.ek = a, b, c, d, e, ei, ej, ek
        return q

    @classmethod
    def product(cls, *args):
        q = copy.copy(args[0])

        for j in args[1:]:

            # elements of first quaternion in dual quaternion product

            a = (q.a * j.a) - (q.b * j.b) - (q.c * j.c) - (q.d * j.d)
            b = (q.a * j.b) + (q.b * j.a) + (q.c * j.d) - (q.d * j.c)
            c = (q.a * j.c) - (q.b * j.d) + (q.c * j.a) + (q.d * j.b)
            d = (q.a * j.d) + (q.b * j.c) - (q.c * j.b) + (q.d * j.a)

            # elements of the second quaternion in dual quaternion product

            e = ((q.a * j.e) - (q.b * j.ei) - (q.c * j.ej) - (q.d * j.ek)) + ((q.e * j.a) - (q.ei * j.b) - (q.ej * j.c) - (q.ek * j.d))
            ei = ((q.a * j.ei) + (q.b * j.e) + (q.c * j.ek) - (q.d * j.ej)) + ((q.e * j.b) + (q.ei * j.a) + (q.ej * j.d) - (q.ek * j.c))
            ej = ((q.a * j.ej) - (q.b * j.ek) + (q.c * j.e) + (q.d * j.ei)) + ((q.e * j.c) - (q.ei * j.d) + (q.ej * j.a) + (q.ek * j.b))
            ek = ((q.a * j.ek) + (q.b * j.ej) - (q.c * j.ei) + (q.d * j.e)) + ((q.e * j.d) + (q.ei * j.c) - (q.ej * j.b) + (q.ek * j.a))

            # redefine q values to new calculated ones

            q.a, q.b, q.c, q.d, q.e, q.ei, q.ej, q.ek = a, b, c, d, e, ei, ej, ek

        return q
