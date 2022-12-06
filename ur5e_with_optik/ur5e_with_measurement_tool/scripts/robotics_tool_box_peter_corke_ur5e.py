import roboticstoolbox as rtb
from spatialmath.base import transl
from math import pi
import numpy as np 

class UR5e(rtb.DHRobot):

    """

    Class that models a Universal Robotics UR5e manipulator



    :param symbolic: use symbolic constants

    :type symbolic: bool



    ``UR5e()`` is an object which models a Universal Robot UR5e and

    describes its kinematic and dynamic characteristics using standard DH

    conventions.



    .. runblock:: pycon



        >>> import roboticstoolbox as rtb

        >>> robot = rtb.models.DH.UR5e()

        >>> print(robot)



    Defined joint configurations are:



    - qz, zero joint angle configuration

    - qr, arm horizontal along x-axis



    .. note::

        - SI units are used.



    :References:



        - `Parameters for calculations of kinematics and dynamics <https://www.universal-robots.com/articles/ur/parameters-for-calculations-of-kinematics-and-dynamics>`_



    :sealso: :func:`UR3`, :func:`UR5`, :func:`UR10`





    .. codeauthor:: Leon Gorissen

    """

    def __init__(
        self,
        symbolic=False,
        d: list = [0.1625, 0, 0, 0.1333, 0.0997, 0.0996],
        a: list = [0, -0.42500, -0.3922, 0, 0, 0],
        alpha: list = [pi / 2, 0, 0, pi / 2, -pi / 2, 0],
    ):

        """UR5e class init.



        Args:

            symbolic (bool, optional): _description_. Defaults to False.

            d (list, optional): d values for robot. Defaults to [0.1625, 0, 0, 0.1333, 0.0997, 0.0996].

            a (list, optional): a values for robot. Defaults to [0, -0.42500, -0.3922, 0, 0, 0].

            alpha (list, optional): alpha values in rad for robot. Defaults to [pi / 2, 0, 0, pi / 2, -pi / 2, 0].

        """

        if symbolic:

            import spatialmath.base.symbolic as sym

            zero = sym.zero()

            pi = sym.pi()

        else:

            from math import pi

            zero = 0.0

        deg = pi / 180

        inch = 0.0254

        mm = 1e-3

        tool_offset = (36.6) * mm

        # robot length values (metres)

        a = a

        d = d

        alpha = alpha

        # mass data, no inertia available

        mass = [3.761, 8.058, 2.846, 1.37, 1.3, 0.365]

        center_of_mass = [
            [0, -0.02561, 0.00193],
            [0.2125, 0, 0.11336],
            [0.15, 0, 0.0265],
            [0, -0.0018, 0.01634],
            [0, 0.0018, 0.01634],
            [0, 0, -0.001159],
        ]

        links = []

        for j in range(6):

            link = rtb.RevoluteDH(
                d=d[j], a=a[j], alpha=alpha[j], m=mass[j], r=center_of_mass[j], G=1
            )

            links.append(link)

        super().__init__(
            links,
            name="UR5e",
            manufacturer="Universal Robotics",
            keywords=("dynamics", "symbolic"),
            tool=transl(0, 0, tool_offset),
        )

        self.qr = np.array([180, 0, 0, 0, 90, 0]) * deg

        self.qz = np.zeros(6)

        self.addconfiguration("qr", self.qr)

        self.addconfiguration("qz", self.qz)


