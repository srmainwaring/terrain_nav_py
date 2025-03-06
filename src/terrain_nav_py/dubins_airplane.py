# BSD 3-Clause License
#
# Copyright (c) 2025, Rhys Mainwaring
#
# Ported to Python from original C++ code in
# https://github.com/ethz-asl/terrain-navigation.git
#
# Copyright (c) 2015, Daniel Schneider, ASL;
#                     Florian Achermann, ASL;
#                     Philipp Oetthershagen, ASL
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""
Dubins airplane model

DUBINSAIRPLANE STATE SPACE for control-based planning with (non-optimal) Dubins airplane paths,
using geometric motion planning algorithms. (This is an extension ot the OMPL DubinsStateSpace)
"""


import math
import sys

from ompl import base as ob

from terrain_nav_py.dubins_path import DubinsPath

half_pi: float = 0.5 * math.pi
pi: float = math.pi
twopi: float = 2.0 * math.pi
one_div_twopi: float = 1.0 / twopi

DUBINS_EPS: float = 1.0e-2
DUBINS_ZERO: float = -1.0e-4


# /** \brief mod2pi
#  * Sets the input angle to the corresponding angle between 0 and 2pi.
#  * TODO Move it to a general file as this function can be used in many different functions/classes
#  */
def mod2pi(x: float) -> float:
    if x < 0 and x > DUBINS_ZERO:
        return 0

    return x - twopi * math.floor(x * one_div_twopi)


# /** \brief sgn
#  * Returns +1 for positive sign, -1 for negative sign and 0 if val=0
#  * TODO Move it to a general file as this function can be used in many different functions/classes
#  */
def sgn(val) -> int:
    return (0 < val) - (val < 0)


# DubinsAirplaneStateSpace
# A Dubins airplane state space for (non-optimal) planning using (non-optimal) Dubins airplane paths.
#
# NOTE: The DubinsAirplaneStateSpace is asymmetric!!!!
#
# Computations are based on these two papers:
#  [1] Time-optimal Paths for a Dubins airplane, Chitzsaz, LaValle, 2007
#  [2] Implementing Dubins Airplane Paths on Fixed-wing UAVs, Beard, McLain, 2013
# The intermediate altitude case is solved non-optimally to assure fast computation of the paths.
# Therefore, the paths are called: (non-optimal) Dubins airplane paths
#
# An attempt to solve all paths optimally according based on [2] is in the code (optimalStSp).
# However, there are start-goal configurations which do not work properly.
# Other start-goal configurations (short path cases) are still solved non-optimally.
#
#
# *****************************************************************************************
# DUBINS AIRPLANE STATE SPACE for geometric planning with Dubins Curves
# (Extension to OMPL DubinsStateSpace for 2D Dubins Car)
# *****************************************************************************************
# States
#  x0 = x       (position)
# 	x1 = y       (position)
#  x2 = z       (position)
#  x3 = theta   (yaw/heading angle)
# Inputs
#  u0 = gamma   (climb angle)
#  u1 = phi     (roll angle)
#
#  Note:
#    - The climb rate (z_dot) can be computed from the climb angle:            z_dot = tan(gamma)*V = tan(u0)*V
#    - The yaw rate (theta_dot) can be calculated from the roll angle (phi):   theta_dot = tan(phi)*g/V = tan(u1)*g/V
#
# Hence the Dubins Airplane Motion Model:
#  x_dot      = V*cos(theta)
#  y_dot      = V*sin(theta)
#  z_dot      = tan(u0)*V
#  theta_dot  = tan(u1)*g/V
#
# Assuming bounded climb angle u0_max, we get a maximum climb/ sink rate:
# u_{z,max} = tan(u0_max)*V
#
# Assuming bounded roll angle u1_max, we get a maximum yaw rate theta_dot_max or correspondingly a minimum turning
# radius r_min = rho = 1/tan(u1_max)*V^2/g
#
# For the computation of (non-optimal) Dubins airplane paths, it is sufficient to know
#  - the maximum climb/sink rate phi_max
#  - the minimum radius r_min
#
#
# TODO:
#  - Check if condition for long path case is correct (if absolute values for sin/cos inside the square root should be
#    taken or not)
#  - Check if classification is correct, sometimes calcDubPathWithClassification and calcDubPathWithoutClassification
#    do not give the same results for the long path case, current guess is that this happens due to floating point
#    inaccuracies.
#


class DubinsAirplaneStateSpace(ob.CompoundStateSpace):
    """
    Constructor

    :param t: length of first path segment of a 2D Dubins car path, defaults to 0.0
    :type t: float

    :param turningRadius: The minimal turning radius of the airplane, defaults to 66.66667
    :type turningRadius: float
    :param gam: The maximum climb angle of the airplane, defaults to 0.15
    :type gam: float
    :param useEuclDist: If true the euclidian distance is used, else the dubins airplane distance, defaults to False
    :type useEuclDist: float
    """

    def __init__(
        self,
        turningRadius: float = 66.66667,
        gam: float = 0.15,
        useEuclDist: bool = False,
    ):
        super().__init__()

        # Threshold for the maximum allowed squared distance to goal.
        # Used for the computation of the dubins path in the wind where the
        # optimal path is determined iteratively.
        #
        # NOTE: A too low number might lead to slow computation of the dubins path.
        #
        self.THRESHOLD_DISTANCE_GOAL_SQUARED: float = math.sqrt(3.0)

        # Maximum number of allowed iterations in the dubins path
        # computation with wind.
        #
        # NOTE: A too low number speeds up the computation in general but leads
        #       to more failed attemps of dubins path computation.
        #   */
        self.MAX_ITER: int = 12

        self._enable_classification: bool = True

        # Minimum turning radius
        self._rho: float = turningRadius

        # Maximum curvature (1/rho), for savings in computational cost
        self._curvature: float = 1.0 / turningRadius

        # Maximum climbing angle
        self._gammaMax: float = gam

        # tan(gammaMax), for savings in computational cost
        self._tanGammaMax: float = math.tan(gam)

        # 1/tan(gammaMax), for savings in computational cost
        self._tanGammaMaxInv: float = 1.0 / math.tan(gam)

        # sin(gammaMax), for savings in computational cost
        self._sin_gammaMax: float = math.sin(gam)

        # 1/sin(gammaMax), for savings in computational cost
        self._one_div_sin_gammaMax: float = 1.0 / math.sin(gam)

        # Use optimal State Space. Optimal State Space is not working properly yet
        self._optimalStSp: bool = False

        # Print a error message if the dubins path with wind failed to compute
        # a multiple of dubinsWindPrintXthError times.
        self._dubinsWindPrintXthError: int = 1000000

        # Shared pointer of the meteo grid.
        self._meteoGrid: ob.MeteoGridClass = None

        # (Non-optimal) Dubins airplane path used for distance computations,
        # for savings in computational cost
        self._dp = None

        # Use a modified euclidean distance for distance queries during planning.
        # This can be used is instead of using a distance function defined as the length of the
        # (non-optimal) Dubins airplane path.
        self._useEuclideanDistance: bool = useEuclDist

        # Number of cases a path of the type csc is computed as the optimal path between two states.
        # NOTE: Just used for testing/debugging.
        self._csc_ctr: int = 0

        # Number of cases a path of the type ccc is computed as the optimal path between two states.
        # NOTE: Just used for testing/debugging.
        self._ccc_ctr: int = 0

        # Number of cases a long type path is computed as the optimal path between two states.
        # NOTE: Just used for testing/debugging.
        self._long_ctr: int = 0

        #  Number of cases a short type path is computed as the optimal path between two states.
        self._short_ctr: int = 0

        # Number of times the computation of a dubins path with wind failed.
        # NOTE: Just used for testing/debugging.
        self._dp_failed_ctr: int = 0

        # Number of times the computation of a dubins path with wind failed because of too strong
        # wind in xy direction.
        # NOTE: Just used for testing/debugging.
        self._dp_failed_xy_wind_ctr: int = 0

        # Number of times the computation of a dubins path with wind failed because of too strong
        # wind in z direction.
        # NOTE: Just used for testing/debugging.
        self._dp_failed_z_wind_ctr: int = 0

        # Number of times the computation of a dubins path with wind was successful.
        # NOTE: Just used for testing/debugging.
        self._dp_success_ctr: int = 0

        # Variable to store an intermediate result (state) in the interpolate function.
        self._stateInterpolation: DubinsAirplaneStateSpace.DubinsAirplaneState = None

        # Variable to store an intermediate result (meteo data) in the compute wind drift function.
        self._CWD_meteoData = None

        # Duration spent computing the distance between two states.
        self._duration_distance: float = 0.0

        # Duration spent doing the interpolation.
        self._duration_interpolate: float = 0.0

        # Duration spent doing the interpolation for the motion validator.
        self._duration_interpolate_motionValidator: float = 0.0

        # Duration spent doing the calculation for the wind drift.
        self._duration_get_wind_drift: float = 0.0

        # variables use to store intermediate result in the interpolation function
        self._interpol_seg: float = 0.0
        self._interpol_tanGamma: float = 0.0
        self._interpol_phiStart: float = 0.0
        self._interpol_dPhi: float = 0.0
        self._interpol_v: float = 0.0
        self._interpol_iter: int = 0
        self._interpol_tmp: float = 0.0

        self.addSubspace(ob.RealVectorStateSpace(3), 1.0)
        self.addSubspace(ob.SO2StateSpace(), 1.0)
        self.lock()

        # TODO: verify ordering (add subspace before or after setting name?)
        self.setName("DubinsAirplane" + self.getName())

        # TODO: type is not being set (protected member of base class)
        # TODO: check if protected members of subclass are exposed to Python.
        self._type = ob.StateSpaceType.STATE_SPACE_SE3

        self._stateInterpolation = DubinsAirplaneStateSpace.DubinsAirplaneState(ob.State(self))

    # NOTE: In C++ this would subclass CompoundStateStateSpace::StateType
    #       which is CompoundStateSpace.CompoundStateInternal in Python.
    # To implement this requires overriding ob.StateSpace.allocState in
    # DubinsAirplaneStateSpace, but that is not supported in Python as the
    # bindings result in a dangling pointer exception when calling ob.State().
    #
    # Our work around is to create a wrapper class instead, which is not as
    # convenient, since we cannot just use state() to get the correct reference
    # type as we can for the bound C++ classes.
    #
    class DubinsAirplaneState:
        """
        The state in the DA2 state space, consisting of
        """

        def __init__(self, state: ob.State):
            """
            Constructor
            """
            self._state = state

        #   double getX() const;
        def getX(self) -> float:
            """
            Get the X component of the state
            """
            return self._state()[0][0]

        #   double getY() const;
        def getY(self) -> float:
            """
            Get the Y component of the state
            """
            return self._state()[0][1]

        #   double getZ() const;
        def getZ(self) -> float:
            """
            Get the Z component of the state
            """
            return self._state()[0][2]

        #   double getYaw() const;
        def getYaw(self) -> float:
            """
            Get the heading/yaw component of the state
            """
            return self._state()[1].value

        #   void setX(double x);
        def setX(self, value: float) -> None:
            """
            Set the X component of the state
            """
            self._state()[0][0] = value

        #   void setY(double y);
        def setY(self, value: float) -> None:
            """
            Set the Y component of the state
            """
            self._state()[0][1] = value

        #   void setZ(double z);
        def setZ(self, value: float) -> None:
            """
            Set the Z component of the state
            """
            self._state()[0][2] = value

        #   void setYaw(double yaw);
        def setYaw(self, value: float) -> None:
            """
            Set the Z component of the state
            """
            self._state()[1].value = value

        #   void setXYZ(double x, double y, double z);
        def setXYZ(self, x: float, y: float, z: float) -> None:
            """
            Set the X, Y and Z components of the state
            """
            self._state()[0][0] = x
            self._state()[0][1] = y
            self._state()[0][2] = z

        #   void setXYZYaw(double x, double y, double z, double yaw);
        def setXYZYaw(self, x: float, y: float, z: float, yaw: float) -> None:
            """
            Set the X, Y, Z and Yaw components of the state
            """
            self._state()[0][0] = x
            self._state()[0][1] = y
            self._state()[0][2] = z
            self._state()[1].value = yaw

        #   void addToX(double val);
        def addToX(self, value: float) -> None:
            """
            Add a value to the x position of the state
            """
            self._state()[0][0] += value

        #   void addToY(double val);
        def addToY(self, value: float) -> None:
            """
            Add a value to the y position of the state
            """
            self._state()[0][1] += value

        #   void addToZ(double val);
        def addToZ(self, value: float) -> None:
            """
            Add a value to the z position of the state
            """
            self._state()[0][2] += value

        #   /** \brief getPosValuePointer
        #     * Get a pointer to the position values.
        #     */
        #   const double* getPosValuePointer() const;

        #   /** \brief printState
        #     * Print the state together with a message.
        #     */
        #   void printState(const std::string& msg = "") const;
        # };

    # struct SegmentStarts {
    #   struct Start {
    #     double x;
    #     double y;
    #     double z;
    #     double yaw;
    #     Start() : x(0.0), y(0.0), z(0.0), yaw(0.0) {}
    #     Start(const Start& that) : x(that.x), y(that.y), z(that.z), yaw(that.yaw) {}
    #   };
    #
    #   std::array<Start, 6> segmentStarts;
    #   SegmentStarts() : segmentStarts{{Start(), Start(), Start(), Start(), Start(), Start()}} {}
    #   SegmentStarts(const SegmentStarts& that) : segmentStarts(that.segmentStarts) {}
    # };
    class SegmentStarts:
        """
        Struct to store the segment starts of a dubins path.
        """

        class Start:
            def __init__(self):
                self.x: float = 0.0
                self.y: float = 0.0
                self.z: float = 0.0
                self.yaw: float = 0.0

        def __init__(self):
            self.segmentStarts = [
                DubinsAirplaneStateSpace.SegmentStarts.Start(),
                DubinsAirplaneStateSpace.SegmentStarts.Start(),
                DubinsAirplaneStateSpace.SegmentStarts.Start(),
                DubinsAirplaneStateSpace.SegmentStarts.Start(),
                DubinsAirplaneStateSpace.SegmentStarts.Start(),
                DubinsAirplaneStateSpace.SegmentStarts.Start(),
            ]

    def getMaximumExtent(self) -> float:
        """
        Get the maximum value a call to distance() can return (or an upper bound).

        For unbounded state spaces, this function can return infinity.
        NOTE: Tight upper bounds are preferred because the value of the extent is used in
              the automatic computation of parameters for planning. If the bounds are less tight,
              the automatically computed parameters will be less useful.

        TODO Think about a meaningful and reasonable MaximumExtent for Dubins state space.
            Remember, the length of (non-optimal) Dubins airplane paths do not define a proper metric space.
            Currently the getMaximumExtent function of the CompoundStateSpace is used.
        """
        # TODO: test
        # For the DubinsAirplaneStateSpace this computes:
        # R3_max_extent + SO2_max_extent = sqrtf(bound_x^2 + bound_y^2 + bound_z^2) + pi */
        e = 0.0
        components_ = self.getSubspaces()
        componentCount_ = self.getSubspaceCount()
        weights_ = self.getSubspaceWeights()
        epsilon = sys.float_info.epsilon
        for i in range(componentCount_):
            if (
                weights_[i] >= epsilon
            ):  # avoid possible multiplication of 0 times infinity
                e += weights_[i] * components_[i].getMaximumExtent()
        return e

    def getEuclideanExtent(self) -> float:
        """
        Get the maximum extent of the RealVectorStateSpace part of the DubinsAirplaneStateSpace
        """
        # TODO: test
        # For the DubinsAirplaneStateSpace this computes:
        # R3_max_extent = sqrtf(bound_x^2 + bound_y^2 + bound_z^2) */
        components_ = self.getSubspaces()
        return components_[0].getMaximumExtent()

    def validSegmentCount(self, state1: ob.State, state2: ob.State) -> int:
        """
        Count how many segments of the "longest valid length" fit on the motion from state1 to state2.

        Used to determine the number of states for collision detection. Always returns the dubins airplane
        distance even though useEuclideanDistance == true.
        """
        # TODO: test
        longestValidSegmentCountFactor_ = self.getValidSegmentCountFactor()
        longestValidSegment_ = self.getLongestValidSegmentLength()

        if self._useEuclideanDistance:
            dist = self.distance(state1, state2)
            if math.isnan(dist):
                return 0
            else:
                return longestValidSegmentCountFactor_ * int(
                    math.ceil(dist / longestValidSegment_)
                )
        else:
            #  still compute the dubins airplane distance.
            self._useEuclideanDistance = False
            nd = longestValidSegmentCountFactor_ * int(
                math.ceil(self.distance(state1, state2) / longestValidSegment_)
            )
            self.useEuclideanDistance_ = True
            return nd

    def distance(self, state1: ob.State, state2: ob.State) -> float:
        """
        Returns the length of the (non-optimal) Dubins airplane path connecting \a state1 and \a state2.
        """
        # TODO: test
        if self._useEuclideanDistance:
            return self.euclidean_distance(state1, state2)
        else:
            self._dp = self.dubins(state1, state2)
            dist = self._rho * self._dp.length_3D()
            return dist

    def euclidean_distance(self, state1: ob.State, state2: ob.State) -> float:
        """
        Returns distance with is an approximation to the dubins airplane path between \a state1 and \a state2.
        """
        # TODO: test
        # TODO: coerce state correctly
        dubinsAirplane2State1 = DubinsAirplaneStateSpace.DubinsAirplaneState(state1)
        dubinsAirplane2State2 = DubinsAirplaneStateSpace.DubinsAirplaneState(state2)

        eucl_dist = (
            (
                (dubinsAirplane2State1.getX() - dubinsAirplane2State2.getX())
                * (dubinsAirplane2State1.getX() - dubinsAirplane2State2.getX())
            )
            + (
                (dubinsAirplane2State1.getY() - dubinsAirplane2State2.getY())
                * (dubinsAirplane2State1.getY() - dubinsAirplane2State2.getY())
            )
            + (
                (dubinsAirplane2State1.getZ() - dubinsAirplane2State2.getZ())
                * (dubinsAirplane2State1.getZ() - dubinsAirplane2State2.getZ())
            )
        )

        eucl_dist = math.sqrt(eucl_dist)

        dub_dist = (
            math.fabs(dubinsAirplane2State1.getZ() - dubinsAirplane2State2.getZ())
            * self._one_div_sin_gammaMax
        )

        return max(eucl_dist, dub_dist)

    # TODO: handle overloads (x2) of dubins, functions must be named differently

    # /** \brief dubins
    #   * Compute the (non-optimal) Dubins airplane path from SE(2)xR3 state state1 to SE(2)xR3 state state2
    #   *
    #   * @param[in] state1: Start state
    #   * @param[in] state2: Goal state
    #   * @param[out] dp: Computed dubins path.
    #   */
    # void dubins(const ob::State* state1, const ob::State* state2, DubinsPath& dp) const;
    def dubins(self, state1: ob.State, state2: ob.State) -> DubinsPath:
        # TODO: implement
        print("[DubinsAirplaneStateSpace] dubins")
        dp: DubinsPath = None
        return dp

    # TODO: handle overloads (x3) of interpolate correctly - functions must have different names

    # virtual void interpolate(const ob::State* from, const ob::State* to, const double t, ob::State* state) const override;
    def interpolate(
        self, from_state: ob.State, to_state: ob.State, t: float
    ) -> ob.State:
        """
        Calculates the state in between from and to after a fraction of t of the length of the path

        :param from_state: Start state.
        :param to_state: End state.
        :param t: Fraction of the length of the path.
        :return state: The interpolated state.
        """
        # TODO: test
        firstTime = True
        (_, _, state) = self.interpolate(from_state, to_state, t, firstTime)
        return state

    # virtual void interpolate(const ob::State* from, const ob::State* to, double t, bool& firstTime, DubinsPath& path,
    #                           SegmentStarts& segmentStarts, ob::State* state) const;
    def interpolate(
        self,
        from_state: ob.State,
        to_state: ob.State,
        t: float,
        firstTime: float,
        path: DubinsPath,
        segmentStarts: SegmentStarts,
        state: ob.State,
    ) -> tuple[DubinsPath, SegmentStarts, ob.State]:
        """
        Calculates the state in between from and to after a fraction of t of the length of the path.

        This function is called by interpolate(from_state, to_state, t) and is
        used in the DubinsMotionValidator for more efficient state validation

        :param from: Start state
        :param to: End state
        :param t: Fraction of the length of the path.
        :param firstTime: Indicates if the interpolation is done the first time for this path
        :return path: The computed path between start and end state.
        :return segmentStarts: The computed segment starts of the dubins path.
        :return state: The interpolated state.
        """
        # TODO: test
        # path: DubinsPath = None
        # segmentStarts: DubinsAirplaneStateSpace.SegmentStarts = None
        # state: ob.State = None
        # return (path, segmentStarts, state)

        # compute the path if interpolate is called the first time.
        if firstTime:
            path = self.dubins(from_state, to_state)

            # compute the segment starts
            segmentStarts = self.calculateSegmentStarts(from_state, path)

            #  this must be after the computation of the dubins path
            # (otherwise a state of an invalid path is returned.
            if t >= 1.0:
                if to_state != state:
                    # TODO: check Python signature for copyState
                    self.copyState(state, to_state)
                return
            if t <= 0.0:
                if from_state != state:
                    # TODO: check Python signature for copyState
                    self.copyState(state, from_state)
                return
            firstTime = False

        if math.isnan(path.length_3d()):
            state().setXYZ(sys.float_info.max, sys.float_info.max, sys.float_info.max)
        else:
            state = self.interpolate(path, segmentStarts, t)

        return (path, segmentStarts, state)

    # /** \brief interpolate
    #   * Calculates the \a state in between \a from and \a to after a fraction of \a t of the length of the known
    #   * (non-optimal) Dubins airplane path \a path.
    #   *
    #   * @param[in] path: Known dubins airplane path.
    #   * @param[in] segmentStarts: Known starts of the segments of the dubins airplane path.
    #   * @param[in] t: Fraction of the length of the path.
    #   * @param[out] state: Interpolated state.
    #   */
    # virtual void interpolate(const DubinsPath& path, const SegmentStarts& segmentStarts, double t,
    #                           ob::State* state) const;
    def interpolate(
        self, path: DubinsPath, segmentStarts: SegmentStarts, t: float
    ) -> ob.State:
        # TODO: implement
        print("[DubinsAirplaneStateSpace] interpolate")
        state: ob.State = None
        return state

    # void calculateSegments(const ob::State* from, const ob::State* to, DubinsPath& path,
    #                         SegmentStarts& segmentStarts) const;
    def calculateSegments(
        self, from_state: ob.State, to_state: ob.State
    ) -> tuple[DubinsPath, SegmentStarts]:
        """
        Calculates the state in between from and to after a fraction of t of the length of the path.

        This function is called by interpolate(from_state, to_state, t) and is
        used in the DubinsMotionValidator for more efficient state validation

        :param from_state: Start state
        :param to_state: End state
        :returns path: The computed path between start and end state.
        :returns segmentStarts: The computed segment starts of the dubins path.
        """
        # TODO: test
        #  compute the path if interpolate is called the first time.
        path = self.dubins(from_state, to_state)
        #  compute the segment starts
        segmentStarts = self.calculateSegmentStarts(from_state, path, segmentStarts)

        return (path, segmentStarts)

    def enforceBounds(self, state: ob.State) -> None:
        """
        Bring the state within the bounds of the state space.
        """
        # TODO: test
        # TODO: document C++ -> Python equivalence of the following
        #
        # 1. casting a state / obtain a reference to the internal state type
        #   C++:
        #     StateType* rstate = static_cast<StateType*>(state);
        #   Python:
        #     rstate = state()
        #
        # 2. index into a compound state
        #   C++
        #     rstate->as<ob::RealVectorStateSpace::StateType>(0)->values[i]
        #   Python:
        #     rtate(0).values[i]
        #
        # 3. index into a compound state space
        #   C++
        #     as<ob::RealVectorStateSpace>(0)->getBounds().high[i]
        #   Python:
        #     self(0)->getBounds().high[i]
        #

        print("[DubinsAirplaneStateSpace] enforceBounds")
        rstate = state()

        # RE3 part
        re3_space = self.getSubspace(0)
        for i in range(re3_space.getDimension()):
            if rstate(0).values[i] > re3_space.getBounds().high[i]:
                rstate(0).values[i] = re3_space.getBounds().high[i]
            elif rstate(0).values[i] < re3_space.getBounds().low[i]:
                rstate(0).values[i] = re3_space.getBounds().low[i]

        # Orientation (SO(2) part)
        so2_space = self.getSubspace(1)
        so2_space.enforceBounds(rstate(1))

    def setMaxClimbingAngle(self, maxClimb: float) -> None:
        """
        Get the value of the maximum climbing angle.
        """
        # TODO: test
        self._gammaMax = maxClimb
        self._tanGammaMax = math.tan(self._gammaMax)
        self._tanGammaMaxInv = 1.0 / self._tanGammaMax
        self._sin_gammaMax = math.sin(self._gammaMax)
        self._one_div_sin_gammaMax = 1.0 / self._sin_gammaMax

    def getMaxClimbingAngle(self) -> float:
        """
        Get the value of the maximum climbing angle.
        """
        # TODO: test
        return self._gammaMax

    def getOneDivSinGammaMax(self) -> float:
        """
        Get the value of 1/sin(gammaMax).
        """
        # TODO: test
        return self._one_div_sin_gammaMax

    def setMinTurningRadius(self, r_min: float) -> None:
        """
        Set the value of the minimum turning radius and update the value of the curvature.
        """
        # TODO: test
        self._rho = r_min
        self._curvature = 1 / self._rho

    def setEnableSetClassification(self, enable: bool) -> None:
        """
        Set enable set of the dubins set classification
        """
        # TODO: test
        self._enable_classification = enable

    def getMinTurningRadius(self) -> float:
        """
        Return the value of the minimum turning radius.
        """
        # TODO: test
        return self._rho

    def getCurvature(self) -> float:
        """
        Return the value of the maximum curvatere (1/r_min).
        """
        # TODO: test
        return self._curvature

    def setUseOptStSp(self, useOptStSp: bool) -> None:
        """
        Set the value of optimalStSp which defines whether the optimal dubins airplane
        path are computed or the suboptimal ones.

        NOTE: The optimal dubins airplane paths do not work at the moment.
        """
        # TODO: test
        self._optimalStSp = useOptStSp

    def setUseEuclideanDistance(self, useEuclDist: bool) -> None:
        """
        Set the value of useEuclideanDistance which defines whether the euclidean distance
        is computed or the dubins airplane distance.
        """
        # TODO: test
        self._useEuclideanDistance = useEuclDist

    def getUseEuclideanDistance(self) -> bool:
        """
        Return if the euclidean distance is computed instead of the
        dubins airplane distance (useEuclideanDistance_).
        """
        # TODO: test
        return self._useEuclideanDistance

    # /** \brief setUseWind
    #   * Set if the wind should be used to compute the path from one state to another.
    #   */
    # void setUseWind(bool useWind);
    # def setUseWind(self, useWind: bool) -> None:
    #     # TODO: not used in C++
    #     pass

    # /** \brief getUseWind
    #   * Get the value of useWind.
    #   */
    # bool getUseWind() const;
    # def getUseWind(self) -> bool:
    #     # TODO: not used in C++
    #     return False

    def setDubinsWindPrintXthError(self, print_xth_error: int) -> None:
        """
        Set the value of dubinsWindPrintXthError.
        """
        # TODO: test
        self._dubinsWindPrintXthError = print_xth_error

    # /** \brief setMeteoGrid
    #   * Set the meteo grid.
    #   */
    # // void setMeteoGrid(const std::shared_ptr<base::MeteoGridClass>& meteoGrid);
    # def setMeteoGrid(self, meteoGrid: ob.MeteoGridClass) -> None:
    #     # TODO: not used in C++
    #     pass

    # /** \brief getMeteoGrid
    #   * Return the meteo grid.
    #   */
    # // std::shared_ptr<base::MeteoGridClass> getMeteoGrid() const;
    # def getMeteoGrid(self) -> ob.MeteoGridClass:
    #     # TODO: not used in C++
    #     pass

    def isMetricSpace(self) -> bool:
        """
        Return if the state space is metric.

        The returned value is false because the space is not a metric:
        The distance defined as the length of the (non-optimal) Dubins airplane path between two points
        is not a metric since it is not symmetric and does not fulfill the triangle inequality. It is a premetric.
        If using optimal Dubins airplane paths, it is a quasimetric since the triangle inequality is fulfilled.
        """
        return False

    def hasSymmetricDistance(self) -> bool:
        """
        Return if the distance is symmetric
        """
        return False

    def hasSymmetricInterpolate(self) -> bool:
        """
        Return if the interpolation is symmetric
        """
        return False

    def sanityChecks(self) -> None:
        """
        Perform sanity checks for this state space. Throws an exception if failures are found.
        NOTE: This checks if distances are always positive, whether the integration works as expected, etc.
        """
        # TODO: test
        zero = sys.float_info.epsilon
        eps = sys.float_info.epsilon
        flags = not (
            ob.StateSpace.STATESPACE_INTERPOLATION
            or ob.StateSpace.STATESPACE_TRIANGLE_INEQUALITY
            or ob.StateSpace.STATESPACE_DISTANCE_BOUND
        )
        flags = flags and (not ob.StateSpace.STATESPACE_DISTANCE_SYMMETRIC)
        # don't do the sanity check in case of wind as it takes a long time and therefore blocks the benchmarking
        # we know that for our purpose the functionality is given
        super().sanityChecks(zero, eps, flags)

    def setBounds(self, bounds: ob.RealVectorBounds) -> None:
        """
        Set the bounds of this state space. This defines
        the range of the space in which sampling is performed.
        """
        # TODO: test
        print("[DubinsAirplaneStateSpace] setBounds")
        # self(0).setBounds(bounds)
        re3_space = self.getSubspace(0)
        re3_space.setBounds(bounds)

    def getBounds(self) -> ob.RealVectorBounds:
        """
        Set the bounds of the state space.
        """
        # TODO: test
        print("[DubinsAirplaneStateSpace] getBounds")
        # return self(0).getBounds()
        re3_space = self.getSubspace(0)
        return re3_space.getBounds()

    def printStateSpaceProperties(self) -> None:
        """
        Print the properties of the state space.
        """
        # TODO: test
        print(f"DubinsAirplaneStateSpace [{self.getName()}]")
        print(f"  !!! This state space is asymmetric !!!")
        print(f"  Airplane speed relative to ground:  9 m/s")
        print(f"  Euclidean Distance: {self.__useEuclideanDistance}")
        print(f"  Minimum turning radius: {self._rho} m")
        print(f"  Maximum climbing angle: {self._gammaMax} radian")
        print(
            f"  Using optimal Dubins airplane paths (not working properly): "
            f"{self._optimalStSp}"
        )
        print(
            f"  State space bounds (in meter and radian): "
            f"[{self.getBounds().low[0]}, {self.getBounds().high[0]}], "
            f"[{self.getBounds().low[1]} {self.getBounds().high[1]}], "
            f"[{self.getBounds().low[2]} {self.getBounds().high[2]}], "
            f"[-pi, pi)"
        )
        print()

    def printCtrs(self) -> None:
        """
        Print the control variables.
        """
        # TODO: implement
        print("[DubinsAirplaneStateSpace] printCtrs")
        pass

    def printDurations(self) -> None:
        """
        Print the durations.
        """
        # TODO: implement
        print("[DubinsAirplaneStateSpace] printDurations")
        pass

    def resetCtrs(self) -> None:
        """
        Reset the control variables to 0.
        """
        # TODO: test
        self._csc_ctr = 0
        self._ccc_ctr = 0
        self._long_ctr = 0
        self._short_ctr = 0
        self._dp_failed_ctr = 0
        self._dp_failed_xy_wind_ctr = 0
        self._dp_failed_z_wind_ctr = 0
        self._dp_success_ctr = 0

    def resetDurations(self) -> None:
        """
        Reset the durations to 0.
        """
        # TODO: test
        self._duration_distance = 0.0
        self._duration_interpolate = 0.0
        self._duration_interpolate_motionValidator = 0.0
        self._duration_get_wind_drift = 0.0

    def printDurationsAndCtrs(self) -> None:
        """
        Print the durations and control variables.
        """
        # TODO: test
        self.printDurations()
        self.printCtrs()

    def resetDurationsAndCtrs(self) -> None:
        """
        Reset the durations and control variables to 0.
        """
        # TODO: test
        self.resetDurations()
        self.resetCtrs()

    def convert_idx(self, i: int) -> int:
        """
        Converts the input segment index (0-5) to the corresponding path segment index (0-2).
        """
        # TODO: test and add bounds check
        # assert(i < 6 and "In convert_idx, i > 5")
        if i == 0:  # start helix
            return 0
        elif i == 1:  # first dubins segment
            # intermediate maneuver, return same direction as first dubins segment before
            # In interpolate function, will handle this and turn the other way.
            return 0
        elif i == 2:
            return 0
        elif i == 3:  # second dubins segment
            return 1
        else:  # third dubins segment and end helix
            return 2

    # TODO: handle overloads (x2) of dubins, functions must be named differently

    # /** \brief dubins
    #   * Compute the 2D dubins path using path classification for the long distance case and
    #   * no classification for the short distance case.
    #   *
    #   * @param[in] d: euclidean distance between start and goal state
    #   * @param[in] alpha: Corrected heading of the start state
    #   * @param[in] beta: Corrected heading of the goal state
    #   * @param[out] path: The computed dubins path.
    #   */
    # void dubins(double d, double alpha, double beta, DubinsPath& path) const;
    def dubins(self, d: float, alpha: float, beta: float) -> DubinsPath:
        # TODO: implement
        print("[DubinsAirplaneStateSpace] dubins")
        path: DubinsPath = None
        return path

    # protected:
    # /** \brief calcDubPathWithClassification
    #   * Compute the dubins airplane path with path classification.
    #   *
    #   * TODO Currently, classifies only samples that far enough from each other ("long paths")!
    #   *    Does not work properly when OPTIMAL Dubins AIRPLANE State Space is used! For intermediate case, there are cases
    #   * with d > ... and still CCC may be optimal (not 100% sure) Bigger parts of work:
    #   *      - Implement classification for short path case (see "Classification of the Dubins set, Shkel & Lumelsky, 2001)
    #   *      - Implement fast and fully optimal Dubins state space. Note that classification of the Dubins set will not be
    #   * correct anymore for some cases. *
    #   *
    #   * @param[out] path: The computed dubins path.
    #   * @param[in] d: euclidean distance between start and goal state
    #   * @param[in] alpha: Corrected heading of the start state
    #   * @param[in] beta: Corrected heading of the goal state
    #   * @param[in] sa: Precomputed sin(alpha)
    #   * @param[in] sb: Precomputed sin(beta)
    #   * @param[in] ca: Precomputed cos(alpha)
    #   * @param[in] cb: Precomputed cos(beta)
    #   */
    # void calcDubPathWithClassification(DubinsPath& path, double d, double alpha, double beta, double sa, double sb,
    #                                     double ca, double cb) const;
    def calcDubPathWithClassification(
        self,
        d: float,
        alpha: float,
        beta: float,
        sa: float,
        sb: float,
        ca: float,
        cb: float,
    ) -> DubinsPath:
        # TODO: implement
        print("[DubinsAirplaneStateSpace] calcDubPathWithClassification")
        path: DubinsPath = None
        return path

    # /** \brief calcDubPathWithoutClassification
    #   * Compute the dubins airplane path without path classification.
    #   * That means computing the paths for all six cases and returning the shortest
    #   * path.
    #   * Slower than calcDubPathWithClassification.
    #   *
    #   * @param[out] path: The computed dubins path.
    #   * @param[in] d: euclidean distance between start and goal state
    #   * @param[in] alpha: Corrected heading of the start state
    #   * @param[in] beta: Corrected heading of the goal state
    #   * @param[in] sa: Precomputed sin(alpha)
    #   * @param[in] sb: Precomputed sin(beta)
    #   * @param[in] ca: Precomputed cos(alpha)
    #   * @param[in] cb: Precomputed cos(beta)
    #   */
    # void calcDubPathWithoutClassification(DubinsPath& path, double d, double alpha, double beta, double sa, double sb,
    #                                       double ca, double cb) const;
    def calcDubPathWithoutClassification(
        self,
        d: float,
        alpha: float,
        beta: float,
        sa: float,
        sb: float,
        ca: float,
        cb: float,
    ) -> DubinsPath:
        # TODO: implement
        print("[DubinsAirplaneStateSpace] calcDubPathWithoutClassification")
        path: DubinsPath = None
        return path

    # /** \brief additionalManeuver
    #   * Calculates an additional maneuver such that in the intermediate altitude case an optimal path is returned.
    #   *
    #   * The implementation is based on the paper:
    #   *      Implementing Dubins Airplane Paths on Fixed-wing UAVs, Beard, McLain, 2013
    #   *
    #   * WARNING: This function does not yet work properly and hence does not yet find an optimal path in all cases.
    #   * Deviations in z-direction of the temporary goal and the final state of the calculated intermediate Dubins airplane
    #   * path are possible!
    #   * TODO: fix this function
    #   */
    # std::tuple<double, bool, double, double, double> additionalManeuver(const DubinsPath& dp, double& L_2D,
    #                                                                     const ob::State* state1,
    #                                                                     const ob::State* state2) const;
    def additionalManeuver(
        self, dp: DubinsPath, L_2D: float, state1: ob.State, state2: ob.State
    ) -> tuple[tuple[float, bool, float, float, float], float]:
        # TODO: implement
        print("[DubinsAirplaneStateSpace] additionalManeuver")
        pass

    # /** \brief classifyPath
    #   * Classify the path based on the heading from the start and goal state.
    #   *
    #   * @param[in] alpha: Corrected heading of the start state
    #   * @param[in] beta: Corrected heading of the goal state
    #   */
    # DubinsPath::Classification classifyPath(double alpha, double beta) const;
    def classifyPath(self, alpha: float, beta: float) -> DubinsPath.Classification:
        # TODO: implement
        print("[DubinsAirplaneStateSpace] classifyPath")
        c: DubinsPath.Classification = None
        return c

    # /** \brief computeOptRratio
    #   * Compute the opt radius ratio for the start/end helix.
    #   *
    #   * @param[in] fabsHdist: Absolute value of height of the helix.
    #   * @param[in] L: Length of the helix.
    #   * @param[in] fabsTanGamma: Maximum climb angle of the airplane.
    #   * @param[in] k: Number of circles in the helix.
    #   */
    # double computeOptRratio(double fabsHdist, double L, double fabsTanGamma, int k) const;
    def computeOptRratio(
        self, fabsHdist: float, L: float, fabsTanGamma: float, k: int
    ) -> float:
        # TODO: implement
        print("[DubinsAirplaneStateSpace] computeOptRratio")
        return 0.0

    # /** \brief interpolateWithWind
    #   * Calculates the \a state in between \a from and \a to after a fraction of \a t of the length of the known
    #   * (non-optimal) Dubins airplane path \a path with wind.
    #   *
    #   * @param[in] from: Start state of the path.
    #   * @param[in] path: Known dubins airplane path.
    #   * @param[in] segmentStarts: Known starts of the segments of the dubins airplane path.
    #   * @param[in] t: Fraction of the length of the path.
    #   * @param[out] state: Interpolated state.
    #   */
    # virtual void interpolateWithWind(const ob::State* from, const DubinsPath& path, const SegmentStarts& segmentStarts,
    #                                   double t, ob::State* state) const;
    def interpolateWithWind(
        self,
        from_state: ob.State,
        path: DubinsPath,
        segmentStarts: SegmentStarts,
        t: float,
    ) -> ob.State:
        # TODO: implement
        print("[DubinsAirplaneStateSpace] interpolateWithWind")
        state: ob.State = None
        return state

    # /** \brief calculateSegmentStarts
    #   * Calculates the segment starts of the input
    #   *
    #   * @param[in] from: Start state of the path.
    #   * @param[in] path: Known dubins airplane path.
    #   * @param[out] segmentStarts: Computed starts of the segments of the dubins airplane path.
    #   */
    # void calculateSegmentStarts(const ob::State* from, const DubinsPath& path, SegmentStarts& segmentStarts) const;
    def calculateSegmentStarts(
        self, from_state: ob.State, path: DubinsPath
    ) -> SegmentStarts:
        # TODO: implement
        print("[DubinsAirplaneStateSpace] calculateSegmentStarts")
        segmentStarts: DubinsAirplaneStateSpace.SegmentStarts = None
        return segmentStarts

    def getStateOnCircle(
        self, from_state: ob.State, rl: int, ud: int, t: float
    ) -> ob.State:
        """
        Calculates a \a state on a circle with radius rho after \a t degrees.
        This function assumes, that the plane is flying with minimum radius
        rho and maximum climbing rate gammaMax
        """
        # TODO: test
        # assuming flying with rho and gammaMax
        state = DubinsAirplaneStateSpace.DubinsAirplaneState(ob.State(self))
        s = DubinsAirplaneStateSpace.DubinsAirplaneState(ob.State(self))

        s.setXYZ(0.0, 0.0, 0.0)
        s.setYaw(from_state().getYaw())
        phi = s.getYaw()

        if rl == 1:  # left
            # TODO: precompute tanf(gammaMax_)
            s.setXYZ(
                s.getX() + math.sin(phi + t) - math.sin(phi),
                s.getY() - math.cos(phi + t) + math.cos(phi),
                s.getZ() + t * math.tan(ud * self._gammaMax),
            )
            s.setYaw(phi + t)

        if rl == 0:  # right
            # precompute tanf(gammaMax_)
            s.setXYZ(
                s.getX() - math.sin(phi - t) + math.sin(phi),
                s.getY() + math.cos(phi - t) - math.cos(phi),
                s.getZ() + t * math.tan(ud * self._gammaMax),
            )
            s.setYaw(phi - t)

        state.setX(s.getX() * self._rho + from_state().getX())
        state.setY(s.getY() * self._rho + from_state().getY())
        state.setZ(s.getZ() * self._rho + from_state().getZ())
        self._getSubspace(1).enforceBounds(s[1])
        state.setYaw(s.getYaw())
        return state

    @staticmethod
    def t_lsr(
        d: float,
        alpha: float,
        beta: float,
        sa: float,
        sb: float,
        ca: float,
        cb: float,
    ) -> float:
        """
        Function to compute a value for classifying the dubins curve.
        """
        # TODO: test
        tmp = -2.0 + d * d + 2.0 * (ca * cb + sa * sb + d * (sa + sb))
        p = math.sqrt(max(tmp, 0.0))
        theta = math.atan2(-ca - cb, d + sa + sb) - math.atan2(-2.0, p)
        return mod2pi(-alpha + theta)  # t

    @staticmethod
    def p_lsr(
        d: float,
        alpha: float,
        beta: float,
        sa: float,
        sb: float,
        ca: float,
        cb: float,
    ) -> float:
        """
        Function to compute a value for classifying the dubins curve.
        """
        # TODO: test
        tmp = -2.0 + d * d + 2.0 * (ca * cb + sa * sb + d * (sa + sb))
        return math.sqrt(max(tmp, 0.0))  # p

    @staticmethod
    def q_lsr(
        d: float,
        alpha: float,
        beta: float,
        sa: float,
        sb: float,
        ca: float,
        cb: float,
    ) -> float:
        """
        Function to compute a value for classifying the dubins curve.
        """
        # TODO: test
        tmp = -2.0 + d * d + 2.0 * (ca * cb + sa * sb + d * (sa + sb))
        p = math.sqrt(max(tmp, 0.0))
        theta = math.atan2(-ca - cb, d + sa + sb) - math.atan2(-2.0, p)
        return mod2pi(-beta + theta)  # q

    @staticmethod
    def t_rsl(
        d: float,
        alpha: float,
        beta: float,
        sa: float,
        sb: float,
        ca: float,
        cb: float,
    ) -> float:
        """
        Function to compute a value for classifying the dubins curve.
        """
        # TODO: test
        tmp = d * d - 2.0 + 2.0 * (ca * cb + sa * sb - d * (sa + sb))
        p = math.sqrt(max(tmp, 0.0))
        theta = math.atan2(ca + cb, d - sa - sb) - math.atan2(2.0, p)
        return mod2pi(alpha - theta)  # t

    @staticmethod
    def p_rsl(
        d: float,
        alpha: float,
        beta: float,
        sa: float,
        sb: float,
        ca: float,
        cb: float,
    ) -> float:
        """
        Function to compute a value for classifying the dubins curve.
        """
        # TODO: test
        tmp = d * d - 2.0 + 2.0 * (ca * cb + sa * sb - d * (sa + sb))
        return math.sqrt(max(tmp, 0.0))  # p

    @staticmethod
    def q_rsl(
        d: float,
        alpha: float,
        beta: float,
        sa: float,
        sb: float,
        ca: float,
        cb: float,
    ) -> float:
        """
        Function to compute a value for classifying the dubins curve.
        """
        # TODO: test
        tmp = d * d - 2.0 + 2.0 * (ca * cb + sa * sb - d * (sa + sb))
        p = math.sqrt(max(tmp, 0.0))
        theta = math.atan2(ca + cb, d - sa - sb) - math.atan2(2.0, p)
        return mod2pi(beta - theta)  # q

    @staticmethod
    def t_rsr(
        d: float,
        alpha: float,
        beta: float,
        sa: float,
        sb: float,
        ca: float,
        cb: float,
    ) -> float:
        """
        Function to compute a value for classifying the dubins curve.
        """
        # TODO: test
        theta = math.atan2(ca - cb, d - sa + sb)
        return mod2pi(alpha - theta)  # t

    @staticmethod
    def p_rsr(
        d: float,
        alpha: float,
        beta: float,
        sa: float,
        sb: float,
        ca: float,
        cb: float,
    ) -> float:
        """
        Function to compute a value for classifying the dubins curve.
        """
        # TODO: test
        tmp = 2.0 + d * d - 2.0 * (ca * cb + sa * sb - d * (sb - sa))
        return math.sqrt(max(tmp, 0.0))  # p

    @staticmethod
    def q_rsr(
        d: float,
        alpha: float,
        beta: float,
        sa: float,
        sb: float,
        ca: float,
        cb: float,
    ) -> float:
        """
        Function to compute a value for classifying the dubins curve.
        """
        # TODO: test
        theta = math.atan2(ca - cb, d - sa + sb)
        return mod2pi(-beta + theta)  # q

    @staticmethod
    def t_lsl(
        d: float,
        alpha: float,
        beta: float,
        sa: float,
        sb: float,
        ca: float,
        cb: float,
    ) -> float:
        """
        Function to compute a value for classifying the dubins curve.
        """
        # TODO: test
        theta = math.atan2(cb - ca, d + sa - sb)
        return mod2pi(-alpha + theta)  # t

    @staticmethod
    def p_lsl(
        d: float,
        alpha: float,
        beta: float,
        sa: float,
        sb: float,
        ca: float,
        cb: float,
    ) -> float:
        """
        Function to compute a value for classifying the dubins curve.
        """
        # TODO: test
        tmp = 2.0 + d * d - 2.0 * (ca * cb + sa * sb - d * (sa - sb))
        return math.sqrt(max(tmp, 0.0))  # p

    @staticmethod
    def q_lsl(
        d: float,
        alpha: float,
        beta: float,
        sa: float,
        sb: float,
        ca: float,
        cb: float,
    ) -> float:
        """
        Function to compute a value for classifying the dubins curve.
        """
        # TODO: test
        theta = math.atan2(cb - ca, d + sa - sb)
        return mod2pi(beta - theta)  # q

    @staticmethod
    def dubinsLSL(d: float, alpha: float, beta: float) -> DubinsPath:
        """
        Compute the dubins LSL path.
        """
        # TODO: test
        ca = math.cos(alpha)
        sa = math.sin(alpha)
        cb = math.cos(beta)
        sb = math.sin(beta)
        return DubinsAirplaneStateSpace.dubinsLSL_fast(d, alpha, beta, sa, sb, ca, cb)

    @staticmethod
    def dubinsLSL_fast(
        d: float,
        alpha: float,
        beta: float,
        sa: float,
        sb: float,
        ca: float,
        cb: float,
    ) -> DubinsPath:
        """
        Overloaded dubinsLSL function to compute the LSL path with precompute sine and cosine values.
        """
        # TODO: test
        tmp = 2.0 + d * d - 2.0 * (ca * cb + sa * sb - d * (sa - sb))
        if tmp >= DUBINS_ZERO:  # TODO Check if fabs is missing.
            theta = math.atan2(cb - ca, d + sa - sb)
            t = mod2pi(-alpha + theta)
            p = math.sqrt(max(tmp, 0.0))
            q = mod2pi(beta - theta)
            # assert(math.fabs(p * math.cos(alpha + t) - sa + sb - d) < DUBINS_EPS)
            # assert(math.fabs(p * math.sin(alpha + t) + ca - cb) < DUBINS_EPS)
            # assert(mod2pi(alpha + t + q - beta + 0.5 * DUBINS_EPS) < DUBINS_EPS)
            return DubinsPath(DubinsPath.TYPE_LSL, t, p, q)
        return DubinsPath()

    @staticmethod
    def dubinsRSR(d: float, alpha: float, beta: float) -> DubinsPath:
        """
        Compute the dubins RSR path.
        """
        # TODO: test
        ca = math.cos(alpha)
        sa = math.sin(alpha)
        cb = math.cos(beta)
        sb = math.sin(beta)
        return DubinsAirplaneStateSpace.dubinsRSR_fast(d, alpha, beta, sa, sb, ca, cb)

    @staticmethod
    def dubinsRSR_fast(
        d: float,
        alpha: float,
        beta: float,
        sa: float,
        sb: float,
        ca: float,
        cb: float,
    ) -> DubinsPath:
        """
        Overloaded dubinsRSR function to compute the RSR path with precompute sine and cosine values.
        """
        # TODO: test
        tmp = 2.0 + d * d - 2.0 * (ca * cb + sa * sb - d * (sb - sa))
        if tmp >= DUBINS_ZERO:  # TODO Check if fabs is missing.
            theta = math.atan2(ca - cb, d - sa + sb)
            t = mod2pi(alpha - theta)
            p = math.sqrt(max(tmp, 0.0))
            q = mod2pi(-beta + theta)
            # assert(math.fabs(p * math.cos(alpha - t) + sa - sb - d) < DUBINS_EPS)
            # assert(math.fabs(p * math.sin(alpha - t) - ca + cb) < DUBINS_EPS)
            # assert(mod2pi(alpha - t - q - beta + 0.5 * DUBINS_EPS) < DUBINS_EPS)
            return DubinsPath(DubinsPath.TYPE_RSR, t, p, q)
        return DubinsPath()

    @staticmethod
    def dubinsRSL(d: float, alpha: float, beta: float) -> DubinsPath:
        """
        Compute the dubins RSL path.
        """
        # TODO: test
        ca = math.cos(alpha)
        sa = math.sin(alpha)
        cb = math.cos(beta)
        sb = math.sin(beta)
        return DubinsAirplaneStateSpace.dubinsRSL_fast(d, alpha, beta, sa, sb, ca, cb)

    @staticmethod
    def dubinsRSL_fast(
        d: float,
        alpha: float,
        beta: float,
        sa: float,
        sb: float,
        ca: float,
        cb: float,
    ) -> DubinsPath:
        """
        Overloaded dubinsRSL function to compute the RSL path with precompute sine and cosine values.
        """
        # TODO: test
        tmp = d * d - 2.0 + 2.0 * (ca * cb + sa * sb - d * (sa + sb))
        if tmp >= DUBINS_ZERO:  # TODO Check if here fabs is missing.
            p = math.sqrt(max(tmp, 0.0))
            theta = math.atan2(ca + cb, d - sa - sb) - math.atan2(2.0, p)
            t = mod2pi(alpha - theta)
            q = mod2pi(beta - theta)
            # assert(math.fabs(p * math.cos(alpha - t) - 2.0 * math.sin(alpha - t) + sa + sb - d) < DUBINS_EPS)
            # assert(math.fabs(p * math.sin(alpha - t) + 2.0 * math.cos(alpha - t) - ca - cb) < DUBINS_EPS)
            # assert(mod2pi(alpha - t + q - beta + 0.5 * DUBINS_EPS) < DUBINS_EPS)
            return DubinsPath(DubinsPath.TYPE_RSL, t, p, q)
        return DubinsPath()

    @staticmethod
    def dubinsLSR(d: float, alpha: float, beta: float) -> DubinsPath:
        """
        Compute the dubins LSR path.
        """
        # TODO: test
        ca = math.cos(alpha)
        sa = math.sin(alpha)
        cb = math.cos(beta)
        sb = math.sin(beta)
        return DubinsAirplaneStateSpace.dubinsLSR_fast(d, alpha, beta, sa, sb, ca, cb)

    @staticmethod
    def dubinsLSR_fast(
        d: float,
        alpha: float,
        beta: float,
        sa: float,
        sb: float,
        ca: float,
        cb: float,
    ) -> DubinsPath:
        """
        Overloaded dubinsLSR function to compute the LSR path with precompute sine and cosine values.
        """
        # TODO: test
        tmp = -2.0 + d * d + 2.0 * (ca * cb + sa * sb + d * (sa + sb))
        if tmp >= DUBINS_ZERO:  # TODO Check if here fabs is missing.
            p = math.sqrt(max(tmp, 0.0))
            theta = math.atan2(-ca - cb, d + sa + sb) - math.atan2(-2.0, p)
            t = mod2pi(-alpha + theta)
            q = mod2pi(-beta + theta)
            # assert(math.fabs(p * math.cos(alpha + t) + 2.0 * math.sin(alpha + t) - sa - sb - d) < DUBINS_EPS)
            # assert(math.fabs(p * math.sin(alpha + t) - 2.0 * math.cos(alpha + t) + ca + cb) < DUBINS_EPS)
            # assert(mod2pi(alpha + t - q - beta + 0.5 * DUBINS_EPS) < DUBINS_EPS)
            return DubinsPath(DubinsPath.TYPE_LSR, t, p, q)
        return DubinsPath()

    @staticmethod
    def dubinsRLR(d: float, alpha: float, beta: float) -> DubinsPath:
        """
        Compute the dubins RLR path.
        """
        # TODO: test
        ca = math.cos(alpha)
        sa = math.sin(alpha)
        cb = math.cos(beta)
        sb = math.sin(beta)
        return DubinsAirplaneStateSpace.dubinsRLR_fast(d, alpha, beta, sa, sb, ca, cb)

    @staticmethod
    def dubinsRLR_fast(
        d: float,
        alpha: float,
        beta: float,
        sa: float,
        sb: float,
        ca: float,
        cb: float,
    ) -> DubinsPath:
        """
        Overloaded dubinsRLR function to compute the RLR path with precompute sine and cosine values.
        """
        # TODO: test
        tmp = 0.125 * (6.0 - d * d + 2.0 * (ca * cb + sa * sb + d * (sa - sb)))
        if math.fabs(tmp) < 1.0:
            p = twopi - math.acos(tmp)
            theta = math.atan2(ca - cb, d - sa + sb)
            t = mod2pi(alpha - theta + 0.5 * p)
            q = mod2pi(alpha - beta - t + p)
            # assert(math.fabs(2.0 * math.sin(alpha - t + p) - 2.0 * math.sin(alpha - t) - d + sa - sb) < DUBINS_EPS)
            # assert(math.fabs(-2.0 * math.cos(alpha - t + p) + 2.0 * math.cos(alpha - t) - ca + cb) < DUBINS_EPS)
            # assert(mod2pi(alpha - t + p - q - beta + 0.5 * DUBINS_EPS) < DUBINS_EPS)
            return DubinsPath(DubinsPath.TYPE_RLR, t, p, q)
        return DubinsPath()

    @staticmethod
    def dubinsLRL(d: float, alpha: float, beta: float) -> DubinsPath:
        """
        Compute the dubins LRL path.
        """
        # TODO: test
        ca = math.cos(alpha)
        sa = math.sin(alpha)
        cb = math.cos(beta)
        sb = math.sin(beta)
        return DubinsAirplaneStateSpace.dubinsLRL_fast(d, alpha, beta, sa, sb, ca, cb)

    @staticmethod
    def dubinsLRL_fast(
        d: float,
        alpha: float,
        beta: float,
        sa: float,
        sb: float,
        ca: float,
        cb: float,
    ) -> DubinsPath:
        """
        Overloaded dubinsLRL function to compute the LRL path with precompute sine and cosine values.
        """
        # TODO: test
        tmp = 0.125 * (6.0 - d * d + 2.0 * (ca * cb + sa * sb - d * (sa - sb)))
        if math.fabs(tmp) < 1.0:
            p = twopi - math.acos(tmp)
            theta = math.atan2(-ca + cb, d + sa - sb)
            t = mod2pi(-alpha + theta + 0.5 * p)
            q = mod2pi(beta - alpha - t + p)
            # assert(math.fabs(-2.0 * math.sin(alpha + t - p) + 2.0 * math.sin(alpha + t) - d - sa + sb) < DUBINS_EPS)
            # assert(math.fabs(2.0 * math.cos(alpha + t - p) - 2.0 * math.cos(alpha + t) + ca - cb) < DUBINS_EPS)
            # assert(mod2pi(alpha + t - p + q - beta + 0.5 * DUBINS_EPS) < DUBINS_EPS)
            return DubinsPath(DubinsPath.TYPE_LRL, t, p, q)
        return DubinsPath()
