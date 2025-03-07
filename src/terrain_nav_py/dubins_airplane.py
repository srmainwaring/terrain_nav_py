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
        self._rho_: float = turningRadius

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

        self._stateInterpolation = DubinsAirplaneStateSpace.DubinsAirplaneState(self)

    # NOTE: use composition rather than inheritance to implement.
    # class StateType : public ob::CompoundStateSpace::StateType {
    class DubinsAirplaneState:
        """
        The state in the DA2 state space, consisting of
        """

        def __init__(self, state_space: ob.CompoundStateSpace):
            """
            Constructor
            """
            self._state = ob.State(state_space)

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

    # /** \brief getMaximumExtent
    #   * Get the maximum value a call to distance() can return (or an upper bound).
    #   *
    #   * For unbounded state spaces, this function can return infinity.
    #   * \note Tight upper bounds are preferred because the value of the extent is used in
    #   * the automatic computation of parameters for planning. If the bounds are less tight,
    #   * the automatically computed parameters will be less useful.
    #   *
    #   * TODO Think about a meaningful and reasonable MaximumExtent for Dubins state space.
    #   * Remember, the length of (non-optimal) Dubins airplane paths do not define a proper metric space.
    #   * Currently the getMaximumExtent function of the CompoundStateSpace is used. */
    # virtual double getMaximumExtent() const override;
    def getMaximumExtent(self) -> float:
        # or the DubinsAirplaneStateSpace this computes:
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

    # /** \brief getEuclideanExtent
    #   * Get the maximum extent of the RealVectorStateSpace part of the DubinsAirplaneStateSpace */
    # double getEuclideanExtent() const;
    def getEuclideanExtent(self) -> float:
        # TODO: implement
        return 0.0

    # /** \brief validSegmentCount
    #   * Count how many segments of the "longest valid length" fit on the motion from \e state1 to \e state2.
    #   * Used to determine the number of states for collision detection. Always returns the dubins airplane
    #   * distance even though useEuclideanDistance == true. */
    # virtual unsigned int validSegmentCount(const ob::State* state1, const ob::State* state2) const override;
    def validSegmentCount(self, state1: ob.State, state2: ob.State) -> int:
        # TODO: implement
        return 0

    # /** \brief distance
    #   * Returns the length of the (non-optimal) Dubins airplane path connecting \a state1 and \a state2.
    #   */
    # virtual double distance(const ob::State* state1, const ob::State* state2) const override;
    def distance(self, state1: ob.State, state2: ob.State) -> float:
        # TODO: implement
        return 0.0

    # /** \brief euclidean_distance
    #   * Returns distance with is an approximation to the dubins airplane path between \a state1 and \a state2.
    #   */
    # double euclidean_distance(const ob::State* state1, const ob::State* state2) const;
    def euclidean_distance(self, state1: ob.State, state2: ob.State) -> float:
        # TODO: implement
        return 0.0

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
        dp: DubinsPath = None
        return dp

    # /** \brief interpolate
    #   * Calculates the \a state in between \a from and \a to after a fraction of \a t of the length of the path
    #   *
    #   * @param[in] from: Start state
    #   * @param[in] to: End state
    #   * @param[in] t: Fraction of the length of the path.
    #   * @param[out] state: The interpolated state.
    #   */
    # virtual void interpolate(const ob::State* from, const ob::State* to, const double t, ob::State* state) const override;
    def interpolate(
        self, from_state: ob.State, to_state: ob.State, t: float
    ) -> ob.State:
        # TODO: implement
        state: ob.State = None
        return state

    # /** \brief interpolate
    #   * Calculates the \a state in between \a from and \a to after a fraction of \a t of the length of the \a path.
    #   *
    #   * This function is called by virtual void interpolate(const ob::State *from, const ob::State *to, const double t,
    #   * ob::State *state) const; and is used in the DubinsMotionValidator for more efficient state validation
    #   *
    #   * @param[in] from: Start state
    #   * @param[in] to: End state
    #   * @param[in] t: Fraction of the length of the path.
    #   * @param[in] firstTime: Indicates if the interpolation is done the first time for this path
    #   * @param[out] path: The computed path between start and end state.
    #   * @param[out] SegmentStarts: The computed segment starts of the dubins path.
    #   * @param[out] state: The interpolated state.
    #   */
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
        # TODO: implement
        path: DubinsPath = None
        segmentStarts: DubinsAirplaneStateSpace.SegmentStarts = None
        state: ob.State = None
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
        state: ob.State = None
        return state

    # /** \brief calculateSegments
    #   * Calculates the \a state in between \a from and \a to after a fraction of \a t of the length of the \a path.
    #   *
    #   * This function is called by virtual void interpolate(const ob::State *from, const ob::State *to, const double t,
    #   * ob::State *state) const; and is used in the DubinsMotionValidator for more efficient state validation
    #   *
    #   * @param[in] from: Start state
    #   * @param[in] to: End state
    #   * @param[out] path: The computed path between start and end state.
    #   * @param[out] SegmentStarts: The computed segment starts of the dubins path.
    #   */
    # void calculateSegments(const ob::State* from, const ob::State* to, DubinsPath& path,
    #                         SegmentStarts& segmentStarts) const;
    def calculateSegments(
        self, from_state: ob.State, to_state: ob.State
    ) -> tuple[DubinsPath, SegmentStarts]:
        # TODO: implement
        path: DubinsPath = None
        segmentStarts: DubinsAirplaneStateSpace.SegmentStarts = None
        return (path, segmentStarts)

    # /** \brief enforceBounds
    #   * Bring the state within the bounds of the state space.
    #   */
    # virtual void enforceBounds(ob::State* state) const override;
    def enforceBounds(self, state: ob.State) -> None:
        # TODO: implement
        pass

    # /** \brief setMaxClimbingAngle
    #   * Get the value of the maximum climbing angle.
    #   */
    # void setMaxClimbingAngle(double maxClimb);
    def setMaxClimbingAngle(self, maxClimb: float) -> None:
        # TODO: implement
        pass

    # /** \brief getMaxClimbingAngle
    #   * Get the value of the maximum climbing angle.
    #   */
    # double getMaxClimbingAngle() const;
    def getMaxClimbingAngle(self) -> float:
        # TODO: implement
        return 0.0

    # /** \brief getOneDivSinGammaMax
    #   * Get the value of 1/sin(gammaMax).
    #   */
    # double getOneDivSinGammaMax() const;
    def getOneDivSinGammaMax(self) -> float:
        # TODO: implement
        return 0.0

    # /** \brief setMinTurningRadius
    #   * Set the value of the minimum turning radius and update the value of the curvature.
    #   */
    # void setMinTurningRadius(double r_min);
    def setMinTurningRadius(self, r_min: float) -> None:
        # TODO: implement
        pass

    # /**
    #   * @brief set enable set of the dubins set classification
    #   *
    #   */
    # void setEnableSetClassification(bool enable) { enable_classification_ = enable; };
    def setEnableSetClassification(self, enable: bool) -> None:
        self._enable_classification = enable

    # /** \brief getMinTurningRadius
    #   * Return the value of the minimum turning radius.
    #   */
    # double getMinTurningRadius() const;
    def getMinTurningRadius(self) -> float:
        # TODO: implement
        return 0.0

    # /** \brief getCurvature
    #   * Return the value of the maximum curvatere (1/r_min).
    #   */
    # double getCurvature() const;
    def getCurvature(self) -> float:
        # TODO: implement
        return 0.0

    # /** \brief setUseOptStSp
    #   * Set the value of optimalStSp_ which defines whether the optimal dubins airplane
    #   * path are computed or the suboptimal ones.
    #   *
    #   * \note The optimal dubins airplane paths do not work at the moment.
    #   */
    # void setUseOptStSp(bool useOptStSp);
    def setUseOptStSp(self, useOptStSp: bool) -> None:
        # TODO: implement
        pass

    # /** \brief setUseEuclideanDistance
    #   * Set the value of useEuclideanDistance_ which defines whether the euclidean distance
    #   * is computed or the dubins airplane distance.
    #   */
    # void setUseEuclideanDistance(bool useEuclDist);
    def setUseEuclideanDistance(self, useEuclDist: bool) -> None:
        # TODO: implement
        pass

    # /** \brief getUseEuclideanDistance
    #   * Return if the euclidean distance is computed instead of the
    #   * dubins airplane distance (useEuclideanDistance_).
    #   */
    # bool getUseEuclideanDistance() const;
    def getUseEuclideanDistance(self) -> bool:
        # TODO: implement
        return False

    # /** \brief setUseWind
    #   * Set if the wind should be used to compute the path from one state to another.
    #   */
    # void setUseWind(bool useWind);
    def setUseWind(self, useWind: bool) -> None:
        # TODO: implement
        pass

    # /** \brief getUseWind
    #   * Get the value of useWind.
    #   */
    # bool getUseWind() const;
    def getUseWind(self) -> bool:
        # TODO: implement
        return False

    # /** \brief dubinsWindPrintXthError_
    #   * Set the value of dubinsWindPrintXthError_.
    #   */
    # void setDubinsWindPrintXthError(int print_xth_error);
    def setDubinsWindPrintXthError(self, print_xth_error: int) -> None:
        # TODO: implement
        pass

    # /** \brief setMeteoGrid
    #   * Set the meteo grid.
    #   */
    # // void setMeteoGrid(const std::shared_ptr<base::MeteoGridClass>& meteoGrid);
    # def setMeteoGrid(self, meteoGrid: ob.MeteoGridClass) -> None:
    #     # TODO: implement
    #     pass

    # /** \brief getMeteoGrid
    #   * Return the meteo grid.
    #   */
    # // std::shared_ptr<base::MeteoGridClass> getMeteoGrid() const;
    # def getMeteoGrid(self) -> ob.MeteoGridClass:
    #     # TODO: implement
    #     pass

    # /** \brief isMetricSpace
    #   * Return if the state space is metric.
    #   *
    #   * The returned value is false because the space is not a metric:
    #   * The distance defined as the length of the (non-optimal) Dubins airplane path between two points
    #   * is not a metric since it is not symmetric and does not fulfill the triangle inequality. It is a premetric.
    #   * If using optimal Dubins airplane paths, it is a quasimetric since the triangle inequality is fulfilled.
    #   */
    # virtual bool isMetricSpace() const override;
    def isMetricSpace(self) -> bool:
        # TODO: implement
        return False

    # /** \brief hasSymmetricDistance
    #   * Return if the distance is symmetric (isSymmetric_)
    #   */
    # virtual bool hasSymmetricDistance() const override;
    def hasSymmetricDistance(self) -> bool:
        # TODO: implement
        return False

    # /** \brief hasSymmetricInterpolate
    #   * Return if the interpolation is symmetric (isSymmetric_)
    #   */
    # virtual bool hasSymmetricInterpolate() const override;
    def hasSymmetricInterpolate(self) -> bool:
        # TODO: implement
        return False

    # /** \brief sanityChecks
    #   * Perform sanity checks for this state space. Throws an exception if failures are found.
    #   * \note This checks if distances are always positive, whether the integration works as expected, etc.
    #   */
    # virtual void sanityChecks() const override;
    def sanityChecks(self) -> None:
        # TODO: implement
        pass

    # /** \brief setBounds
    #   * Set the bounds of this state space. This defines
    #   * the range of the space in which sampling is performed.
    #   */
    # void setBounds(const ob::RealVectorBounds& bounds);
    def setBounds(self, bounds: ob.RealVectorBounds) -> None:
        # TODO: implement
        pass

    # /** \brief getBounds
    #   * Set the bounds of the state space.
    #   */
    # const ob::RealVectorBounds& getBounds() const;
    def getBounds(self) -> ob.RealVectorBounds:
        # TODO: implement
        bounds: ob.RealVectorBounds = None
        return bounds

    # /** \brief printStateSpaceProperties
    #   * Print the properties of the state space.
    #   */
    # void printStateSpaceProperties() const;
    def getBounds(self) -> None:
        # TODO: implement
        pass

    # /** \brief printCtrs
    #   * Print the control variables.
    #   */
    # void printCtrs() const;
    def printCtrs(self) -> None:
        # TODO: implement
        pass

    # /** \brief printDurations
    #   * Print the durations.
    #   */
    # void printDurations();
    def printDurations(self) -> None:
        # TODO: implement
        pass

    # /** \brief resetCtrs
    #   * Reset the control variables to 0.
    #   */
    # void resetCtrs();
    def resetCtrs(self) -> None:
        # TODO: implement
        pass

    # /** \brief resetDurations
    #   * Reset the durations to 0.
    #   */
    # void resetDurations();
    def resetDurations(self) -> None:
        # TODO: implement
        pass

    # /** \brief printDurationsAndCtrs
    #   * Print the durations and control variables.
    #   */
    # void printDurationsAndCtrs();
    def printDurationsAndCtrs(self) -> None:
        # TODO: implement
        pass

    # /** \brief resetDurationsAndCtrs
    #   * Reset the durations and control variables to 0.
    #   */
    # void resetDurationsAndCtrs();
    def resetDurationsAndCtrs(self) -> None:
        # TODO: implement
        pass

    # /** \brief convert_idx
    #   * Converts the input segment index (0-5) to the corresponding path segment index (0-2).
    #   */
    # unsigned int convert_idx(unsigned int i) const;
    def resetDurationsAndCtrs(self, i: int) -> int:
        # TODO: implement
        return 0

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
        segmentStarts: DubinsAirplaneStateSpace.SegmentStarts = None
        return segmentStarts

    # /** \brief getStateOnCircle
    #   * Calculates a \a state on a circle with radius rho_ after \a t degrees.
    #   * This function assumes, that the plane is flying with minimum radius rho_ and maximum climbing rate gammaMax_
    #   */
    # void getStateOnCircle(const ob::State* from, int rl /* right (0), left (1)*/, int ud /* up(0), down(1) */, double t,
    #                       ob::State* state) const;
    def getStateOnCircle(
        self, from_state: ob.State, rl: int, ud: int, t: float
    ) -> ob.State:
        # TODO: implement
        state: ob.State = None
        return state

    # // TODO: Check if it makes sense to use mutable class variables for the following functions to speed it up.
    # /** \brief t_lsr
    #   * Function to compute a value for classifying the dubins curve.
    #   */
    # double t_lsr(double d, double alpha, double beta, double sa, double sb, double ca, double cb) const;
    def t_lsr(
        self,
        d: float,
        alpha: float,
        beta: float,
        sa: float,
        sb: float,
        ca: float,
        cb: float,
    ) -> float:
        # TODO: implement
        return 0.0

    # /** \brief p_lsr
    #   * Function to compute a value for classifying the dubins curve.
    #   */
    # double p_lsr(double d, double alpha, double beta, double sa, double sb, double ca, double cb) const;
    def p_lsr(
        self,
        d: float,
        alpha: float,
        beta: float,
        sa: float,
        sb: float,
        ca: float,
        cb: float,
    ) -> float:
        # TODO: implement
        return 0.0

    # /** \brief q_lsr
    #   * Function to compute a value for classifying the dubins curve.
    #   */
    # double q_lsr(double d, double alpha, double beta, double sa, double sb, double ca, double cb) const;
    def q_lsr(
        self,
        d: float,
        alpha: float,
        beta: float,
        sa: float,
        sb: float,
        ca: float,
        cb: float,
    ) -> float:
        # TODO: implement
        return 0.0

    # /** \brief t_rsl
    #   * Function to compute a value for classifying the dubins curve.
    #   */
    # double t_rsl(double d, double alpha, double beta, double sa, double sb, double ca, double cb) const;
    def t_rsl(
        self,
        d: float,
        alpha: float,
        beta: float,
        sa: float,
        sb: float,
        ca: float,
        cb: float,
    ) -> float:
        # TODO: implement
        return 0.0

    # /** \brief p_rsl
    #   * Function to compute a value for classifying the dubins curve.
    #   */
    # double p_rsl(double d, double alpha, double beta, double sa, double sb, double ca, double cb) const;
    def p_rsl(
        self,
        d: float,
        alpha: float,
        beta: float,
        sa: float,
        sb: float,
        ca: float,
        cb: float,
    ) -> float:
        # TODO: implement
        return 0.0

    # /** \brief q_rsl
    #   * Function to compute a value for classifying the dubins curve.
    #   */
    # double q_rsl(double d, double alpha, double beta, double sa, double sb, double ca, double cb) const;
    def q_rsl(
        self,
        d: float,
        alpha: float,
        beta: float,
        sa: float,
        sb: float,
        ca: float,
        cb: float,
    ) -> float:
        # TODO: implement
        return 0.0

    # /** \brief t_rsr
    #   * Function to compute a value for classifying the dubins curve.
    #   */
    # double t_rsr(double d, double alpha, double beta, double sa, double sb, double ca, double cb) const;
    def t_rsr(
        self,
        d: float,
        alpha: float,
        beta: float,
        sa: float,
        sb: float,
        ca: float,
        cb: float,
    ) -> float:
        # TODO: implement
        return 0.0

    # /** \brief p_rsr
    #   * Function to compute a value for classifying the dubins curve.
    #   */
    # double p_rsr(double d, double alpha, double beta, double sa, double sb, double ca, double cb) const;
    def p_rsr(
        self,
        d: float,
        alpha: float,
        beta: float,
        sa: float,
        sb: float,
        ca: float,
        cb: float,
    ) -> float:
        # TODO: implement
        return 0.0

    # /** \brief q_rsr
    #   * Function to compute a value for classifying the dubins curve.
    #   */
    # double q_rsr(double d, double alpha, double beta, double sa, double sb, double ca, double cb) const;
    def q_rsr(
        self,
        d: float,
        alpha: float,
        beta: float,
        sa: float,
        sb: float,
        ca: float,
        cb: float,
    ) -> float:
        # TODO: implement
        return 0.0

    # /** \brief t_lsl
    #   * Function to compute a value for classifying the dubins curve.
    #   */
    # double t_lsl(double d, double alpha, double beta, double sa, double sb, double ca, double cb) const;
    def t_lsl(
        self,
        d: float,
        alpha: float,
        beta: float,
        sa: float,
        sb: float,
        ca: float,
        cb: float,
    ) -> float:
        # TODO: implement
        return 0.0

    # /** \brief p_lsl
    #   * Function to compute a value for classifying the dubins curve.
    #   */
    # double p_lsl(double d, double alpha, double beta, double sa, double sb, double ca, double cb) const;
    def p_lsl(
        self,
        d: float,
        alpha: float,
        beta: float,
        sa: float,
        sb: float,
        ca: float,
        cb: float,
    ) -> float:
        # TODO: implement
        return 0.0

    # /** \brief q_lsl
    #   * Function to compute a value for classifying the dubins curve.
    #   */
    # double q_lsl(double d, double alpha, double beta, double sa, double sb, double ca, double cb) const;
    def q_lsl(
        self,
        d: float,
        alpha: float,
        beta: float,
        sa: float,
        sb: float,
        ca: float,
        cb: float,
    ) -> float:
        # TODO: implement
        return 0.0

    # /** \brief dubinsLSL
    #   * Compute the dubins LSL path.
    #   */
    # DubinsPath dubinsLSL(double d, double alpha, double beta) const;
    def dubinsLSL(self, d: float, alpha: float, beta: float) -> DubinsPath:
        # TODO: implement
        path: DubinsPath = None
        return path

    # /** \brief dubinsLSL
    #   * Overloaded dubinsLSL function to compute the LSL path with precompute sine and cosine values.
    #   */
    # DubinsPath dubinsLSL(double d, double alpha, double beta, double sa, double sb, double ca, double cb) const;
    def dubinsLSL(
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
        path: DubinsPath = None
        return path

    # /** \brief dubinsRSR
    #   * Compute the dubins RSR path.
    #   */
    # DubinsPath dubinsRSR(double d, double alpha, double beta) const;
    def dubinsRSR(self, d: float, alpha: float, beta: float) -> DubinsPath:
        # TODO: implement
        path: DubinsPath = None
        return path

    # /** \brief dubinsRSR
    #   * Overloaded dubinsRSR function to compute the RSR path with precompute sine and cosine values.
    #   */
    # DubinsPath dubinsRSR(double d, double alpha, double beta, double sa, double sb, double ca, double cb) const;
    def dubinsRSR(
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
        path: DubinsPath = None
        return path

    # /** \brief dubinsRSL
    #   * Compute the dubins RSL path.
    #   */
    # DubinsPath dubinsRSL(double d, double alpha, double beta) const;
    def dubinsRSL(self, d: float, alpha: float, beta: float) -> DubinsPath:
        # TODO: implement
        path: DubinsPath = None
        return path

    # /** \brief dubinsRSL
    #   * Overloaded dubinsRSL function to compute the RSL path with precompute sine and cosine values.
    #   */
    # DubinsPath dubinsRSL(double d, double alpha, double beta, double sa, double sb, double ca, double cb) const;
    def dubinsRSL(
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
        path: DubinsPath = None
        return path

    # /** \brief dubinsLSR
    #   * Compute the dubins LSR path.
    #   */
    # DubinsPath dubinsLSR(double d, double alpha, double beta) const;
    def dubinsLSR(self, d: float, alpha: float, beta: float) -> DubinsPath:
        pass

    # /** \brief dubinsLSR
    #   * Overloaded dubinsLSR function to compute the LSR path with precompute sine and cosine values.
    #   */
    # DubinsPath dubinsLSR(double d, double alpha, double beta, double sa, double sb, double ca, double cb) const;
    def dubinsLSR(
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
        path: DubinsPath = None
        return path

    # /** \brief dubinsRLR
    #   * Compute the dubins RLR path.
    #   */
    # DubinsPath dubinsRLR(double d, double alpha, double beta) const;
    def dubinsRLR(self, d: float, alpha: float, beta: float) -> DubinsPath:
        # TODO: implement
        path: DubinsPath = None
        return path

    # /** \brief dubinsRLR
    #   * Overloaded dubinsRLR function to compute the RLR path with precompute sine and cosine values.
    #   */
    # DubinsPath dubinsRLR(double d, double alpha, double beta, double sa, double sb, double ca, double cb) const;
    def dubinsRLR(
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
        path: DubinsPath = None
        return path

    # /** \brief dubinsLRL
    #   * Compute the dubins LRL path.
    #   */
    # DubinsPath dubinsLRL(double d, double alpha, double beta) const;
    def dubinsLRL(self, d: float, alpha: float, beta: float) -> DubinsPath:
        # TODO: implement
        path: DubinsPath = None
        return path

    # /** \brief dubinsLRL
    #   * Overloaded dubinsLRL function to compute the LRL path with precompute sine and cosine values.
    #   */
    # DubinsPath dubinsLRL(double d, double alpha, double beta, double sa, double sb, double ca, double cb) const;
    def dubinsLRL(
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
        path: DubinsPath = None
        return path
