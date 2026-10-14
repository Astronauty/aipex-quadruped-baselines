#include "convex_mpc/gait_planner.hpp"


using namespace std;
using namespace Eigen;

GaitPlanner::GaitPlanner(GaitType gait_type, double duty_factor, double gait_duration_s, double swing_height_m, double footstep_planning_horizon_s, double start_time_s)
    : gait_type_(gait_type),
      duty_factor_(duty_factor),
      gait_duration_s_(gait_duration_s),
      swing_height_m_(swing_height_m),
      footstep_planning_horizon_s_(footstep_planning_horizon_s),
      start_time_s_(start_time_s)
{
    this->set_phase_offsets();                 // Define gait types based on phase offsets
    this->update_time_and_phase(start_time_s); // Initialize the current time and phase

    // Initialize the swing leg trajectory queue for each leg
    swing_leg_trajectories_["FL"] = {};
    swing_leg_trajectories_["FR"] = {};
    swing_leg_trajectories_["RL"] = {};
    swing_leg_trajectories_["RR"] = {};
}

void GaitPlanner::set_phase_offsets()
{
    switch (gait_type_)
    {
    case GaitType::STAND:
        phase_offsets_ =
            {
                {"FL", 0.0},
                {"FR", 0.0},
                {"RL", 0.0},
                {"RR", 0.0}};
        break;
    case GaitType::TROT:
        phase_offsets_ =
            {
                {"FL", 0.0},
                {"FR", 0.5},
                {"RL", 0.5},
                {"RR", 0.0}};
        break;
    case GaitType::BOUND:
        phase_offsets_ =
            {
                {"FL", 0.5},
                {"FR", 0.5},
                {"RL", 0.0},
                {"RR", 0.0}};
        break;

    default:
        throw std::invalid_argument("Selected gait type is not implemented.");
    }
}

void GaitPlanner::update_time_and_phase(double current_time_s)
{
    this->current_time_s_ = current_time_s;
    this->gait_phase_ = time_to_phase(current_time_s);
}

unordered_map<std::string, int> GaitPlanner::get_contact_state()
{
    return get_contact_state_at_phase(gait_phase_);
}

unordered_map<std::string, int> GaitPlanner::get_contact_state_at_phase(double phase)
{
    unordered_map<std::string, int> contact_state;

    for (const auto &[leg, offset] : phase_offsets_)
    {
        contact_state[leg] = leg_in_stance(phase, offset) ? 1 : 0;
    }

    return contact_state;
}

/**
 * @brief Stance test for a single leg at a given gait phase.
 *
 * Shared by every contact query so the schedule the MPC constrains against and
 * the schedule the torque FSM switches on can never disagree. C fmod returns a
 * negative result for a negative dividend, which happens whenever the phase is
 * evaluated before start_time_s_; normalize back into [0, 1) so such a query
 * does not report a spurious stance.
 */
bool GaitPlanner::leg_in_stance(double phase, double phase_offset) const
{
    double adjusted_phase = fmod(phase + phase_offset, 1.0);
    if (adjusted_phase < 0.0)
        adjusted_phase += 1.0;

    return adjusted_phase < duty_factor_;
}

double GaitPlanner::get_phase(double current_time) const
{
    double phase = fmod((current_time - start_time_s_), gait_duration_s_) / gait_duration_s_;
    if (phase < 0.0)
        phase += 1.0;

    return phase;
}

/**
 * @brief Contact schedule sampled across the MPC horizon.
 *
 * The MPC constrains N_MPC-1 timesteps, and over a 0.25 s horizon a trot flips
 * contact partway through. Applying a single contact state to every step would
 * have the solver plan forces on feet that have already lifted, so the schedule
 * has to be sampled per step rather than frozen at k=0.
 *
 * Returns array<bool,4> indexed by leg (FL, FR, RL, RR) rather than a
 * string-keyed map: this runs inside the 40 Hz control loop and a map would cost
 * a string hash per leg per timestep.
 */
std::vector<std::array<bool, 4>> GaitPlanner::get_contact_schedule(double start_time_s, double dt, int N) const
{
    static const std::array<const char *, 4> leg_names = {{"FL", "FR", "RL", "RR"}};

    std::vector<std::array<bool, 4>> schedule;
    schedule.reserve(N);

    for (int k = 0; k < N; k++)
    {
        const double phase = get_phase(start_time_s + k * dt);

        std::array<bool, 4> stance{};
        for (int leg = 0; leg < 4; leg++)
        {
            stance[leg] = leg_in_stance(phase, phase_offsets_.at(leg_names[leg]));
        }
        schedule.push_back(stance);
    }

    return schedule;
}

void GaitPlanner::set_gait_type(GaitType gait_type)
{
    gait_type_ = gait_type;
    set_phase_offsets(); // Update phase offsets based on the new gait type
}

/**
 * @brief Switch gait and restart the gait clock at the given time.
 *
 * Used to hand over from the startup stance phase to the walking gait. The phase
 * origin is moved to start_time_s so the new gait begins at phase 0 -- every leg
 * that starts in stance gets a full stance period before its first lift-off,
 * rather than being dropped into the middle of a cycle.
 *
 * The duty factor has to change with the gait: STAND holds it at 1.0, and
 * inheriting that into a trot would leave every leg permanently in stance.
 *
 * Stale swing trajectories from the previous gait are discarded; they were built
 * against the old schedule and would otherwise be matched by end time and reused.
 */
void GaitPlanner::start_gait(GaitType gait_type, double duty_factor, double start_time_s)
{
    gait_type_ = gait_type;
    duty_factor_ = duty_factor;
    start_time_s_ = start_time_s;

    set_phase_offsets();

    for (auto &[leg, trajectories] : swing_leg_trajectories_)
    {
        (void)leg;
        trajectories.clear();
    }

    update_time_and_phase(start_time_s);
}

/**
 * @brief Compute the desired footstep position based on the the hip position (projection onto ground plane), CoM velocity, and foot index.
 * @param x The state vector at the desired footstep time.
 * @param foot_index The index of the foot (FL, FR, RL, RR) for which to compute the desired position.
 * @return The desired footstep position in the world frame, projected onto the ground plane.
 */
Vector3d GaitPlanner::compute_desired_footstep_position(const Eigen::VectorXd &x, const string &foot_index)
{
    // Compute the desired footstep position based on the reference position, CoM velocity, and foot index
    const Eigen::Vector3d p_BoH_B = p_BoH_B_.at(foot_index); // nominal foothold H about Bo, in B

    double roll = x[IDX_R];
    double pitch = x[IDX_P];
    double yaw = x[IDX_Y];
    // The body position slot is the base origin Bo (see p_BoH_B_), not the CoM.
    const Vector3d p_WBo_W = x.segment<3>(IDX_X);
    const Vector3d v_WBo_W = x.segment<3>(IDX_Xd);

    // Both the pose and the velocity come from x, the trajectory passed in.
    // Cheetah 3 eq. (33) wants the STATE ESTIMATE for the step touching down now
    // and the reference for later steps. That is NOT what happens today: x is
    // always a latched reference state (X_ref no longer seeds slot 0 with the
    // measurement, and even when it did, a target frozen at lift-off was read
    // past the end of the horizon and never reached slot 0). The fix belongs in
    // the caller, which should pass the measured state when it freezes a target.
    //
    // Both terms matter. Reading the reference VELOCITY made foot placement open
    // loop in speed (a 0.15 m/s command reached 0.906 m/s before diverging), and
    // reading the reference POSE projected the hip from where the robot was
    // commanded to be rather than where it is, so a drifting robot aimed its feet
    // sideways -- eq. (33)'s p_ref is "the location on the ground beneath the hip
    // of the robot", the actual robot.
    //
    // Historical note: this used to read x.segment<3>(6), which in the
    // [rpy, p, omega, v, g] layout is the body ANGULAR rate -- the heuristic was
    // adding an angular velocity to a position. Latent because GaitType::STAND
    // meant the function was never reached with motion.

    Matrix3d R_WB = (AngleAxisd(yaw, Vector3d::UnitZ()) *
                     AngleAxisd(pitch, Vector3d::UnitY()) *
                     AngleAxisd(roll, Vector3d::UnitX()))
                        .toRotationMatrix();

    const Vector3d p_WH_W = p_WBo_W + R_WB * p_BoH_B; // nominal foothold beneath the hip, world

    // Raibert heuristic: place the foot half a stance duration ahead along the
    // velocity vector. The stance duration is duty_factor * gait_duration_s --
    // the gait_duration_s factor was previously missing, leaving the expression
    // dimensionally wrong (m/s times a dimensionless ratio). It was masked only
    // because gait_duration_s happened to be hardcoded to 1.0.
    const double stance_duration_s = duty_factor_ * gait_duration_s_;
    Vector3d p_WF_W = p_WH_W + v_WBo_W * stance_duration_s / 2.0; // F: desired foothold

    // Project onto the ground plane. Without this the touchdown target sits at
    // roughly body height (~0.31 m) in the air and the robot cannot step at all,
    // despite the docs above having always claimed the projection was done.
    //
    // Flat-ground assumption; a terrain height lookup would replace this.
    p_WF_W.z() = 0.0;

    return p_WF_W;
}

double GaitPlanner::time_to_phase(double current_time_s)
{
    return get_phase(current_time_s);
}

double GaitPlanner::phase_to_time(double phase)
{
    // Returns the time corresponding to a given phase in the gait cycle
    if (phase < 0.0 || phase > 1.0)
        throw std::out_of_range("Phase must be in the range [0, 1]");

    double gait_cycle_start_time_s_ = current_time_s_ - fmod((current_time_s_ - start_time_s_), gait_duration_s_);

    return phase * gait_duration_s_ + gait_cycle_start_time_s_;
}

std::unordered_map<std::string, std::deque<SwingLegTrajectory>> GaitPlanner::update_swing_leg_trajectories(const Eigen::VectorXd &X_ref, const MPCParams &mpc_params, const unordered_map<string, Vector3d> &current_foot_positions)
{
    const bool verbose = mpc_params.verbose_diagnostics;

    if (verbose)
    {
        cout << "\nUpdating swing leg trajectories..." << endl;
        cout << "Start time: " << start_time_s_ << endl;
        cout << "Current time: " << current_time_s_ << ", Current phase: " << gait_phase_ << endl;
        cout << "Footstep planning horizon: " << footstep_planning_horizon_s_ << " seconds" << endl;
    }

    clear_expired_swing_leg_trajectories();

    auto future_swing_times = get_future_swing_times(current_time_s_, footstep_planning_horizon_s_);
    unordered_map<std::string, int> contact_state = get_contact_state();

    Vector3d p_des_start;

    for (const auto &[leg, swing_times] : future_swing_times)            
    {
        for (const auto &[start_time, end_time] : swing_times)
        {
            if (verbose) {
                cout << "\nLeg: " << leg << ", Swing start time: " << start_time - current_time_s_ << ", Swing end time: " << end_time - current_time_s_ << endl;
            }

            VectorXd x = get_state_at_time_from_ref_traj(X_ref, mpc_params, current_time_s_, end_time); // End of swing corresponds to start of stance
            Vector3d p_des_end = compute_desired_footstep_position(x, leg); // Cheetah 3 eq. (33)

            auto &leg_trajectories = swing_leg_trajectories_[leg];

            if (leg_trajectories.empty())
            {
                // No previous trajectory: lift off from where the foot actually is.
                p_des_start = current_foot_positions.at(leg);
            }
            else
            {
                // Lift off from the previous swing's touchdown, which is where the
                // foot will have been standing through the intervening stance.
                p_des_start = leg_trajectories.back().get_end_position();
            }

            // Diagnostic mode: step in place. Touchdown is forced back to the
            // lift-off point, so the foot rises and sets down exactly where it
            // started and the footstep planner contributes no horizontal motion.
            // This splits the search space when a gait diverges -- if the robot
            // still falls with the Raibert target removed, the fault is in the
            // swing tracking or its frame conversions rather than in footstep
            // placement.
            if (step_in_place_)
            {
                p_des_end = p_des_start;
            }

            // Match on the swing window rather than on the deque's back element.
            // The planning horizon can hold several swings per leg, so back() is
            // the furthest-future swing, not necessarily the one this iteration
            // is about -- keying on end_time updates the right trajectory.
            //
            // The tolerance must be well above double resolution at the clock's
            // magnitude. Times are absolute epoch seconds (~1.8e9), where one ULP is
            // 2.4e-7 s, so the old 1e-9 never matched a window recomputed with
            // different rounding: every leg carried a duplicate swing in ~36% of
            // plan cycles, and the next lift-off chained off the copy. Swing
            // windows are a gait half-period apart, so 1 ms cannot alias two.
            constexpr double kSameWindowTolS = 1e-3;
            auto existing = std::find_if(
                leg_trajectories.begin(), leg_trajectories.end(),
                [end_time](const SwingLegTrajectory &traj) {
                    return std::abs(traj.get_end_time() - end_time) < kSameWindowTolS;
                });

            if (existing != leg_trajectories.end())
            {
                // Already planned. Only the touchdown target moves, as the Raibert
                // heuristic re-evaluates against the latest reference trajectory.
                // In step-in-place mode the target is fixed, so leave it alone.
                //
                // Must retarget, NOT set_end_position. set_end_position rebuilds
                // the spline from the ORIGINAL lift-off point, so the new curve
                // does not pass through where the foot currently is and the
                // reference teleports. Measured on the pinned rig at 200 Hz: ref z
                // jumped -0.2848 -> -0.2410 m between consecutive 5 ms ticks, an
                // 8.8 m/s step, which at Kp=400 alone accounts for the 112 Nm
                // spikes and the 1.4e4 Nm peaks. retarget() re-anchors the start
                // at the current point and carries its velocity, so position and
                // velocity stay continuous across the update.
                if (!step_in_place_) {
                    const double t_now = current_time_s_;
                    const bool in_flight = t_now > existing->get_start_time() &&
                                           t_now < existing->get_end_time();

                    if (!in_flight)
                    {
                        // Not yet lifted off: free to rebuild, no continuity
                        // constraint and no apex to preserve. The target therefore
                        // keeps tracking the latest command right up to lift-off.
                        existing->set_end_position(p_des_end);
                    }
                    // Once airborne the foothold is FROZEN: the trajectory it lifted
                    // off with is the one it flies. No mid-flight retargeting.
                    //
                    // Mid-flight retarget was tried with a 20 mm deadband and is
                    // removed because retarget() cannot preserve the apex -- it
                    // rebuilds via build3ClampedAutoMid, which places a fresh
                    // swing_height above the midpoint of the REMAINING, shortened
                    // window, dragging the reference upward late in the swing (ref z
                    // measured oscillating -0.1288 / -0.1420 / -0.1175 / -0.1737 on
                    // consecutive 5 ms ticks).
                    //
                    // The deadband was sized on the assumption that the target is
                    // "essentially static at zero commanded velocity". That held
                    // while the heuristic read the COMMANDED velocity, but it now
                    // reads the MEASURED velocity (Cheetah 3 eq. 33), which swings
                    // over ~0.54 m/s within a gait cycle as the body surges. Through
                    // v * t_stance / 2 that is a ~130 mm target excursion against a
                    // 20 mm deadband, so the rebuild fired almost every planning
                    // cycle instead of rarely.
                    //
                    // Freezing is also what the fixed-timing schedule already
                    // assumes: the MPC is given these footholds as the r_i it plans
                    // forces about, so a foothold that moves mid-flight invalidates
                    // the forces already committed to. A disturbance mid-swing is
                    // corrected by the NEXT step, which replans from the measured
                    // state anyway.
                }
                continue;
            }

            leg_trajectories.emplace_back(start_time, end_time, p_des_start, p_des_end, swing_height_m_);
        }
    }

    return swing_leg_trajectories_;
}

void GaitPlanner::clear_expired_swing_leg_trajectories()
{
    for (auto &[leg, trajectories] : swing_leg_trajectories_)
    {
        // Remove trajectories that have ended before the current time
        trajectories.erase(
            std::remove_if(trajectories.begin(), trajectories.end(),
                [this](const SwingLegTrajectory& traj) {
                    return traj.get_end_time() < current_time_s_;
                }),
            trajectories.end()
        );
    }
}

// Get the time in seconds corresponding to the next time the given leg enters stance
double GaitPlanner::time_to_next_stance(double current_time_s, const std::string &leg)
{
    double phase = time_to_phase(current_time_s);
    double leg_phase = fmod(phase + phase_offsets_[leg], 1.0);

    return (1.0 - leg_phase) * gait_duration_s_;
}

// double GaitPlanner::time_to_next_swing(double current_time_s, const std::string& leg)
// {
//     double phase = time_to_phase(current_time_s);
//     double leg_phase = fmod(phase + phase_offsets_[leg], 1.0);

//     if (leg_phase < duty_factor_)
//         return (duty_factor_ - leg_phase) * gait_duration_s_;
//     else
//         return 0.0;
// }

std::unordered_map<std::string, std::vector<std::pair<double, double>>> GaitPlanner::get_future_swing_times(double current_time_s, double footstep_planning_horizon_s)
{
    std::unordered_map<std::string, std::vector<std::pair<double, double>>> future_swing_times;

    for (const auto &leg : {"FL", "FR", "RL", "RR"})
    {
        future_swing_times[leg] = std::vector<std::pair<double, double>>();
    }

    double end_time = current_time_s + footstep_planning_horizon_s; // End time to find future swing entry times

    // A gait with no swing phase has no swings to enumerate. STAND holds
    // duty_factor at 1.0, so (1 - duty_factor) * gait_duration_s is exactly zero
    // and every window below would come back with start == end -- a degenerate
    // spline. This is reached during the startup stance hold, when the planner is
    // still STAND but the node is already calling update_swing_leg_trajectories().
    if (duty_factor_ >= 1.0)
    {
        return future_swing_times; // all legs present, each with an empty list
    }

    for (const auto &[leg, leg_phase_offset] : phase_offsets_)
    {
        vector<pair<double, double>> &leg_swing_times = future_swing_times[leg];

        double phase = time_to_phase(current_time_s);
        double leg_phase = fmod(phase + leg_phase_offset, 1.0); // Adjust phase for the leg

        if (leg_phase < 0.0)
            leg_phase += 1.0;

        double time_cursor = current_time_s; // Get the time to the next stance phase for the leg

        // If currently in swing, add remaining portion of swing
        if (leg_phase >= duty_factor_)
        {
            double current_swing_start = current_time_s - (leg_phase - duty_factor_) * gait_duration_s_; // Start of current swing phase
            double current_swing_end = current_time_s + (1.0 - leg_phase) * gait_duration_s_;
            leg_swing_times.emplace_back(current_swing_start, current_swing_end);
            // double clipped_swing_end = min(current_swing_end, end_time); // If the planning horizon ends before swing phase ends, clip the end time
            // leg_swing_times.emplace_back(current_time_s, clipped_swing_end);

            // Move cursor to start of next swing
            time_cursor = current_time_s + (1.0 - leg_phase) * gait_duration_s_ + duty_factor_ * gait_duration_s_; // Move cursor to the end of the current swing phase
        }
        else
        {
            // Currently in stance, find start of next swing phase
            time_cursor = current_time_s + (duty_factor_ - leg_phase) * gait_duration_s_; // Start of next swing phase
        }

        const double swing_duration_s = (1.0 - duty_factor_) * gait_duration_s_;

        while (time_cursor < end_time)
        {
            const double swing_start = time_cursor;
            const double swing_end = swing_start + swing_duration_s;

            // Emit the swing at its TRUE duration, never clipped to the planning
            // horizon. Clipping used to set end = min(swing_end, end_time), so a
            // swing starting just inside the horizon came back with
            // start == end -- a zero-duration window. SwingLegTrajectory then
            // built a spline over [t0, t0], whose coefficients divide by the
            // duration, producing a degenerate curve and the 1e6-1e7 Nm torques.
            // With a 1.0 s horizon and a 0.5 s period the horizon boundary lands
            // exactly on swing boundaries, so this fired on almost every cycle.
            //
            // A swing that extends past the horizon is correct to plan in full:
            // the horizon bounds which swings to plan, not how long a swing is.
            // get_state_at_time_from_ref_traj extrapolates past the MPC horizon,
            // so querying the true end time is safe.
            leg_swing_times.emplace_back(swing_start, swing_end);

            time_cursor += gait_duration_s_;
        }
    }
    return future_swing_times;
}



bool GaitPlanner::get_current_footstep_target(const std::string &leg, double current_time_s,
                                               Eigen::Vector3d &target_position,
                                               Eigen::Vector3d &target_velocity) const
{

    // Check if leg exists in trajectories
    auto leg_it = swing_leg_trajectories_.find(leg);
    if (leg_it == swing_leg_trajectories_.end() || leg_it->second.empty()) {
        // No trajectories for this leg
        return false;
    }
    
    const auto& trajectories = leg_it->second;
    
    // Find the active trajectory for the current time
    for (const auto& trajectory : trajectories) {
        if (current_time_s >= trajectory.get_start_time() && 
            current_time_s <= trajectory.get_end_time()) {
            
            // Found active trajectory - evaluate at current time
            Eigen::Vector3d acceleration; // We don't need this, but evaluate() requires it
            trajectory.evaluate(current_time_s, target_position, target_velocity, acceleration);

            if (verbose_)
            {
                std::cout << "Active trajectory found for leg " << leg
                          << " at time " << current_time_s
                          << ", position: " << target_position.transpose()
                          << ", velocity: " << target_velocity.transpose() << std::endl;
            }

            return true;
        }
    }

    // Check if we're before the first trajectory starts
    if (!trajectories.empty() && current_time_s < trajectories.front().get_start_time()) {
        // Return start position of first trajectory with zero velocity
        target_position = trajectories.front().get_start_position();
        target_velocity = Eigen::Vector3d::Zero();

        if (verbose_)
        {
            std::cout << "Before first trajectory for leg " << leg
                      << ", using start position: " << target_position.transpose() << std::endl;
        }

        return true;
    }

    // Check if we're after the last trajectory ends
    if (!trajectories.empty() && current_time_s > trajectories.back().get_end_time()) {
        // Return end position of last trajectory with zero velocity
        target_position = trajectories.back().get_end_position();
        target_velocity = Eigen::Vector3d::Zero();

        if (verbose_)
        {
            std::cout << "After last trajectory for leg " << leg
                      << ", using end position: " << target_position.transpose() << std::endl;
        }

        return true;
    }

    // No valid trajectory found
    if (verbose_)
    {
        std::cout << "Warning: No valid trajectory found for leg " << leg
                  << " at time " << current_time_s << std::endl;
    }

    return false;
}

            
std::unordered_map<std::string, std::pair<Eigen::Vector3d, Eigen::Vector3d>> 
GaitPlanner::get_all_current_footstep_targets(double current_time_s) const
{
    std::unordered_map<std::string, std::pair<Eigen::Vector3d, Eigen::Vector3d>> targets;
    
    for (const auto& leg : {"FL", "FR", "RL", "RR"}) {
        Eigen::Vector3d position, velocity;

        // Legs with no planned trajectory are simply absent from the result. The
        // previous zero-vector fallback was worse than no answer: a swing leg
        // would have been commanded toward the world origin, metres away. Callers
        // must check for the key and hold position when it is missing.
        if (get_current_footstep_target(leg, current_time_s, position, velocity)) {
            targets[leg] = std::make_pair(position, velocity);
        } else if (verbose_) {
            std::cout << "No swing trajectory for leg " << leg
                      << " at time " << current_time_s << std::endl;
        }
    }

    return targets;
}

// int main(int, char **)
// {
//     // GaitPlanner gait_planner(GaitPlanner::GaitType::TROT, 0.5, 1.0);
//     double duty_factor = 0.5;
//     double gait_duration_s = 1.0;
//     double swing_height_m = 0.08;
//     double footstep_planning_horizon = 0.8;
//     double start_time_s = 0.0;

//     GaitPlanner gait_planner = GaitPlanner(GaitType::TROT, duty_factor, gait_duration_s, swing_height_m, footstep_planning_horizon, start_time_s);
//     gait_planner.update_time_and_phase(start_time_s);

//     vector<double> phases = {0.0};

//     for (double phase : phases)
//     {
//         cout << "Contact states at phase: " << phase << endl;
//         unordered_map<std::string, int> contact_state = gait_planner.get_contact_state(phase);
//         for (const auto &[leg, state] : contact_state)
//             cout << "Leg: " << leg << ", Contact State: " << state << endl;
//     }

//     // Dummy reference trajectory for testing
//     int MPC_HORIZON = 10;
//     int N_STATES = 12;

//     VectorXd X_ref = Eigen::VectorXd::Zero(N_STATES * MPC_HORIZON);

//     for (int i = 0; i < MPC_HORIZON; ++i)
//     {
//         VectorXd state = VectorXd::Zero(12); // Initialize with zeros
//         state(3) = 1.0 * i;                  // Add 1.0 to the x position at each step (x position is at index 3)
//         X_ref.segment<12>(i * 12) = state;
//     }

//     MPCParams mpc_params = MPCParams(
//         MPC_HORIZON,                            // N_MPC
//         12,                                     // N_CONTROLS
//         N_STATES,                               // N_STATES
//         0.1,                                    // dt
//         MatrixXd::Identity(N_STATES, N_STATES), // Q
//         MatrixXd::Identity(12, 12),             // R
//         VectorXd::Constant(12, -1.0),           // u_lower
//         VectorXd::Constant(12, 1.0)             // u_upper
//     );

//     unordered_map<string, Vector3d> current_footstep_positions = {
//         {"FL", Vector3d(0.3, 0.2, 0.0)},
//         {"FR", Vector3d(0.3, -0.2, 0.0)},
//         {"RL", Vector3d(-0.3, 0.2, 0.0)},
//         {"RR", Vector3d(-0.3, -0.2, 0.0)}};

//     std::unordered_map<std::string, std::deque<SwingLegTrajectory>> swing_leg_trajectories = gait_planner.update_swing_leg_trajectories(X_ref, mpc_params, current_footstep_positions);

//     for (const auto &[leg, trajectories] : swing_leg_trajectories)
//     {
//         cout << "\nSwing trajectories for leg: " << leg << endl;
//         for (const auto &traj : trajectories)
//         {
//             cout << "Start time: " << traj.get_start_time() << ", End time: " << traj.get_end_time() << endl;
//             cout << "Start position: " << traj.get_start_position().transpose() << ", End position: " << traj.get_end_position().transpose() << endl;
//             // cout << "Apex position: " << traj.apex_position.transpose() << endl;
//         }
//     }


//     return 0;
// }

std::unordered_map<std::string, SwingLegTrajectory>
GaitPlanner::get_active_swing_trajectories() const
{
    std::unordered_map<std::string, SwingLegTrajectory> active;

    for (const auto &[leg, trajectories] : swing_leg_trajectories_)
    {
        if (trajectories.empty()) continue;

        // Pick the swing whose window CONTAINS the current time, not front().
        //
        // front() is "the swing in progress or the next one due" -- and those are
        // different things. clear_expired_swing_leg_trajectories() only drops a
        // swing once its end time is past, and it runs at the 40 Hz planning rate,
        // so for up to 25 ms after a swing ends front() is still the FINISHED
        // swing. publish_cmd evaluates this snapshot at 200 Hz and
        // SwingLegTrajectory::evaluate() clamps t to [t0, t2], so the reference sat
        // pinned at the old touchdown z and then jumped onto the new curve when the
        // snapshot refreshed. Measured: ref z wandering non-monotonically near the
        // apex and then stepping 86 mm in one 5 ms tick, which is what drove the
        // ~107-155 Nm spikes.
        //
        // A leg between swings (in stance) is deliberately absent from the result:
        // publish_cmd holds position for a leg with no entry, which is correct for
        // a stance leg, whereas handing it a stale swing curve is not.
        const SwingLegTrajectory *current = nullptr;
        for (const auto &traj : trajectories)
        {
            if (current_time_s_ >= traj.get_start_time() &&
                current_time_s_ <= traj.get_end_time())
            {
                current = &traj;
                break;
            }
        }

        if (current != nullptr) active.emplace(leg, *current);
    }

    return active;
}
