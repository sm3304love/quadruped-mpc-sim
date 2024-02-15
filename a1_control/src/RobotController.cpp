
#include "a1_control/RobotController.h"

RobotController::RobotController()
{
    std::cout << "init controller" << std::endl;
    // init QP solver
    // init some parameters
    Q.diagonal() << 1.0, 1.0, 1.0, 400.0, 400.0, 100.0;
    R = 1e-3;
    mu = 0.5;
    hessian.resize(3 * NUM_LEG, 3 * NUM_LEG);
    gradient.resize(3 * NUM_LEG);
    linearMatrix.resize(NUM_LEG + 4 * NUM_LEG, 3 * NUM_LEG);
    lowerBound.resize(NUM_LEG + 4 * NUM_LEG);
    lowerBound.setZero();
    upperBound.resize(NUM_LEG + 4 * NUM_LEG);
    upperBound.setZero();

    // init mpc skip counter
    mpc_init_counter = 0;

    // constraint matrix fixed
    for (int i = 0; i < NUM_LEG; ++i)
    {
        // extract F_zi
        linearMatrix.insert(i, 2 + i * 3) = 1;
        // friction pyramid
        // 1. F_xi < uF_zi
        linearMatrix.insert(NUM_LEG + i * 4, i * 3) = 1;
        linearMatrix.insert(NUM_LEG + i * 4, 2 + i * 3) = -mu;
        lowerBound(NUM_LEG + i * 4) = -qpOASES::INFTY;
        // 2. F_xi > -uF_zi    ===> -F_xi -uF_zi < 0
        linearMatrix.insert(NUM_LEG + i * 4 + 1, i * 3) = -1;
        linearMatrix.insert(NUM_LEG + i * 4 + 1, 2 + i * 3) = -mu;
        lowerBound(NUM_LEG + i * 4 + 1) = -qpOASES::INFTY;
        // 3. F_yi < uF_zi
        linearMatrix.insert(NUM_LEG + i * 4 + 2, 1 + i * 3) = 1;
        linearMatrix.insert(NUM_LEG + i * 4 + 2, 2 + i * 3) = -mu;
        lowerBound(NUM_LEG + i * 4 + 2) = -qpOASES::INFTY;
        // 4. -F_yi > uF_zi
        linearMatrix.insert(NUM_LEG + i * 4 + 3, 1 + i * 3) = -1;
        linearMatrix.insert(NUM_LEG + i * 4 + 3, 2 + i * 3) = -mu;
        lowerBound(NUM_LEG + i * 4 + 3) = -qpOASES::INFTY;
    }
    // debug linearMatrix
    //    std::cout << Eigen::MatrixXd(linearMatrix) << std::endl;

    terrain_angle_filter = MovingWindowFilter(100);
    for (int i = 0; i < NUM_LEG; ++i)
    {
        recent_contact_x_filter[i] = MovingWindowFilter(60);
        recent_contact_y_filter[i] = MovingWindowFilter(60);
        recent_contact_z_filter[i] = MovingWindowFilter(60);
    }
}

RobotController::RobotController(ros::NodeHandle &_nh) : RobotController()
{
    std::cout << "init nh" << std::endl;
    nh = _nh;
    _nh.param("use_sim_time", use_sim_time);

    pub_terrain_angle = nh.advertise<std_msgs::Float64>("a1_debug/terrain_angle", 100);
}

void RobotController::update_plan(RobotStates &state, double dt)
{
    state.counter += 1;
    if (!state.movement_mode)
    {
        // movement_mode == 0, standstill with all feet in contact with ground
        for (bool &plan_contact : state.plan_contacts)
            plan_contact = true;    // plan_contacts set true --> all feet in contact with ground
        state.gait_counter_reset(); // counter reset
    }
    else
    {
        // movement_mode == 1, walk
        for (int i = 0; i < NUM_LEG; ++i) // each leg   --> 땅에 닿아있는 다리와 그렇지 않은 다리 구별 및 gait cycle
                                          // 확인
        {
            state.gait_counter(i) =
                state.gait_counter(i) +
                state.gait_counter_speed(i); // gait counter cycle update && gait_counter로 다리 제어 (swing <-> stance)
                                             // --> 초기 gait_counter가 다리별로 다르더라
            state.gait_counter(i) = std::fmod(
                state.gait_counter(i),
                state
                    .counter_per_gait); // counter_per_gait --> whole gait cycle --> counter_per_gait를 넘지 않도록 조정
            if (state.gait_counter(i) <=
                state.counter_per_swing) // gait_counter <=  counter_per_swing --> ground contact
            {
                state.plan_contacts[i] = true; // gait_counter <=  counter_per_swing --> ground contact
            }
            else
            {
                state.plan_contacts[i] = false; // gait_counter >  counter_per_swing --> swing phase
            }
        }
    }

    // update foot plan: state.foot_pos_target_world
    Eigen::Vector3d lin_vel_world = state.root_lin_vel;                             // world frame linear velocity
    Eigen::Vector3d lin_vel_rel = state.root_rot_mat_z.transpose() * lin_vel_world; // robot body frame linear velocity

    // Raibert Heuristic, calculate desired foot position
    state.foot_pos_target_rel = state.default_foot_pos;
    for (int i = 0; i < NUM_LEG; ++i)
    {
        double delta_x =
            std::sqrt(std::abs(state.default_foot_pos(2)) / 9.8) * (lin_vel_rel(0) - state.root_lin_vel_d(0)) +
            ((state.counter_per_swing / state.gait_counter_speed(i)) * state.control_dt) / 2.0 *
                state.root_lin_vel_d(0);
        double delta_y =
            std::sqrt(std::abs(state.default_foot_pos(2)) / 9.8) * (lin_vel_rel(1) - state.root_lin_vel_d(1)) +
            ((state.counter_per_swing / state.gait_counter_speed(i)) * state.control_dt) / 2.0 *
                state.root_lin_vel_d(1);

        if (delta_x < -FOOT_DELTA_X_LIMIT)
        {
            delta_x = -FOOT_DELTA_X_LIMIT;
        }
        if (delta_x > FOOT_DELTA_X_LIMIT)
        {
            delta_x = FOOT_DELTA_X_LIMIT;
        }
        if (delta_y < -FOOT_DELTA_Y_LIMIT)
        {
            delta_y = -FOOT_DELTA_Y_LIMIT;
        }
        if (delta_y > FOOT_DELTA_Y_LIMIT)
        {
            delta_y = FOOT_DELTA_Y_LIMIT;
        }

        state.foot_pos_target_rel(0, i) += delta_x;
        state.foot_pos_target_rel(1, i) += delta_y;

        state.foot_pos_target_abs.block<3, 1>(0, i) = state.root_rot_mat * state.foot_pos_target_rel.block<3, 1>(0, i);
        state.foot_pos_target_world.block<3, 1>(0, i) = state.foot_pos_target_abs.block<3, 1>(0, i) + state.root_pos;
    }
}

void RobotController::generate_swing_legs_ctrl(RobotStates &state, double dt) // swing force calculate
{
    state.joint_torques.setZero();

    // get current foot pos and target foot pose
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_cur;
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_cur;
    Eigen::Matrix<float, 1, NUM_LEG> spline_time;
    spline_time.setZero();
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target;
    foot_pos_target.setZero();
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_target;
    foot_vel_target.setZero();
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_error;
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_error;

    // the foot force of swing foot and stance foot, both are in robot frame
    Eigen::Matrix<double, 3, NUM_LEG> foot_forces_kin;
    Eigen::Matrix<double, 3, NUM_LEG> foot_forces_grf;

    for (int i = 0; i < NUM_LEG; ++i)
    {
        foot_pos_cur.block<3, 1>(0, i) = state.root_rot_mat_z.transpose() * state.foot_pos_abs.block<3, 1>(0, i);

        // from foot_pos_cur to foot_pos_final computes an intermediate point using BezierUtils // 베지어 곡선으로 다리
        // 궤적 부드럽게 계산
        if (state.gait_counter(i) <= state.counter_per_swing)
        {
            // stance foot
            spline_time(i) = 0.0;
            // in this case the foot should be stance
            // keep refreshing foot_pos_start in stance mode
            state.foot_pos_start.block<3, 1>(0, i) = foot_pos_cur.block<3, 1>(0, i);
        }
        else
        {
            // in this case the foot should be swing
            spline_time(i) = float(state.gait_counter(i) - state.counter_per_swing) / float(state.counter_per_swing);
        }

        foot_pos_target.block<3, 1>(0, i) = bezierUtils[i].get_foot_pos_curve(
            spline_time(i), state.foot_pos_start.block<3, 1>(0, i), state.foot_pos_target_rel.block<3, 1>(0, i), 0.0);

        foot_vel_cur.block<3, 1>(0, i) =
            (foot_pos_cur.block<3, 1>(0, i) - state.foot_pos_rel_last_time.block<3, 1>(0, i)) / dt;
        state.foot_pos_rel_last_time.block<3, 1>(0, i) = foot_pos_cur.block<3, 1>(0, i);

        foot_vel_target.block<3, 1>(0, i) =
            (foot_pos_target.block<3, 1>(0, i) - state.foot_pos_target_last_time.block<3, 1>(0, i)) / dt;
        state.foot_pos_target_last_time.block<3, 1>(0, i) = foot_pos_target.block<3, 1>(0, i);

        foot_pos_error.block<3, 1>(0, i) = foot_pos_target.block<3, 1>(0, i) - foot_pos_cur.block<3, 1>(0, i);
        foot_vel_error.block<3, 1>(0, i) = foot_vel_target.block<3, 1>(0, i) - foot_vel_cur.block<3, 1>(0, i);
        foot_forces_kin.block<3, 1>(0, i) =
            foot_pos_error.block<3, 1>(0, i).cwiseProduct(state.kp_foot.block<3, 1>(0, i)) +
            foot_vel_error.block<3, 1>(0, i).cwiseProduct(state.kd_foot.block<3, 1>(0, i));
    }
    state.foot_pos_cur = foot_pos_cur;

    // detect early contact
    bool last_contacts[NUM_LEG];

    for (int i = 0; i < NUM_LEG; ++i)
    {
        if (state.gait_counter(i) <= state.counter_per_swing * 1.5)
        {
            state.early_contacts[i] = false;
        }
        if (!state.plan_contacts[i] && (state.gait_counter(i) > state.counter_per_swing * 1.5) &&
            (state.foot_force(i) > FOOT_FORCE_LOW))
        {
            state.early_contacts[i] = true;
        }
        // actual contact
        last_contacts[i] = state.contacts[i];
        state.contacts[i] = state.plan_contacts[i] || state.early_contacts[i];

        // record recent contact position if the foot is in touch with the ground
        if (state.contacts[i])
        {

            state.foot_pos_recent_contact.block<3, 1>(0, i)
                << recent_contact_x_filter[i].CalculateAverage(state.foot_pos_abs(0, i)),
                recent_contact_y_filter[i].CalculateAverage(state.foot_pos_abs(1, i)),
                recent_contact_z_filter[i].CalculateAverage(state.foot_pos_abs(2, i));
        }
    }
    state.foot_forces_kin = foot_forces_kin;
}

void RobotController::compute_joint_torques(RobotStates &state)
{
    Eigen::Matrix<double, NUM_DOF, 1> joint_torques;
    joint_torques.setZero();
    mpc_init_counter++;
    // for the first 100 ticks, just return zero torques.
    if (mpc_init_counter < 100)
    {
        state.joint_torques = joint_torques;
    }
    else
    {
        // for each leg, if it is a swing leg (contact[i] is false), use foot_force_kin to get joint_torque
        // for each leg, if it is a stance leg (contact[i] is true), use foot_forces_grf to get joint_torque
        // gait counter에 의해 leg trajectory가 그려지고, 그 trajectory에 따라 이동, 공중에 뜬 trajectory일 때는 swing
        // leg force 제어, 땅에 닿아있을 때는 mpc 제어 gait에 의해 공중에 뜨는 다리와 그렇지 않은 다리 판별함
        for (int i = 0; i < NUM_LEG; ++i)
        {
            Eigen::Matrix3d jac = state.j_foot.block<3, 3>(3 * i, 3 * i);
            if (state.contacts[i])
            {
                // stance leg --> MPC
                joint_torques.segment<3>(i * 3) = jac.transpose() * -state.foot_forces_grf.block<3, 1>(0, i);
            }
            else
            {
                // swing leg --> swing leg control
                Eigen::Vector3d force_tgt = state.km_foot.cwiseProduct(state.foot_forces_kin.block<3, 1>(0, i));
                joint_torques.segment<3>(i * 3) = jac.transpose() * force_tgt; // jac * tau = F
                // std::cout << "force_tgt: " << force_tgt << std::endl;
            }
        }
        // gravity compensation
        joint_torques += state.torques_gravity;
        // Coriolis
        // joint_torques += state.Coriolis_torques;

        // prevent nan
        for (int i = 0; i < 12; ++i)
        {
            if (!isnan(joint_torques[i]))
                state.joint_torques[i] = joint_torques[i];
        }
    }
}

Eigen::Matrix<double, 3, NUM_LEG> RobotController::compute_grf(RobotStates &state, double dt)
{
    Eigen::Matrix<double, 3, NUM_LEG> foot_forces_grf;
    // first get parameters needed to construct the solver hessian and gradient
    // use euler angle to get desired angle
    Eigen::Vector3d euler_error = state.root_euler_d - state.root_euler;

    // limit euler error to pi/2
    if (euler_error(2) > 3.1415926 * 1.5)
    {
        euler_error(2) = state.root_euler_d(2) - 3.1415926 * 2 - state.root_euler(2);
    }
    else if (euler_error(2) < -3.1415926 * 1.5)
    {
        euler_error(2) = state.root_euler_d(2) + 3.1415926 * 2 - state.root_euler(2);
    }

    // surface adaptation
    Eigen::Vector3d surf_coef = compute_walking_surface(state); // 지형 coef 계산
    Eigen::Vector3d flat_ground_coef;
    flat_ground_coef << 0, 0, 1;
    double terrain_angle = 0;
    // only record terrain angle when the body is high
    if (state.root_pos[2] > 0.1)
    {
        terrain_angle = terrain_angle_filter.CalculateAverage(
            Utils::cal_dihedral_angle(flat_ground_coef, surf_coef)); // coef 사이 각도 계산
    }
    else
    {
        terrain_angle = 0;
    }

    if (terrain_angle > 0.5)
    {
        terrain_angle = 0.5;
    }
    if (terrain_angle < -0.5)
    {
        terrain_angle = -0.5;
    }

    // 전 후방 높이 차이 계산
    double F_R_diff = state.foot_pos_recent_contact(2, 0) + state.foot_pos_recent_contact(2, 1) -
                      state.foot_pos_recent_contact(2, 2) - state.foot_pos_recent_contact(2, 3);

    // root_euler_d에 terrain_angle 만큼 보정
    if (state.use_terrain_adapt)
    {
        if (F_R_diff > 0.05)
        {
            state.root_euler_d[1] = -terrain_angle;
        }
        else
        {
            state.root_euler_d[1] = terrain_angle;
        }
    }

    std_msgs::Float64 terrain_angle_msg;
    terrain_angle_msg.data = terrain_angle * (180 / 3.1415926);
    pub_terrain_angle.publish(terrain_angle_msg); // publish in deg
    // std::cout << "desire pitch in deg: " << state.root_euler_d[1] * (180 / 3.1415926) << std::endl;
    // std::cout << "terrain angle: " << terrain_angle << std::endl;

    // save calculated terrain pitch angle
    // TODO: limit terrain pitch angle to -30 to 30?
    state.terrain_pitch_angle = terrain_angle;

    // ConvexMPC
    ConvexMpc mpc_solver = ConvexMpc(state.q_weights, state.r_weights);
    mpc_solver.reset();

    // initialize the mpc state at the first time step
    // state.mpc_states.resize(13);
    state.mpc_states << state.root_euler[0], state.root_euler[1], state.root_euler[2], state.root_pos[0],
        state.root_pos[1], state.root_pos[2], state.root_ang_vel[0], state.root_ang_vel[1], state.root_ang_vel[2],
        state.root_lin_vel[0], state.root_lin_vel[1], state.root_lin_vel[2], -9.8;

    // previously we use dt passed by outer thread. It turns out that this dt is not stable on hardware.
    // if the thread is slowed down, dt becomes large, then MPC will output very large force and torque value
    // which will cause over current. Here we use a new mpc_dt, this should be roughly close to the average dt
    // of thread 1
    // double mpc_dt = 0.0025;

    // in simulation, use dt has no problem
    // if (use_sim_time == "true") {
    //     mpc_dt = dt;
    // }

    double mpc_dt = dt;

    // initialize the desired mpc states trajectory
    state.root_lin_vel_d_world = state.root_rot_mat * state.root_lin_vel_d;

    // 속도 명령
    for (int i = 0; i < PLAN_HORIZON; ++i)
    {
        state.mpc_states_d.segment(i * 13, 13) << state.root_euler_d[0], state.root_euler_d[1],
            state.root_euler[2] + state.root_ang_vel_d[2] * mpc_dt * (i + 1),
            state.root_pos[0] + state.root_lin_vel_d_world[0] * mpc_dt * (i + 1),
            state.root_pos[1] + state.root_lin_vel_d_world[1] * mpc_dt * (i + 1), state.root_pos_d[2],
            state.root_ang_vel_d[0], state.root_ang_vel_d[1], state.root_ang_vel_d[2], state.root_lin_vel_d_world[0],
            state.root_lin_vel_d_world[1], 0, -9.8;
    }

    // a single A_c is computed for the entire reference trajectory
    mpc_solver.calculate_A_c(state.root_euler);

    // for each point in the reference trajectory, an approximate B_c matrix is computed using desired values of euler
    // angles and feet positions from the reference trajectory and foot placement controller state.foot_pos_abs_mpc =
    // state.foot_pos_abs;
    for (int i = 0; i < PLAN_HORIZON; i++)
    {
        // calculate current B_c matrix
        mpc_solver.calculate_B_c(state.robot_mass, state.a1_trunk_inertia, state.root_rot_mat, state.foot_pos_abs);

        // state space discretization, calculate A_d and current B_d
        mpc_solver.state_space_discretization(mpc_dt);

        // store current B_d matrix
        mpc_solver.B_mat_d_list.block<13, 12>(i * 13, 0) = mpc_solver.B_mat_d;
    }

    // calculate QP matrices
    mpc_solver.calculate_mpc_matrix(state);

    qpOASES::int_t nV = NUM_DOF * PLAN_HORIZON;            // Number of variables
    qpOASES::int_t nC = MPC_CONSTRAINT_DIM * PLAN_HORIZON; // Number of constraints

    qpOASES::real_t H[nV * nV]; // Hessian matrix
    qpOASES::real_t A[nC * nV]; // Constraint matrix
    qpOASES::real_t g[nV];      // Gradient vector
    qpOASES::real_t lbA[nC];    // Lower bound for constraints
    qpOASES::real_t ubA[nC];    // Upper bound for constraints

    Eigen::Map<Eigen::Matrix<qpOASES::real_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(H, nV, nV) =
        mpc_solver.hessian;
    Eigen::Map<Eigen::Matrix<qpOASES::real_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(A, nC, nV) =
        mpc_solver.linear_constraints;
    Eigen::Map<Eigen::Matrix<qpOASES::real_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(g, nV, 1) =
        mpc_solver.gradient;
    Eigen::Map<Eigen::Matrix<qpOASES::real_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(lbA, nC, 1) =
        mpc_solver.lb;
    Eigen::Map<Eigen::Matrix<qpOASES::real_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(ubA, nC, 1) =
        mpc_solver.ub;

    // std::cout << " mpc_solver.hessian: " << std::endl << mpc_solver.hessian << std::endl;
    // std::cout << " mpc_solver.gradient: " << std::endl << mpc_solver.gradient << std::endl;

    qpOASES::SQProblem qp_solver(nV, nC);

    qpOASES::Options op;
    op.setToMPC();
    op.printLevel = qpOASES::PL_NONE;
    qp_solver.setOptions(op);

    // solve
    qpOASES::int_t nWSR = 1000; // Maximum number of working set recalculation

    int rval = qp_solver.init(H, g, A, NULL, NULL, lbA, ubA, nWSR);

    (void)rval;

    qpOASES::real_t QPsolution[nV];

    int rval2 = qp_solver.getPrimalSolution(QPsolution);

    Eigen::VectorXd solution = Eigen::Map<Eigen::VectorXd>(QPsolution, NUM_DOF * PLAN_HORIZON);

    for (int i = 0; i < NUM_LEG; ++i)
    {
        if (!isnan(solution.segment<3>(i * 3).norm()))
            foot_forces_grf.block<3, 1>(0, i) = state.root_rot_mat.transpose() * solution.segment<3>(i * 3);
    }

    return foot_forces_grf;
}

Eigen::Vector3d RobotController::compute_walking_surface(RobotStates &state) // walking surface modeling
{
    Eigen::Matrix<double, NUM_LEG, 3> W;
    Eigen::VectorXd foot_pos_z;
    Eigen::Vector3d a;
    Eigen::Vector3d surf_coef;

    W.block<NUM_LEG, 1>(0, 0).setOnes();
    W.block<NUM_LEG, 2>(0, 1) = state.foot_pos_recent_contact.block<2, NUM_LEG>(0, 0).transpose();

    foot_pos_z.resize(NUM_LEG);
    foot_pos_z = state.foot_pos_recent_contact.block<1, NUM_LEG>(2, 0).transpose();

    a = Utils::pseudo_inverse(W.transpose() * W) * W.transpose() * foot_pos_z;
    // surface: a1 * x + a2 * y - z + a0 = 0, coefficient vector: [a1, a2, -1]
    surf_coef << a[1], a[2], -1;
    return surf_coef;
}