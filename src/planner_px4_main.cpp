#include <sys/ipc.h>
#include <sys/shm.h>

#include "numeric"

#include "map/map.hpp"
#include "map/sdf.hpp"
#include "ros_interface/ros_interface.hpp"
#include "kinodynamic_astar/kinodynamic_astar.hpp"
#include "bspline/uniform_bspline.hpp"
#include "bspline_opt/bspline_optimizer.hpp"
#include "bspline_opt/bspline_optimizer2.hpp"
#include "px4_interface/px4_interface.hpp"
#include "mpcc/nominal_mpcc.hpp"
// #include "quadrotor_dynamics/quad_simulator.hpp"
#include "common/rotation_math.hpp"
#include "disturbance_observer/gpiobserver.hpp"
#include "logger/logger.hpp"
#include "tracker/pid_tracker.hpp"
#include "utils.hpp"

constexpr int n_step = NominalMpcc::_n_step;

int main(int argc, char **argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "planner_px4");
    ros::NodeHandle nh("~");

    // Set the CPU affinity (if necessary)
    cpu_set_t mask;
    CPU_ZERO(&mask);
    CPU_SET(5, &mask);
    pthread_setaffinity_np(pthread_self(), sizeof(mask), &mask);

    // Get parameters from the ROS parameter server
    bool enable_dob, enable_mpcc;
    double MAX_VEL, MAX_ACC, mu;

    nh.getParam("enable_dob", enable_dob);
    nh.getParam("enable_mpcc", enable_mpcc);
    nh.getParam("MAX_VEL", MAX_VEL);
    nh.getParam("MAX_ACC", MAX_ACC);
    nh.getParam("mu", mu);

    Logger logger(string(enable_dob == true ? "enable-DOB" : "disable-DOB") + "-"
        + string(enable_mpcc == true ? "enable-MPCC" : "disable-MPCC") + "-test");

    key_t key;
	int shmId;
	int i = 0;
	key = 102988;
    cout << "key=" << key << endl;
	shmId = shmget(key, 0, 0);
	cout << "shmId=" << shmId << endl;
	if(shmId == -1)
	{
		cout << "shmget failed" << endl;
	}

    //PX4 interface
    Px4Interface px4("");
    // QuadSimulator px4(0.309);

    // TODO: construct map Ptr?
    GridMap gridmap;
    gridmap.update_grid_map();

    //ros interface
    RosInterface ros_inte(gridmap.size());
    sleep(1);
    ros_inte.publish_grid_map(gridmap);
    
    //construct sdf map
    SdfMap sdfmap(gridmap.resolution() / 1.0, gridmap);
    ros_inte.publish_sdf_map(sdfmap);

    //global planner
    KinodynamicAstar kino_astar(gridmap);
    kino_astar.set_param(
        10., //w_time 2.5
        MAX_VEL,  //max_vel
        MAX_ACC,  //max_acc
        1.0 + 1.0 / 10000, //tie_breaker
        1 / 2.0,  //acc_resolution
        1 / 1.0,      //time_resolution
        0.6,      //max_duration
        1 / 50.,   //safety_check_res
        5.0);     //lamda_heu
        // TODO: set the start and goal positions
    auto start_pos = Vector3d(1., 5., 1.0);
    // TODO: set the goal position
    auto goal_pos = Vector3d(49.0, 5.0, 1.0);
    kino_astar.search(
        start_pos,
        Vector3d(0., 0., 0.),
        goal_pos,
        Vector3d(0., 0., 0.));
    double t_sample = 0.05;
    auto kino_path = kino_astar.get_sample_path(t_sample);
    kino_path.first.push_back(goal_pos);
    ros_inte.publish_kino_traj(kino_path.first);

    //trajectory parameterize
    vector<Vector3d> vels, accs;
    vels.push_back(Vector3d::Zero());
    vels.push_back(Vector3d::Zero());
    accs.push_back(Vector3d::Zero());
    accs.push_back(Vector3d::Zero());
    MatrixXd ctrl_pts;
    UniformBspline::parameter2Bspline(t_sample, kino_path.first, vels, accs, ctrl_pts);

    //B-spline trajectory optimization
    BsplineOptimizer optimizer;
    vector<tuple<double, double, double, double, double, double>> cost_history;
    
    if (enable_mpcc) {
        optimizer.optimize(ctrl_pts, start_pos,
            Vector3d(0, 0, 0), Vector3d(0, 0, 0), goal_pos, 
            Vector3d(0, 0, 0), Vector3d(0, 0, 0), &sdfmap, t_sample, 
            100.0,  //lambda_s
            100.0,   //lambda_c
            0.1,    //lambda_v
            0.1,    //lambda_a
            0.0,    //lambda_l
            0.1,      //lambda_dl 0.1
            100,   //lambda_ep
            0,   //lambda_ev 100
            0,   //lambda_ea 10
            0.4,   //risk_dis
            MAX_VEL,    //vel_max
            MAX_ACC,    //acc_max 
            cost_history);
    }
    else{
        //t_sample *= 1.5;
        optimizer.optimize(ctrl_pts, start_pos,
            Vector3d(0, 0, 0), Vector3d(0, 0, 0), goal_pos, 
            Vector3d(0, 0, 0), Vector3d(0, 0, 0), &sdfmap, t_sample,  
            100.0,  //lambda_s
            100.0,   //lambda_c
            0.1,    //lambda_v
            0.1,    //lambda_a
            0.0,    //lambda_l
            0.0,      //lambda_dl 0.1
            100,   //lambda_ep
            100,   //lambda_ev 100
            10,   //lambda_ea 10
            0.4,   //risk_dis
            MAX_VEL,    //vel_max
            MAX_ACC,    //acc_max 
            cost_history);
    }
    auto v_ctrl_pts = UniformBspline::getDerivativeCtrlPts(ctrl_pts, t_sample);
    auto a_ctrl_pts = UniformBspline::getDerivativeCtrlPts(v_ctrl_pts, t_sample);
    vector<Vector3d> opt_bsp_p, opt_bsp_v, opt_bsp_a;
    vector<double> opt_t_list;
    for (double t = 3 * t_sample; t < ctrl_pts.rows() * t_sample + 1e-6; t += t_sample * 1) {
        auto p = UniformBspline::getBsplineValue(t_sample, ctrl_pts, t, 3);
        auto v = UniformBspline::getBsplineValue(t_sample, v_ctrl_pts, t - t_sample, 2);
        auto a = UniformBspline::getBsplineValue(t_sample, a_ctrl_pts, t - t_sample - t_sample, 1);
        opt_bsp_p.push_back(p);
        opt_bsp_v.push_back(v);
        opt_bsp_a.push_back(a);
        opt_t_list.push_back(t);
    }
    ros_inte.publish_bspline_traj(opt_bsp_p);

    cout << "B-spline duration: " << ctrl_pts.rows() * t_sample - 3 * t_sample << " s" << endl;

    vector<double> arc_lengths, arc_t;
    auto start_t = chrono::steady_clock::now();
    double total_len = 0.0;
    double dt = 0.001;
    for (double t = 2 * t_sample; t < v_ctrl_pts.rows() * t_sample; t += dt) {
        total_len += UniformBspline::getBsplineValueFast<Vector3d>(t_sample, v_ctrl_pts, t, 2).norm() * dt;
        arc_lengths.push_back(total_len);
        arc_t.push_back(t - 2 * t_sample);
    }
    cout << "Total len: " << total_len << " m" << endl;
    auto spend = chrono::duration<double>(chrono::steady_clock::now() - start_t).count();

    logger.length_ = total_len;

    SdfMap sdfmap2(gridmap.resolution() / 1.0, gridmap);
    ros_inte.publish_sdf_map(sdfmap2);
    
    sleep(2);

    px4.set_px4_mode("AUTO.LOITER");
    sleep(1);
    px4.arm();
    sleep(1);
    px4.set_px4_mode("OFFBOARD");

    ros::Rate rate(50);

    vector<DynObs> dynobs;

    for (double p = 5; p < 45; p += 2.0) {
        dynobs.push_back(DynObs(0.2, 1.0, Vector2d(p, 3), Vector2d(p, 7)));
    }

    auto t0 = chrono::steady_clock::now();

    auto start_time = ros::Time::now();
    while((ros::Time::now() - start_time).toSec() < 6.0 && ros::ok()) {
        Vector3d pos = px4.pos();
        Vector3d vel = px4.vel();
        Vector4d quat = px4.quat();
        ros_inte.publish_quadmesh(pos, quat);
        auto p = UniformBspline::getBsplineValue(t_sample, ctrl_pts, 3 * t_sample + 1e-6, 3);
        px4.set_pos(p[0], p[1], p[2], 1.57);
        rate.sleep();
        for (auto &o : dynobs) {
            o.update(0.02);
        }
        ros_inte.publish_dyn_obs(dynobs);
        ros::spinOnce();
    }

    double omega = 14;
    double l1 = 3 * omega - 1;
    double l2 = 3 * omega * omega - 1;
    double l3 = omega * omega * omega;
    double omega_z = 20;
    double l1_z = 3 * omega_z - 1;
    double l2_z = 3 * omega_z * omega_z - 1;
    double l3_z = omega_z * omega_z * omega_z;
    GPIObserver dob(Vector3d(l1, l1, l1_z), Vector3d(l2, l2, l2_z), Vector3d(l3, l3, l3_z));
    PidTracker pidtracker(Vector3d(2.5, 2.5, 2.5), Vector3d(2.5, 2.5, 2.5));
    NominalMpcc nominal_mpcc(0.309, "LD_AUGLAG", 300);
    Matrix<double, 3 + NominalMpcc::u_dim_ + 1 + NominalMpcc::u_dim_ + 1, 1> cost_w;
    cost_w << 0.5, 50.0     //weight of contouring error and lag error
            , mu * total_len / (ctrl_pts.rows() * t_sample - 3 * t_sample) //weight of progress
            , 0.0000, 0.0000, 0.005, 0.1    //weight of control input
            , 0     //weight of the cost of violating distance constraints
            , 0.2, 0.2, 0.2, 200.0  //weight of the difference of control input
            , 75;   //weight of the cost of violating CBF constraints
    nominal_mpcc.set_w(cost_w);
    Matrix<double, NominalMpcc::x_dim_, 1> state;
    Matrix<double, n_step, NominalMpcc::u_dim_> u;
    Matrix<double, n_step, NominalMpcc::x_dim_> x_predict;
    Matrix<double, n_step + 1, 1> t_index;
    t_index[0] = t_sample * 3;
    for (int k = 0; k < n_step; k++) {
        t_index[k + 1] = t_index[k] + 0.05;
    }
    for (int k = 0; k < n_step; k++) {
        u.row(k) << 0., 0., 0., 0.309;
    }
    vector<double> solve_times;
    vector<double> vel_norm, sim_vel_norm, acc_list, sim_acc_list, t_list, r_list, p_list, y_list, tilt_list, rr_list, pr_list, yr_list, thr_list;
    vector<double> acc_x_list, acc_y_list, acc_z_list, sim_acc_x_list, sim_acc_y_list, sim_acc_z_list;
    vector<double> real_rr_list, real_pr_list, real_yr_list;
    vector<double> acc_x_err_list, acc_y_err_list, acc_z_err_list, vel_x_list, vel_y_list, vel_z_list;
    vector<double> sim_vel_x_list, sim_vel_y_list, sim_vel_z_list;
    vector<double> dob_vx_list, dob_vy_list, dob_vz_list, dob_dx_list, dob_dy_list, dob_dz_list;
    vector<double> nominal_acc_x_err_list, nominal_acc_y_err_list, nominal_acc_z_err_list;
    vector<Vector3d> collision_pos;
    double mean_acc_norm_err = 0.0, mean_nominal_acc_norm_err = 0.0;
    double mean_v_norm_err = 0.0, mean_nominal_v_norm_err = 0.0;
    double mean_p_err = 0.0, mean_nominal_p_err = 0.0;
    vector<Vector3d> mpcc_traj;
    vector<double> dis_to_obs, dis_to_obs_sim, dis_to_obs_sim2;
    int reach_cnt = 1 / 0.02;
#if USE_EXTENDED_DYNAMICS
    ExtendedQuadDynamic quaddynamic(0.309);
#else
    NominalQuadDynamic quaddynamic(0.309);
#endif
    Eigen::Matrix<double, NominalMpcc::x_dim_, 1> sim_xdot, sim_state1, sim_state2, nominal_sim_xdot, nominal_sim_state1;
    sim_xdot.setZero();
    sim_state1.setZero();
    start_time = ros::Time::now();
    VectorXd u_opt(4);
    u_opt.setZero();
    u_opt(3) = 0.309;
    double solve_t;
    Vector3d acc_r(0, 0, 0);
    int loop_cnt = 0;
    ros::Time past_vel_stamp = ros::Time::now();
    while (ros::ok()) {
        // cout << "new loop" << endl;
        Vector3d pos = px4.pos();
        Vector3d vel = px4.vel();
        Vector3d acc_b = px4.acc();
        Vector4d quat = px4.quat();
        Vector3d ang_rate = px4.rate();
        ros::Time vel_stamp = px4.vel_stamp();
#if USE_EXTENDED_DYNAMICS
        state << pos, vel, quat.w(), quat.x(), quat.y(), quat.z(), u_opt(3);
#else
        state << pos, vel, quat.w(), quat.x(), quat.y(), quat.z();
#endif

        // quaddynamic.xdot_func(state, u_opt, sim_xdot);

        Vector3d ang = quaternion_to_rpy(Quaterniond(quat.w(), quat.x(), quat.y(), quat.z()));
        auto b_e_matrix = Quaterniond(quat.w(), quat.x(), quat.y(), quat.z()).toRotationMatrix();
        Vector3d acc = b_e_matrix * acc_b - Vector3d(0, 0, 9.81);

        dob.update(vel, b_e_matrix, u_opt[3] * 9.81 / 0.309/*1.084e-5 * pow(u_opt[3] * 1000 + 100, 2) * 4 / 0.74*/, (vel_stamp - past_vel_stamp).toSec());
        // cout << vel.transpose() << " " << (vel_stamp - past_vel_stamp).toSec() << " " << dob.get_dhat().transpose() << " " << dob.get_dhat().norm() << endl;
        dob_vx_list.push_back(dob.get_vhat().x());
        dob_vy_list.push_back(dob.get_vhat().y());
        dob_vz_list.push_back(dob.get_vhat().z());
        dob_dx_list.push_back(dob.get_dhat().x());
        dob_dy_list.push_back(dob.get_dhat().y());
        dob_dz_list.push_back(dob.get_dhat().z());

        dis_to_obs.push_back(min(sdfmap.get_dist_with_grad_trilinear(pos).first, get_dis_to_dynobs(dynobs, pos)));
        dis_to_obs_sim.push_back(sdfmap.get_dist_with_grad_trilinear(Vector3d(sim_state1.block(0, 0, 3, 1))).first);
        dis_to_obs_sim2.push_back(sdfmap.get_dist_with_grad_trilinear(Vector3d(sim_state2.block(0, 0, 3, 1))).first);
        if (sdfmap.get_dist_with_grad_trilinear(pos).first < 0.3) {
            collision_pos.push_back(pos);
        }

        mpcc_traj.push_back(pos);
        vel_norm.push_back(vel.norm());
        sim_vel_norm.push_back(sim_state1.block(3, 0, 3, 1).norm());
        mean_v_norm_err += fabs((vel - sim_state1.block(3, 0, 3, 1)).norm());
        mean_nominal_v_norm_err += fabs((vel - nominal_sim_state1.block(3, 0, 3, 1)).norm());
        mean_p_err += fabs((pos - sim_state1.block(0, 0, 3, 1)).norm());
        mean_nominal_p_err += fabs((pos - nominal_sim_state1.block(0, 0, 3, 1)).norm());
        // cout << (pos - sim_state1.block(0, 0, 3, 1)).norm() << " " << (pos - nominal_sim_state1.block(0, 0, 3, 1)).norm() << endl;
        acc_list.push_back(acc.norm());
        acc_x_list.push_back(acc.x());
        acc_y_list.push_back(acc.y());
        acc_z_list.push_back(acc.z());
        Vector3d sim_acc = sim_xdot.block(3, 0, 3, 1);
        Vector3d nominal_sim_acc = nominal_sim_xdot.block(3, 0, 3, 1);
        sim_acc_list.push_back(sim_acc.norm());
        sim_acc_x_list.push_back(sim_acc.x());
        sim_acc_y_list.push_back(sim_acc.y());
        sim_acc_z_list.push_back(sim_acc.z());
        mean_acc_norm_err += fabs((acc - sim_acc).norm());
        mean_nominal_acc_norm_err += fabs((acc - nominal_sim_acc).norm());
        acc_x_err_list.push_back((acc - sim_acc).x());
        acc_y_err_list.push_back((acc - sim_acc).y());
        acc_z_err_list.push_back((acc - sim_acc).z());
        nominal_acc_x_err_list.push_back((acc - nominal_sim_acc).x());
        nominal_acc_y_err_list.push_back((acc - nominal_sim_acc).y());
        nominal_acc_z_err_list.push_back((acc - nominal_sim_acc).z());
        vel_x_list.push_back(vel.x());
        vel_y_list.push_back(vel.y());
        vel_z_list.push_back(vel.z());
        t_list.push_back((ros::Time::now() - start_time).toSec());
        r_list.push_back(ang.x() * 180.0 / M_PI);
        p_list.push_back(ang.y() * 180.0 / M_PI);
        y_list.push_back(ang.z() * 180.0 / M_PI);
        tilt_list.push_back(acos(cos(ang.x())*cos(ang.y())) * 180.0 / M_PI);
        rr_list.push_back(u_opt(0) * 180.0 / M_PI);
        pr_list.push_back(u_opt(1) * 180.0 / M_PI);
        yr_list.push_back(u_opt(2) * 180.0 / M_PI);
        thr_list.push_back(u_opt(3));
        real_rr_list.push_back(ang_rate(0) * 180.0 / M_PI);
        real_pr_list.push_back(ang_rate(1) * 180.0 / M_PI);
        real_yr_list.push_back(ang_rate(2) * 180.0 / M_PI);

        logger.pos_ = pos;
        logger.vel_ = vel;
        logger.acc_ = acc;
        logger.att_ = ang;
        logger.rate_ = ang_rate;
        logger.tilt_ = acos(cos(ang.x())*cos(ang.y())) * 180.0 / M_PI;
        logger.disturbance_ = dob.get_dhat();
        logger.u_ = u_opt;
        logger.dis_to_obs_ = min(sdfmap.get_dist_with_grad_trilinear(pos).first, get_dis_to_dynobs(dynobs, pos));
        logger.solution_time_ = solve_t;
        logger.time_ = (ros::Time::now() - start_time).toSec();

        auto time_ = chrono::steady_clock::now();
        //global planner
        kino_astar.set_param(
            10., //w_time 2.5
            MAX_VEL,  //max_vel
            MAX_ACC,  //max_acc
            1.0 + 1.0 / 10000, //tie_breaker
            1 / 2.0,  //acc_resolution
            1 / 1.0,      //time_resolution
            0.6,      //max_duration
            1 / 50.,   //safety_check_res
            5.0);     //lamda_heu
        Vector3d vel_limed;
        vel_limed = vel / (vel - Vector3d(MAX_VEL, MAX_VEL, MAX_VEL)).cwiseAbs().maxCoeff() * MAX_VEL;
        kino_astar.search(
            pos,
            vel_limed,
            goal_pos,
            Vector3d(0., 0., 0.),
            &dynobs);
        t_sample = 0.2;
        auto kino_path = kino_astar.get_sample_path(t_sample);
        kino_path.first.push_back(goal_pos);
        ros_inte.publish_kino_traj(kino_path.first);

        //trajectory parameterization
        vector<Vector3d> vels, accs;
        vels.push_back(Vector3d::Zero());
        vels.push_back(Vector3d::Zero());
        accs.push_back(Vector3d::Zero());
        accs.push_back(Vector3d::Zero());
        UniformBspline::parameter2Bspline(t_sample, kino_path.first, vels, accs, ctrl_pts);

        //B-spline trajectory optimization
        BsplineOptimizer optimizer;
        vector<tuple<double, double, double, double, double, double>> cost_history;
        if (ctrl_pts.size() <= 12) {
            break;
        }
        MatrixXd new_ctrl_pts;
        Vector3d tmp_goal = goal_pos;
        if (ctrl_pts.rows() > 10) {
            new_ctrl_pts = ctrl_pts.block(0, 0, 10, ctrl_pts.cols());
            ctrl_pts = new_ctrl_pts;
            tmp_goal = UniformBspline::getBsplineValue(t_sample, ctrl_pts, ctrl_pts.rows() * t_sample - 1e-3, 3);
        }
        optimizer.optimize(ctrl_pts, pos,
            vel, acc_r, tmp_goal,
            Vector3d(0, 0, 0), Vector3d(0, 0, 0), &sdfmap, t_sample,
            100.0,  //lambda_s
            5000.0,   //lambda_c
            10.,    //lambda_v
            0.1,    //lambda_a
            0.0,    //lambda_l
            0.0,      //lambda_dl 0.1
            10000,   //lambda_ep
            100,   //lambda_ev 100
            0,   //lambda_ea 10
            0.4,   //risk_dis
            MAX_VEL,    //vel_max
            MAX_ACC,    //acc_max
            cost_history,
            &dynobs);
        v_ctrl_pts = UniformBspline::getDerivativeCtrlPts(ctrl_pts, t_sample);
        a_ctrl_pts = UniformBspline::getDerivativeCtrlPts(v_ctrl_pts, t_sample);
        vector<Vector3d> opt_bsp_p, opt_bsp_v, opt_bsp_a;
        vector<double> opt_t_list;
        for (double t = 3 * t_sample; t < ctrl_pts.rows() * t_sample + 1e-6; t += t_sample * 1) {
            auto p = UniformBspline::getBsplineValue(t_sample, ctrl_pts, t, 3);
            auto v = UniformBspline::getBsplineValue(t_sample, v_ctrl_pts, t - t_sample, 2);
            auto a = UniformBspline::getBsplineValue(t_sample, a_ctrl_pts, t - t_sample - t_sample, 1);
            opt_bsp_p.push_back(p);
            opt_bsp_v.push_back(v);
            opt_bsp_a.push_back(a);
            opt_t_list.push_back(t);
        }
        ros_inte.publish_bspline_traj(opt_bsp_p);
        double spend = chrono::duration<double>(chrono::steady_clock::now() - time_).count();
        // cout << "spend: " << spend * 1e3 << " ms" << endl;
        solve_times.push_back(spend);


        if (enable_mpcc) {
            //MPCC
            auto ret = nominal_mpcc.solve(state,
                sdfmap2, ctrl_pts,
                v_ctrl_pts, a_ctrl_pts,
                t_sample,
                total_len,
                u_opt,
                (enable_dob && dob.get_dhat().norm() < 15) ? dob.get_dhat() : Vector3d(0., 0., 0.),
                dynobs,
                u, x_predict,
                t_index, solve_t);

            vector<Vector3d> predict_traj;
            predict_traj.push_back(pos);
            for (int i = 0; i < n_step; i++) {
                predict_traj.push_back(x_predict.block(i, 0, 1, 3).transpose());
            }
            ros_inte.publish_predict_traj(predict_traj);

            // cout << t_index.transpose() << endl;
            // cout << ctrl_pts.rows() * t_sample << endl;

            // cout << "u:" << endl << u << endl;
            // cout << "x_predict:" << endl << x_predict << endl;
            u_opt = u.row(0);
#if USE_EXTENDED_DYNAMICS
            u_opt(3) = x_predict(0, 10);
#endif
            solve_times.push_back(solve_t);

            px4.set_rate_with_trust(u_opt(0), u_opt(1), u_opt(2), u_opt(3));
        } else {
            solve_times.push_back(0);
            double t = loop_cnt * 0.02;
                t = 0.02;

            if (t > t_sample * (ctrl_pts.rows() - 3)) {
                t = t_sample * (ctrl_pts.rows() - 3);
                if (reach_cnt-- < 0)
                    break;
            }
            Vector3d pd = UniformBspline::getBsplineValueFast<Vector3d>(t_sample, ctrl_pts, t + t_sample * 3, 3);
            Vector3d vd = UniformBspline::getBsplineValueFast<Vector3d>(t_sample, v_ctrl_pts, t + t_sample * 2, 2);
            Vector3d ad = UniformBspline::getBsplineValueFast<Vector3d>(t_sample, a_ctrl_pts, t + t_sample * 1, 1);
            acc_r = ad;
            auto ret = pidtracker.calculate_control(
                pd, vd, ad, pos, vel, Quaterniond(quat.w(), quat.x(), quat.y(), quat.z())
            );
            px4.set_attitude_with_trust(ret.first, ret.second);
            pidtracker.estimateThrustModel(acc);
        }

        // break;

        quaddynamic.xdot_func(state, u_opt, dob.get_dhat(), sim_xdot);
        quaddynamic.rk4_func(state, u_opt, dob.get_dhat(), 0.02, sim_state1);
        quaddynamic.rk4_func(state, u_opt, dob.get_dhat(), 0.1, sim_state2);

        quaddynamic.xdot_func(state, u_opt, Vector3d(0, 0, 0), nominal_sim_xdot);
        quaddynamic.rk4_func(state, u_opt, Vector3d(0, 0, 0), 0.02, nominal_sim_state1);

        ros_inte.publish_quadmesh(pos, quat);
        ros_inte.publish_mpcc_traj(mpcc_traj);
        // ros_inte.publish_collision(collision_pos);

        for (auto &o : dynobs) {
            o.update((vel_stamp - past_vel_stamp).toSec());
            // o.update(0.02);
        }
        ros_inte.publish_dyn_obs(dynobs);

        logger.update();

        // rate.sleep();
        past_vel_stamp = vel_stamp;
        ros::spinOnce();

        loop_cnt++;

        if (t_index[0] - ctrl_pts.rows() * t_sample > -1e-1) {
            if (reach_cnt-- < 0)
                break;
        }
        for (int i = 1; i < t_index.size(); i++)
            t_index[i] += 0.02;
        // break;
    }

    mean_acc_norm_err /= loop_cnt;
    mean_nominal_acc_norm_err /= loop_cnt;
    mean_v_norm_err /= loop_cnt;
    mean_nominal_v_norm_err /= loop_cnt;
    mean_p_err /= loop_cnt;
    mean_nominal_p_err /= loop_cnt;

    start_time = ros::Time::now();
    while((ros::Time::now() - start_time).toSec() < 8.0 && ros::ok()) {
        Vector3d pos = px4.pos();
        Vector3d vel = px4.vel();
        Vector4d quat = px4.quat();
        ros_inte.publish_quadmesh(pos, quat);
        auto p = UniformBspline::getBsplineValue(t_sample, ctrl_pts, 3 * t_sample + 1e-6, 3);
        px4.set_pos(p[0], p[1], p[2], 1.57);
        rate.sleep();
        ros::spinOnce();
    }

    px4.set_px4_mode("POSCTL");

    ros_inte.publish_mpcc_traj(mpcc_traj);

    vector<double> v_x, v_y, v_z, v_norm, a_x, a_y, a_z, a_norm;
    t_list.clear();
    for (double t = 3 * t_sample; t < ctrl_pts.rows() * t_sample + 1e-6; t += t_sample * 0.1) {
        auto p = UniformBspline::getBsplineValue(t_sample, ctrl_pts, t, 3);
        auto v = UniformBspline::getBsplineValue(t_sample, v_ctrl_pts, t - t_sample, 2);
        auto a = UniformBspline::getBsplineValue(t_sample, a_ctrl_pts, t - t_sample - t_sample, 1);
        t_list.push_back(t - 3 * t_sample);
        v_x.push_back(v.x());
        v_y.push_back(v.y());
        v_z.push_back(v.z());
        v_norm.push_back(v.norm());
        a_x.push_back(a.x());
        a_y.push_back(a.y());
        a_z.push_back(a.z());
        a_norm.push_back(a.norm());
    }

    return 0;
}
