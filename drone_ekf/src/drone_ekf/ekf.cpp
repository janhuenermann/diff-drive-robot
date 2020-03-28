#include <drone_ekf/ekf.hpp>

using namespace drone_ekf;
using namespace std;

template<typename System>
void EKF<System>::reset()
{
    running_ = false;
    last_step_time_ = ros::Time(0);
    system_.reset();
}

template<typename System>
void EKF<System>::step(const Input &u)
{
    const ros::Time now = ros::Time::now();
    const typename System::Transform &pred = system_.prediction;

    double dt = 0.0;

    if (running_)
    {
        dt = (now - last_step_time_).toSec();
        assert(dt > 0.0);
    }

    // save in order to have noalias operation
    __cov = state_.cov;

    // runs f(x, u) = system_.transform.x, gives us F and G (derivatives) in system_.transform
    system_.predict(u, dt);

    // sanity checks
    assert(u.cov.isApproxSymmetric());
    assert(pred.Q.isApproxSymmetric());

    // update state by copying over data and calculating new cov
    state_.mean.noalias() = pred.x;
    state_.cov.noalias() = pred.F * __cov * pred.F.transpose() + pred.G * u.cov * pred.G.transpose() + pred.Q;

    // sanity checks (important!)
    assert(state_.cov.isApproxSymmetric());

    running_ = true;
    last_step_time_ = now;
}

template<typename System>
template<typename Measurement>
void EKF<System>::correct(const Measurement &Z)
{
    // static_assert(is_measurement_class<drone_ekf::Measurement, Measurement>{});

    const typename Measurement::Transform &pred = Z.prediction;
    const typename Measurement::Correction &corr = Z.correction;
    const StateCov I = State::StateCov::Identity();

    // save in order to have noalias operation
    __mean = state_.mean;
    __cov = state_.cov;

    typename Measurement::StateType st = Measurement::StateType::construct(state_);

    // transform state to predicted measurement
    Z.predict(st);

    // sanity check
    assert(pred.R.isApproxSymmetric());

    corr.y.noalias() = Z.mean - pred.zhat;
    corr.S.noalias() = pred.H * st.cov * pred.H.transpose() + Z.cov;
    corr.K.noalias() = st.cov * pred.H.transpose() * corr.S.inverse();

    #ifndef NDEBUG
    if (corr.S.determinant() < 1e-2)
    {
        ROS_WARN("det S very small");
    }
    #endif

    // apply correction
    st.meanRef().noalias() = __mean + corr.K * corr.y;
    st.covRef().noalias()  = (I - corr.K * pred.H) * __cov;

}
