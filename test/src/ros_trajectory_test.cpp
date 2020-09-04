#include <ros/ros.h>

#include "accuracy_test.h"
#include "accuracy_test_config.h"
#include "mock_trajectory_node.h"

namespace invariant::test
{

class RosIekfTestSuite : public AccuracyTest<invariant::IEKF>
{
protected:
    RosIekfTestSuite()
        : AccuracyTest()
        , mock_trajectory_node_(trajectory_, imu_, mocap_)
    {
        // TODO: Set initial values to kalman filter through ROS parameters.
        // TODO: Reset kalman_node through service call.
        mock_trajectory_node_.start();
    }

    ~RosIekfTestSuite() override
    {
        // TODO: Reset internal error tracking.
    }

public:
    static void SetUpTestSuite()
    {
        // TODO: Initialise reset service client.
    }

    static void TearDownTestSuite()
    {

    }

protected:
    MockTrajectoryNode mock_trajectory_node_;
    static ros::ServiceClient reset_kalman_service_;
};

ros::ServiceClient RosIekfTestSuite::reset_kalman_service_{};

TEST_P(RosIekfTestSuite, IekfTestCase)
{
    ros::Duration(trajectory_.duration()).sleep();
    mock_trajectory_node_.stop();
    const auto result = mock_trajectory_node_.get_result();

    RecordProperty("PositionRMSE", std::to_string(result.position_rmse));
    RecordProperty("VelocityRMSE", std::to_string(result.velocity_rmse));
    RecordProperty("RotationRMSE", std::to_string(result.rotation_rmse));

    std::cout << "position_rmse : " << result.position_rmse << '\n';
    std::cout << "velocity_rmse : " << result.velocity_rmse << '\n';
    std::cout << "rotation_rmse : " << result.rotation_rmse << '\n';
}

INSTANTIATE_TEST_CASE_P(
    AccuracyTestBase,
    RosIekfTestSuite,
    test_configs_full,
);

} // namespace invariant::test

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "trajectory_test_node");
    return RUN_ALL_TESTS();
}
