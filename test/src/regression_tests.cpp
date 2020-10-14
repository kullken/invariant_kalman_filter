#include <iostream>
#include <string>

#include <gtest/gtest.h>

#include "iekf.h"
#include "mekf.h"

#include "accuracy_test.h"
#include "accuracy_test_config.h"

namespace invariant::test
{

template<typename FilterType>
class RegressionTest: public AccuracyTest<FilterType>
{
private:
    static constexpr int kNumTestRuns      = 10;
    static constexpr int kNumOffsetSamples = 1;

protected:
    void run_test()
    {
        double position_rmse{0};
        double velocity_rmse{0};
        double rotation_rmse{0};

        for (int i = 0; i < kNumTestRuns; ++i)
        {
            const auto sensor_events = generate_events(this->trajectory_, this->sensors_);
            for (int j = 0; j < kNumOffsetSamples; ++j)
            {
                const auto result = this->compute_accuracy(sensor_events);
                position_rmse += result.position_rmse;
                velocity_rmse += result.velocity_rmse;
                rotation_rmse += result.rotation_rmse;
            }
        }

        position_rmse /= kNumTestRuns * kNumOffsetSamples;
        velocity_rmse /= kNumTestRuns * kNumOffsetSamples;
        rotation_rmse /= kNumTestRuns * kNumOffsetSamples;

        this->RecordProperty("PositionRMSE", std::to_string(position_rmse));
        this->RecordProperty("VelocityRMSE", std::to_string(velocity_rmse));
        this->RecordProperty("RotationRMSE", std::to_string(rotation_rmse));

        std::cout << "position_rmse : " << position_rmse << '\n';
        std::cout << "velocity_rmse : " << velocity_rmse << '\n';
        std::cout << "rotation_rmse : " << rotation_rmse << '\n';
    }
};

using IekfTestSuite = RegressionTest<IEKF>;
TEST_P(IekfTestSuite, IekfTestCase) { run_test(); }
INSTANTIATE_TEST_CASE_P(
    RegressionTests,
    IekfTestSuite,
    test_configs_full,
);

using MekfTestSuite = RegressionTest<MEKF>;
TEST_P(MekfTestSuite, MekfTestCase) { run_test(); }
INSTANTIATE_TEST_CASE_P(
    RegressionTests,
    MekfTestSuite,
    test_configs_full,
);

} // namespace invariant::test

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
