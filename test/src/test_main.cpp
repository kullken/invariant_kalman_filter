#include <gtest/gtest.h>

#include <ugl/math/vector.h>
#include <ugl/trajectory/trajectory.h>

#include "accuracy_test.h"

namespace invariant::test
{

auto test_trajectories = testing::Values(
        AccuracyTestConfig{"TestConfig_1", {} },
        AccuracyTestConfig{"TestConfig_2", {} }
        );

TEST_P(AccuracyTestParam, noNoiseTest)
{
    AccuracyResult result = compute_accuracy(iekf_, trajectory_);
    RecordProperty("Accuracy", result.final_accuracy);
}

INSTANTIATE_TEST_CASE_P(
        AccuracyTestInstantiation,
        AccuracyTestParam,
        test_trajectories,
        [](const testing::TestParamInfo<AccuracyTestParam::ParamType>& info) { return info.param.name; }
        );

namespace
{

class FooTest : public ::testing::Test
{
protected:
    FooTest() = default;

    ugl::Vector3 a{1, 2, 3};
    ugl::Vector3 b{1, 2, 3};
};

// Tests that the Foo::Bar() method does Abc.
TEST_F(FooTest, MethodBarDoesAbc)
{
    EXPECT_EQ(a, b);
}

} // namespace
} // namespace invariant::test

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
