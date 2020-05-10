#include <gtest/gtest.h>

#include <ugl/math/vector.h>

namespace invariant::test
{
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
