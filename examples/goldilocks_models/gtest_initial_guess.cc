#include "gtest/gtest.h"
#include "examples/goldilocks_models/initial_guess.h"

namespace dairlib::goldilocks_models{

class InitialGuessTest : public ::testing::Test{};

TEST_F(InitialGuessTest, DifferentIter) {
EXPECT_EQ (0, test_initial_guess (10,0,0));
EXPECT_EQ (0, test_initial_guess (15,0,0));
EXPECT_EQ (0, test_initial_guess (20,0,0));
}
TEST_F(InitialGuessTest, DifferentRobot) {
EXPECT_EQ (0, test_initial_guess (10,0,1));
EXPECT_EQ (0, test_initial_guess (15,0,1));
EXPECT_EQ (0, test_initial_guess (20,0,1));
}

}

int main(int argc, char** argv) {
::testing::InitGoogleTest(&argc, argv);
return RUN_ALL_TESTS();
}

