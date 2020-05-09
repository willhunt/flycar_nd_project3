#include <gtest/gtest.h>
#include "Common.h"
#include "../src/Utility/SimpleConfig.h"
#include "../src/Utility/StringUtils.h"
#include "../src/Math/Quaternion.h"
#include "../src/Math/V3F.h"
#include "../src/QuadControl.h"

// using SLR::Quaternion;
using namespace SLR;

// Test test
TEST(ATestGroup, ATest) {
    EXPECT_TRUE(true);
}

// Test fixture
struct QuadControlTest : testing::Test {
    QuadControl* quad_control;

    QuadControlTest() {
        ParamsHandle config = SimpleConfig::GetInstance();
        char buf[100];
        string controlConfig = config->Get(config->Get(buf, "Quad") + ".ControlConfig", "ControlParams");
        quad_control = new QuadControl(controlConfig);
        // quad_control = QuadControl::init();
    }

    ~QuadControlTest() {
        delete quad_control;
    }
};


// // Test_F for test with fixture
// TEST_F(QuadControlTest, AltitudeControlPositiveTrust) {
//     Quaternion<float> q = Quaternion<float>::FromEulerYPR(0, 0, 0);
//     float thrust = quad_control->AltitudeControl(-10.0f, -1.0f, -5.0f, 0.0f, q, -0.1f, 1.0f);
//     ASSERT_GT(thrust, 0);
// }

// TEST_F(QuadControlTest, YAWCONTROL) {
//     float yawCmd = 3;
//     float yaw = 1;
//     float yawResult = quad_control->YawControl(yawCmd, yaw);
//     ASSERT_GT(yaw, 0);
// }

// int main(int argc, char* argv[]) {
//     testing::InitGoogleTest(&argc, argv);
//     return RUN_ALL_TESTS();
// }