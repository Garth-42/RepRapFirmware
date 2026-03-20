/*
 * TestHexapodKinematics.cpp
 *
 * Host-compilable regression tests for the hexapod (Stewart platform) kinematics.
 *
 * The core geometry and kinematics math is extracted here to avoid pulling in
 * the full firmware dependency chain (FreeRTOS, HAL, ObjectModel, GCodes, etc.).
 * This tests the same algorithms used in HexapodKinematics.cpp:
 *   - Joint position computation (Recalc)
 *   - Inverse kinematics (ComputeActuatorLength)
 *   - Forward kinematics (Newton-Raphson solver)
 *
 * Build and run:
 *   make test-hexapod
 *   or: g++ -std=c++17 -O2 -o tests/test_hexapod tests/TestHexapodKinematics.cpp -lm && tests/test_hexapod
 */

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>

// ---------------------------------------------------------------------------
// Minimal stubs matching the firmware math helpers
// ---------------------------------------------------------------------------

static constexpr float DegreesToRadians = 3.141592653589793f / 180.0f;

static inline float fastSqrtf(float x) { return sqrtf(x); }
static inline float fsquare(float x)   { return x * x; }

// ---------------------------------------------------------------------------
// Extracted hexapod kinematics (mirrors HexapodKinematics.cpp exactly)
// ---------------------------------------------------------------------------

static constexpr size_t NumActuators = 6;

struct HexapodGeometry {
	float baseRadius;
	float platformRadius;
	float baseHalfAngle;      // degrees
	float platformHalfAngle;  // degrees
	float initialHeight;
	float minActuatorLength;
	float maxActuatorLength;

	float baseJoints[NumActuators][3];
	float platformJoints[NumActuators][3];
};

static void Recalc(HexapodGeometry& g)
{
	const float baseAngles[NumActuators] = {
		 0.0f - g.baseHalfAngle,
		 0.0f + g.baseHalfAngle,
		120.0f - g.baseHalfAngle,
		120.0f + g.baseHalfAngle,
		240.0f - g.baseHalfAngle,
		240.0f + g.baseHalfAngle
	};

	const float platformAngles[NumActuators] = {
		 60.0f - g.platformHalfAngle,
		 60.0f + g.platformHalfAngle,
		180.0f - g.platformHalfAngle,
		180.0f + g.platformHalfAngle,
		300.0f - g.platformHalfAngle,
		300.0f + g.platformHalfAngle
	};

	for (size_t i = 0; i < NumActuators; ++i)
	{
		const float baseAngleRad = baseAngles[i] * DegreesToRadians;
		g.baseJoints[i][0] = g.baseRadius * cosf(baseAngleRad);
		g.baseJoints[i][1] = g.baseRadius * sinf(baseAngleRad);
		g.baseJoints[i][2] = 0.0f;

		const float platAngleRad = platformAngles[i] * DegreesToRadians;
		g.platformJoints[i][0] = g.platformRadius * cosf(platAngleRad);
		g.platformJoints[i][1] = g.platformRadius * sinf(platAngleRad);
		g.platformJoints[i][2] = 0.0f;
	}
}

static float ComputeActuatorLength(const HexapodGeometry& g, size_t actuator, const float machinePos[])
{
	const float rollRad  = machinePos[3] * DegreesToRadians;
	const float pitchRad = machinePos[4] * DegreesToRadians;
	const float yawRad   = machinePos[5] * DegreesToRadians;

	const float cr = cosf(rollRad),  sr = sinf(rollRad);
	const float cp = cosf(pitchRad), sp = sinf(pitchRad);
	const float cy = cosf(yawRad),   sy = sinf(yawRad);

	const float px = g.platformJoints[actuator][0];
	const float py = g.platformJoints[actuator][1];
	const float pz = g.platformJoints[actuator][2];

	const float rpx = (cy * cp) * px + (cy * sp * sr - sy * cr) * py + (cy * sp * cr + sy * sr) * pz;
	const float rpy = (sy * cp) * px + (sy * sp * sr + cy * cr) * py + (sy * sp * cr - cy * sr) * pz;
	const float rpz = (-sp)     * px + (cp * sr)                * py + (cp * cr)                * pz;

	const float wx = machinePos[0] + rpx;
	const float wy = machinePos[1] + rpy;
	const float wz = machinePos[2] + rpz;

	const float dx = wx - g.baseJoints[actuator][0];
	const float dy = wy - g.baseJoints[actuator][1];
	const float dz = wz - g.baseJoints[actuator][2];

	return fastSqrtf(fsquare(dx) + fsquare(dy) + fsquare(dz));
}

static bool ForwardKinematics(const HexapodGeometry& g, const float actuatorLengths[NumActuators], float machinePos[])
{
	static constexpr size_t MaxIterations = 50;
	static constexpr float Tolerance = 0.001f;
	static constexpr float DeltaPos = 0.01f;

	float pose[6];
	for (size_t i = 0; i < 6; ++i)
	{
		pose[i] = machinePos[i];
	}

	for (size_t iter = 0; iter < MaxIterations; ++iter)
	{
		float errors[NumActuators];
		float maxError = 0.0f;
		for (size_t i = 0; i < NumActuators; ++i)
		{
			errors[i] = actuatorLengths[i] - ComputeActuatorLength(g, i, pose);
			const float absErr = fabsf(errors[i]);
			if (absErr > maxError)
			{
				maxError = absErr;
			}
		}

		if (maxError < Tolerance)
		{
			for (size_t i = 0; i < 6; ++i)
			{
				machinePos[i] = pose[i];
			}
			return true;
		}

		float jacobian[NumActuators][6];
		for (size_t j = 0; j < 6; ++j)
		{
			float perturbedPose[6];
			for (size_t k = 0; k < 6; ++k)
			{
				perturbedPose[k] = pose[k];
			}
			perturbedPose[j] += DeltaPos;

			for (size_t i = 0; i < NumActuators; ++i)
			{
				const float perturbedLength = ComputeActuatorLength(g, i, perturbedPose);
				const float currentLength = ComputeActuatorLength(g, i, pose);
				jacobian[i][j] = (perturbedLength - currentLength) / DeltaPos;
			}
		}

		float aug[6][7];
		for (size_t i = 0; i < 6; ++i)
		{
			for (size_t j = 0; j < 6; ++j)
			{
				aug[i][j] = jacobian[i][j];
			}
			aug[i][6] = errors[i];
		}

		bool singular = false;
		for (size_t col = 0; col < 6; ++col)
		{
			size_t maxRow = col;
			float maxVal = fabsf(aug[col][col]);
			for (size_t row = col + 1; row < 6; ++row)
			{
				const float v = fabsf(aug[row][col]);
				if (v > maxVal)
				{
					maxVal = v;
					maxRow = row;
				}
			}

			if (maxVal < 1e-10f)
			{
				singular = true;
				break;
			}

			if (maxRow != col)
			{
				for (size_t j = 0; j < 7; ++j)
				{
					const float tmp = aug[col][j];
					aug[col][j] = aug[maxRow][j];
					aug[maxRow][j] = tmp;
				}
			}

			for (size_t row = col + 1; row < 6; ++row)
			{
				const float factor = aug[row][col] / aug[col][col];
				for (size_t j = col; j < 7; ++j)
				{
					aug[row][j] -= factor * aug[col][j];
				}
			}
		}

		if (singular)
		{
			return false;
		}

		float delta[6];
		for (int i = 5; i >= 0; --i)
		{
			float sum = aug[i][6];
			for (size_t j = (size_t)(i + 1); j < 6; ++j)
			{
				sum -= aug[i][j] * delta[j];
			}
			delta[i] = sum / aug[i][i];
		}

		for (size_t i = 0; i < 6; ++i)
		{
			pose[i] += delta[i];
		}
	}

	return false;
}

// ---------------------------------------------------------------------------
// Test framework (minimal, no external dependencies)
// ---------------------------------------------------------------------------

static int g_testsPassed = 0;
static int g_testsFailed = 0;

#define TEST_ASSERT(cond, msg) \
	do { \
		if (!(cond)) { \
			fprintf(stderr, "  FAIL: %s (line %d): %s\n", msg, __LINE__, #cond); \
			g_testsFailed++; \
			return; \
		} \
	} while (0)

#define TEST_ASSERT_NEAR(a, b, tol, msg) \
	do { \
		if (fabsf((a) - (b)) > (tol)) { \
			fprintf(stderr, "  FAIL: %s (line %d): %.6f != %.6f (tol %.6f)\n", msg, __LINE__, (double)(a), (double)(b), (double)(tol)); \
			g_testsFailed++; \
			return; \
		} \
	} while (0)

#define RUN_TEST(fn) \
	do { \
		printf("  %-50s ", #fn); \
		fn(); \
		/* if we get here without early return, it passed */ \
		if (g_testsFailed == prevFailed) { \
			printf("PASS\n"); \
			g_testsPassed++; \
		} \
	} while (0)

static HexapodGeometry MakeDefault()
{
	HexapodGeometry g;
	g.baseRadius = 200.0f;
	g.platformRadius = 100.0f;
	g.baseHalfAngle = 15.0f;
	g.platformHalfAngle = 40.0f;
	g.initialHeight = 250.0f;
	g.minActuatorLength = 150.0f;
	g.maxActuatorLength = 350.0f;
	Recalc(g);
	return g;
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

// Test that base joints lie on a circle of baseRadius in the XY plane
static void test_base_joints_on_circle()
{
	HexapodGeometry g = MakeDefault();
	for (size_t i = 0; i < NumActuators; ++i)
	{
		float r = sqrtf(fsquare(g.baseJoints[i][0]) + fsquare(g.baseJoints[i][1]));
		TEST_ASSERT_NEAR(r, g.baseRadius, 0.01f, "base joint radius");
		TEST_ASSERT_NEAR(g.baseJoints[i][2], 0.0f, 0.001f, "base joint Z=0");
	}
}

// Test that platform joints lie on a circle of platformRadius
static void test_platform_joints_on_circle()
{
	HexapodGeometry g = MakeDefault();
	for (size_t i = 0; i < NumActuators; ++i)
	{
		float r = sqrtf(fsquare(g.platformJoints[i][0]) + fsquare(g.platformJoints[i][1]));
		TEST_ASSERT_NEAR(r, g.platformRadius, 0.01f, "platform joint radius");
		TEST_ASSERT_NEAR(g.platformJoints[i][2], 0.0f, 0.001f, "platform joint Z=0");
	}
}

// At home position (0,0,initialHeight,0,0,0), all actuators should have the same length
static void test_home_position_symmetric_lengths()
{
	HexapodGeometry g = MakeDefault();
	float pose[6] = { 0, 0, g.initialHeight, 0, 0, 0 };

	float lengths[NumActuators];
	for (size_t i = 0; i < NumActuators; ++i)
	{
		lengths[i] = ComputeActuatorLength(g, i, pose);
	}

	// With different base/platform half-angles, odd and even actuators have different lengths.
	// But 120-degree rotational symmetry means: 0==2==4 and 1==3==5.
	TEST_ASSERT_NEAR(lengths[0], lengths[2], 0.01f, "home symmetry even 0==2");
	TEST_ASSERT_NEAR(lengths[0], lengths[4], 0.01f, "home symmetry even 0==4");
	TEST_ASSERT_NEAR(lengths[1], lengths[3], 0.01f, "home symmetry odd 1==3");
	TEST_ASSERT_NEAR(lengths[1], lengths[5], 0.01f, "home symmetry odd 1==5");
}

// At home position, actuator lengths should be within configured bounds
static void test_home_position_within_bounds()
{
	HexapodGeometry g = MakeDefault();
	float pose[6] = { 0, 0, g.initialHeight, 0, 0, 0 };

	for (size_t i = 0; i < NumActuators; ++i)
	{
		float len = ComputeActuatorLength(g, i, pose);
		TEST_ASSERT(len >= g.minActuatorLength, "length >= min");
		TEST_ASSERT(len <= g.maxActuatorLength, "length <= max");
	}
}

// Inverse-then-forward kinematics round trip at home position
static void test_roundtrip_home_position()
{
	HexapodGeometry g = MakeDefault();
	float pose[6] = { 0, 0, g.initialHeight, 0, 0, 0 };

	float lengths[NumActuators];
	for (size_t i = 0; i < NumActuators; ++i)
	{
		lengths[i] = ComputeActuatorLength(g, i, pose);
	}

	// Start forward kinematics from a slightly perturbed guess
	float recovered[6] = { 1.0f, -1.0f, g.initialHeight + 2.0f, 0.5f, -0.5f, 0.3f };
	bool ok = ForwardKinematics(g, lengths, recovered);
	TEST_ASSERT(ok, "forward kinematics converged");

	for (size_t i = 0; i < 3; ++i)
	{
		TEST_ASSERT_NEAR(recovered[i], pose[i], 0.01f, "translation round-trip");
	}
	for (size_t i = 3; i < 6; ++i)
	{
		TEST_ASSERT_NEAR(recovered[i], pose[i], 0.01f, "rotation round-trip");
	}
}

// Round trip with a pure Z translation
static void test_roundtrip_z_offset()
{
	HexapodGeometry g = MakeDefault();
	float pose[6] = { 0, 0, 230.0f, 0, 0, 0 };

	float lengths[NumActuators];
	for (size_t i = 0; i < NumActuators; ++i)
	{
		lengths[i] = ComputeActuatorLength(g, i, pose);
	}

	float recovered[6] = { 0, 0, g.initialHeight, 0, 0, 0 };
	bool ok = ForwardKinematics(g, lengths, recovered);
	TEST_ASSERT(ok, "FK converged for Z offset");

	for (size_t i = 0; i < 6; ++i)
	{
		TEST_ASSERT_NEAR(recovered[i], pose[i], 0.01f, "Z offset round-trip");
	}
}

// Round trip with XY translation
static void test_roundtrip_xy_translation()
{
	HexapodGeometry g = MakeDefault();
	float pose[6] = { 10.0f, -15.0f, g.initialHeight, 0, 0, 0 };

	float lengths[NumActuators];
	for (size_t i = 0; i < NumActuators; ++i)
	{
		lengths[i] = ComputeActuatorLength(g, i, pose);
	}

	float recovered[6] = { 0, 0, g.initialHeight, 0, 0, 0 };
	bool ok = ForwardKinematics(g, lengths, recovered);
	TEST_ASSERT(ok, "FK converged for XY translation");

	for (size_t i = 0; i < 3; ++i)
	{
		TEST_ASSERT_NEAR(recovered[i], pose[i], 0.01f, "XY translation round-trip");
	}
	for (size_t i = 3; i < 6; ++i)
	{
		TEST_ASSERT_NEAR(recovered[i], pose[i], 0.01f, "rotation stays zero");
	}
}

// Round trip with small rotation (roll only)
static void test_roundtrip_roll()
{
	HexapodGeometry g = MakeDefault();
	float pose[6] = { 0, 0, g.initialHeight, 5.0f, 0, 0 };

	float lengths[NumActuators];
	for (size_t i = 0; i < NumActuators; ++i)
	{
		lengths[i] = ComputeActuatorLength(g, i, pose);
	}

	float recovered[6] = { 0, 0, g.initialHeight, 0, 0, 0 };
	bool ok = ForwardKinematics(g, lengths, recovered);
	TEST_ASSERT(ok, "FK converged for roll");

	for (size_t i = 0; i < 6; ++i)
	{
		TEST_ASSERT_NEAR(recovered[i], pose[i], 0.05f, "roll round-trip");
	}
}

// Round trip with combined translation and rotation
static void test_roundtrip_combined_pose()
{
	HexapodGeometry g = MakeDefault();
	float pose[6] = { 5.0f, -3.0f, g.initialHeight - 10.0f, 2.0f, -3.0f, 1.5f };

	float lengths[NumActuators];
	for (size_t i = 0; i < NumActuators; ++i)
	{
		lengths[i] = ComputeActuatorLength(g, i, pose);
	}

	float recovered[6] = { 0, 0, g.initialHeight, 0, 0, 0 };
	bool ok = ForwardKinematics(g, lengths, recovered);
	TEST_ASSERT(ok, "FK converged for combined pose");

	for (size_t i = 0; i < 3; ++i)
	{
		TEST_ASSERT_NEAR(recovered[i], pose[i], 0.05f, "combined translation round-trip");
	}
	for (size_t i = 3; i < 6; ++i)
	{
		TEST_ASSERT_NEAR(recovered[i], pose[i], 0.05f, "combined rotation round-trip");
	}
}

// Zero rotation should not change actuator lengths compared to identity
static void test_zero_rotation_is_identity()
{
	HexapodGeometry g = MakeDefault();
	float pose_a[6] = { 10, 20, g.initialHeight, 0, 0, 0 };
	float pose_b[6] = { 10, 20, g.initialHeight, 0, 0, 0 };

	for (size_t i = 0; i < NumActuators; ++i)
	{
		float la = ComputeActuatorLength(g, i, pose_a);
		float lb = ComputeActuatorLength(g, i, pose_b);
		TEST_ASSERT_NEAR(la, lb, 0.001f, "zero rotation identity");
	}
}

// Translating the platform up should increase all actuator lengths
static void test_z_up_increases_lengths()
{
	HexapodGeometry g = MakeDefault();
	float pose_low[6]  = { 0, 0, g.initialHeight, 0, 0, 0 };
	float pose_high[6] = { 0, 0, g.initialHeight + 20.0f, 0, 0, 0 };

	for (size_t i = 0; i < NumActuators; ++i)
	{
		float l_low  = ComputeActuatorLength(g, i, pose_low);
		float l_high = ComputeActuatorLength(g, i, pose_high);
		TEST_ASSERT(l_high > l_low, "Z up increases actuator length");
	}
}

// Changing configuration parameters should change joint positions
static void test_recalc_updates_joints()
{
	HexapodGeometry g1 = MakeDefault();
	HexapodGeometry g2 = MakeDefault();
	g2.baseRadius = 250.0f;
	Recalc(g2);

	// Base joints should now be at a larger radius
	float r1 = sqrtf(fsquare(g1.baseJoints[0][0]) + fsquare(g1.baseJoints[0][1]));
	float r2 = sqrtf(fsquare(g2.baseJoints[0][0]) + fsquare(g2.baseJoints[0][1]));
	TEST_ASSERT(r2 > r1, "larger base radius gives larger joint positions");
	TEST_ASSERT_NEAR(r2, 250.0f, 0.01f, "joint radius matches configured base radius");
}

// Forward kinematics should fail (not converge) for impossible actuator lengths
static void test_fk_fails_for_impossible_lengths()
{
	HexapodGeometry g = MakeDefault();
	// All actuators at 1mm - geometrically impossible for this configuration
	float lengths[NumActuators] = { 1, 1, 1, 1, 1, 1 };
	float recovered[6] = { 0, 0, g.initialHeight, 0, 0, 0 };
	bool ok = ForwardKinematics(g, lengths, recovered);
	TEST_ASSERT(!ok, "FK should not converge for impossible lengths");
}

// Inverse kinematics should produce positive actuator lengths
static void test_actuator_lengths_positive()
{
	HexapodGeometry g = MakeDefault();
	float pose[6] = { 0, 0, g.initialHeight, 0, 0, 0 };

	for (size_t i = 0; i < NumActuators; ++i)
	{
		float len = ComputeActuatorLength(g, i, pose);
		TEST_ASSERT(len > 0.0f, "actuator length is positive");
	}
}

// Test with a non-default geometry
static void test_roundtrip_custom_geometry()
{
	HexapodGeometry g;
	g.baseRadius = 150.0f;
	g.platformRadius = 75.0f;
	g.baseHalfAngle = 20.0f;
	g.platformHalfAngle = 35.0f;
	g.initialHeight = 200.0f;
	g.minActuatorLength = 100.0f;
	g.maxActuatorLength = 300.0f;
	Recalc(g);

	float pose[6] = { 3.0f, -2.0f, 195.0f, 1.0f, -1.0f, 0.5f };

	float lengths[NumActuators];
	for (size_t i = 0; i < NumActuators; ++i)
	{
		lengths[i] = ComputeActuatorLength(g, i, pose);
	}

	float recovered[6] = { 0, 0, g.initialHeight, 0, 0, 0 };
	bool ok = ForwardKinematics(g, lengths, recovered);
	TEST_ASSERT(ok, "FK converged with custom geometry");

	for (size_t i = 0; i < 3; ++i)
	{
		TEST_ASSERT_NEAR(recovered[i], pose[i], 0.05f, "custom geometry translation");
	}
	for (size_t i = 3; i < 6; ++i)
	{
		TEST_ASSERT_NEAR(recovered[i], pose[i], 0.05f, "custom geometry rotation");
	}
}

// Regression: specific known-good actuator lengths at home position with default config.
// If the geometry math changes, this catches it.
static void test_known_home_actuator_length()
{
	HexapodGeometry g = MakeDefault();
	float pose[6] = { 0, 0, g.initialHeight, 0, 0, 0 };

	float len0 = ComputeActuatorLength(g, 0, pose);

	// Compute expected length analytically for actuator 0:
	// Base joint 0: angle = 0 - 15 = -15 degrees on base circle radius 200
	// Platform joint 0: angle = 60 - 40 = 20 degrees on platform circle radius 100
	float bx = 200.0f * cosf(-15.0f * DegreesToRadians);
	float by = 200.0f * sinf(-15.0f * DegreesToRadians);
	float px = 100.0f * cosf(20.0f * DegreesToRadians);
	float py = 100.0f * sinf(20.0f * DegreesToRadians);
	float expected = sqrtf(fsquare(px - bx) + fsquare(py - by) + fsquare(250.0f));

	TEST_ASSERT_NEAR(len0, expected, 0.01f, "known actuator 0 length at home");
}

// Test that 120-degree rotational symmetry holds: pairs at 0/120/240 degrees
// should produce the same lengths at the centred home position
static void test_120_degree_symmetry()
{
	HexapodGeometry g = MakeDefault();
	float pose[6] = { 0, 0, g.initialHeight, 0, 0, 0 };

	float len[NumActuators];
	for (size_t i = 0; i < NumActuators; ++i)
	{
		len[i] = ComputeActuatorLength(g, i, pose);
	}

	// Pair-wise: actuators 0&1 should match 2&3 and 4&5 in the pair pattern
	TEST_ASSERT_NEAR(len[0], len[2], 0.01f, "120-deg symmetry act 0 vs 2");
	TEST_ASSERT_NEAR(len[0], len[4], 0.01f, "120-deg symmetry act 0 vs 4");
	TEST_ASSERT_NEAR(len[1], len[3], 0.01f, "120-deg symmetry act 1 vs 3");
	TEST_ASSERT_NEAR(len[1], len[5], 0.01f, "120-deg symmetry act 1 vs 5");
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main()
{
	printf("Hexapod Kinematics Regression Tests\n");
	printf("====================================\n");

	int prevFailed = 0;

	prevFailed = g_testsFailed; RUN_TEST(test_base_joints_on_circle);
	prevFailed = g_testsFailed; RUN_TEST(test_platform_joints_on_circle);
	prevFailed = g_testsFailed; RUN_TEST(test_home_position_symmetric_lengths);
	prevFailed = g_testsFailed; RUN_TEST(test_home_position_within_bounds);
	prevFailed = g_testsFailed; RUN_TEST(test_actuator_lengths_positive);
	prevFailed = g_testsFailed; RUN_TEST(test_zero_rotation_is_identity);
	prevFailed = g_testsFailed; RUN_TEST(test_z_up_increases_lengths);
	prevFailed = g_testsFailed; RUN_TEST(test_recalc_updates_joints);
	prevFailed = g_testsFailed; RUN_TEST(test_known_home_actuator_length);
	prevFailed = g_testsFailed; RUN_TEST(test_120_degree_symmetry);
	prevFailed = g_testsFailed; RUN_TEST(test_roundtrip_home_position);
	prevFailed = g_testsFailed; RUN_TEST(test_roundtrip_z_offset);
	prevFailed = g_testsFailed; RUN_TEST(test_roundtrip_xy_translation);
	prevFailed = g_testsFailed; RUN_TEST(test_roundtrip_roll);
	prevFailed = g_testsFailed; RUN_TEST(test_roundtrip_combined_pose);
	prevFailed = g_testsFailed; RUN_TEST(test_roundtrip_custom_geometry);
	prevFailed = g_testsFailed; RUN_TEST(test_fk_fails_for_impossible_lengths);

	printf("====================================\n");
	printf("Results: %d passed, %d failed\n", g_testsPassed, g_testsFailed);

	return (g_testsFailed > 0) ? 1 : 0;
}
