/*
 * HexapodKinematics.cpp
 *
 * Stewart platform (hexapod) kinematics implementation.
 *
 * The platform has 6 linear actuators connecting fixed base joints to
 * moving platform joints. The platform pose is described by 6 DOF:
 *   machinePos[0..2] = X, Y, Z translation of platform centre (mm)
 *   machinePos[3..5] = A, B, C rotation angles (degrees): roll, pitch, yaw
 *
 * Motor positions represent actuator lengths in steps (steps = length * stepsPerMm).
 *
 * Joint layout uses the semi-regular hexagonal pattern:
 *   Base joints at angles: 0 +/- baseHalfAngle, 120 +/- baseHalfAngle, 240 +/- baseHalfAngle
 *   Platform joints at angles: 60 +/- platformHalfAngle, 180 +/- platformHalfAngle, 300 +/- platformHalfAngle
 *
 * Configuration via M669 K15:
 *   B = base radius (mm)
 *   P = platform radius (mm)
 *   A = base half-angle (degrees)
 *   D = platform half-angle (degrees)
 *   H = initial/home height (mm)
 *   L = min:max actuator length (mm)
 *   S = segments per second
 *   T = min segment length
 */

#include "HexapodKinematics.h"

#if SUPPORT_HEXAPOD

#include <Platform/RepRap.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Movement/DDA.h>
#include <Movement/Move.h>

#if SUPPORT_OBJECT_MODEL

// Object model table and functions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(HexapodKinematics, __VA_ARGS__)

constexpr ObjectModelTableEntry HexapodKinematics::objectModelTable[] =
{
	// 0. kinematics members
	{ "baseHalfAngle",		OBJECT_MODEL_FUNC(self->baseHalfAngle, 1),		ObjectModelEntryFlags::none },
	{ "baseRadius",			OBJECT_MODEL_FUNC(self->baseRadius, 1),			ObjectModelEntryFlags::none },
	{ "maxActuatorLength",	OBJECT_MODEL_FUNC(self->maxActuatorLength, 1),	ObjectModelEntryFlags::none },
	{ "minActuatorLength",	OBJECT_MODEL_FUNC(self->minActuatorLength, 1),	ObjectModelEntryFlags::none },
	{ "name",				OBJECT_MODEL_FUNC(self->GetName(true)),			ObjectModelEntryFlags::none },
	{ "platformHalfAngle",	OBJECT_MODEL_FUNC(self->platformHalfAngle, 1),	ObjectModelEntryFlags::none },
	{ "platformRadius",		OBJECT_MODEL_FUNC(self->platformRadius, 1),		ObjectModelEntryFlags::none },
};

constexpr uint8_t HexapodKinematics::objectModelTableDescriptor[] = { 1, 7 };

DEFINE_GET_OBJECT_MODEL_TABLE_WITH_PARENT(HexapodKinematics, Kinematics)

#endif

// Constructor
HexapodKinematics::HexapodKinematics() noexcept
	: Kinematics(KinematicsType::hexapod, SegmentationType(true, true, true)),
	  baseRadius(DefaultBaseRadius),
	  platformRadius(DefaultPlatformRadius),
	  baseHalfAngle(DefaultBaseHalfAngle),
	  platformHalfAngle(DefaultPlatformHalfAngle),
	  initialHeight(DefaultInitialHeight),
	  minActuatorLength(DefaultMinActuatorLength),
	  maxActuatorLength(DefaultMaxActuatorLength)
{
	Recalc();
}

const char *_ecv_array HexapodKinematics::GetName(bool forStatusReport) const noexcept
{
	return "Hexapod";
}

// Configure from M669 K15 parameters
bool HexapodKinematics::Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply, bool& error) THROWS(GCodeException)
{
	if (mCode == 669)
	{
		bool seen = false;
		gb.TryGetFValue('B', baseRadius, seen);
		gb.TryGetFValue('P', platformRadius, seen);
		gb.TryGetFValue('A', baseHalfAngle, seen);
		gb.TryGetFValue('D', platformHalfAngle, seen);
		gb.TryGetFValue('H', initialHeight, seen);

		if (gb.Seen('L'))
		{
			seen = true;
			float limits[2];
			size_t numLimits = 2;
			gb.GetFloatArray(limits, numLimits, false);
			if (numLimits == 2)
			{
				minActuatorLength = limits[0];
				maxActuatorLength = limits[1];
			}
			else
			{
				reply.copy("L parameter requires min:max actuator lengths");
				error = true;
				return true;
			}
		}

		bool seenNonGeometry = TryConfigureSegmentation(gb);

		if (seen)
		{
			Recalc();
		}
		else if (!seenNonGeometry && !gb.Seen('K'))
		{
			Kinematics::Configure(mCode, gb, reply, error);
			reply.catf(", base radius %.1f, platform radius %.1f, base half-angle %.1f, platform half-angle %.1f"
					   ", height %.1f, actuator length %.1f to %.1f",
					   (double)baseRadius, (double)platformRadius,
					   (double)baseHalfAngle, (double)platformHalfAngle,
					   (double)initialHeight,
					   (double)minActuatorLength, (double)maxActuatorLength);
		}
		return seen;
	}
	else
	{
		return Kinematics::Configure(mCode, gb, reply, error);
	}
}

// Compute base and platform joint positions from configuration
void HexapodKinematics::Recalc() noexcept
{
	// Base joints: pairs at 0, 120, 240 degrees, each pair separated by +/- baseHalfAngle
	// Platform joints: pairs at 60, 180, 300 degrees, each pair separated by +/- platformHalfAngle
	const float baseAngles[NumActuators] = {
		 0.0f - baseHalfAngle,
		 0.0f + baseHalfAngle,
		120.0f - baseHalfAngle,
		120.0f + baseHalfAngle,
		240.0f - baseHalfAngle,
		240.0f + baseHalfAngle
	};

	const float platformAngles[NumActuators] = {
		 60.0f - platformHalfAngle,
		 60.0f + platformHalfAngle,
		180.0f - platformHalfAngle,
		180.0f + platformHalfAngle,
		300.0f - platformHalfAngle,
		300.0f + platformHalfAngle
	};

	for (size_t i = 0; i < NumActuators; ++i)
	{
		const float baseAngleRad = baseAngles[i] * DegreesToRadians;
		baseJoints[i][0] = baseRadius * cosf(baseAngleRad);
		baseJoints[i][1] = baseRadius * sinf(baseAngleRad);
		baseJoints[i][2] = 0.0f;

		const float platAngleRad = platformAngles[i] * DegreesToRadians;
		platformJoints[i][0] = platformRadius * cosf(platAngleRad);
		platformJoints[i][1] = platformRadius * sinf(platAngleRad);
		platformJoints[i][2] = 0.0f;
	}
}

// Compute the actuator length for one strut given a platform pose
// machinePos[0..2] = X, Y, Z translation
// machinePos[3..5] = A (roll), B (pitch), C (yaw) in degrees
float HexapodKinematics::ComputeActuatorLength(size_t actuator, const float machinePos[]) const noexcept
{
	// Build rotation matrix from ZYX Euler angles (yaw-pitch-roll convention)
	const float rollRad  = machinePos[3] * DegreesToRadians;
	const float pitchRad = machinePos[4] * DegreesToRadians;
	const float yawRad   = machinePos[5] * DegreesToRadians;

	const float cr = cosf(rollRad),  sr = sinf(rollRad);
	const float cp = cosf(pitchRad), sp = sinf(pitchRad);
	const float cy = cosf(yawRad),   sy = sinf(yawRad);

	// Rotation matrix R = Rz(yaw) * Ry(pitch) * Rx(roll)
	// R = [ cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr ]
	//     [ sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr ]
	//     [  -sp,        cp*sr,             cp*cr        ]

	const float px = platformJoints[actuator][0];
	const float py = platformJoints[actuator][1];
	const float pz = platformJoints[actuator][2];

	// Rotated platform joint position
	const float rpx = (cy * cp) * px + (cy * sp * sr - sy * cr) * py + (cy * sp * cr + sy * sr) * pz;
	const float rpy = (sy * cp) * px + (sy * sp * sr + cy * cr) * py + (sy * sp * cr - cy * sr) * pz;
	const float rpz = (-sp)     * px + (cp * sr)                * py + (cp * cr)                * pz;

	// World position of platform joint = translation + rotated platform joint
	const float wx = machinePos[0] + rpx;
	const float wy = machinePos[1] + rpy;
	const float wz = machinePos[2] + rpz;

	// Actuator length = distance from base joint to world platform joint
	const float dx = wx - baseJoints[actuator][0];
	const float dy = wy - baseJoints[actuator][1];
	const float dz = wz - baseJoints[actuator][2];

	return fastSqrtf(fsquare(dx) + fsquare(dy) + fsquare(dz));
}

// Inverse kinematics: Cartesian + rotation pose -> motor steps (actuator lengths)
MovementError HexapodKinematics::CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[],
	size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const noexcept
{
	MovementError rslt = MovementError::ok;

	for (size_t i = 0; i < NumActuators; ++i)
	{
		const float length = ComputeActuatorLength(i, machinePos);
		RoundToInt32(rslt, length * stepsPerMm[i], motorPos[i]);
	}

	// Any additional axes beyond the 6 DOF are treated linearly
	for (size_t axis = NumActuators; axis < numVisibleAxes; ++axis)
	{
		RoundToInt32(rslt, machinePos[axis] * stepsPerMm[axis], motorPos[axis]);
	}

	return rslt;
}

// Forward kinematics using Newton-Raphson iteration
// Given actuator lengths, solve for the platform pose
bool HexapodKinematics::ForwardKinematics(const float actuatorLengths[NumActuators], float machinePos[]) const noexcept
{
	// Start from current machinePos as initial guess (caller provides it)
	// Newton-Raphson: iteratively adjust pose to minimise actuator length errors

	static constexpr size_t MaxIterations = 50;
	static constexpr float Tolerance = 0.001f;		// mm
	static constexpr float DeltaPos = 0.01f;		// mm or degrees for numerical Jacobian

	float pose[6];
	for (size_t i = 0; i < 6; ++i)
	{
		pose[i] = machinePos[i];
	}

	for (size_t iter = 0; iter < MaxIterations; ++iter)
	{
		// Compute current actuator lengths and errors
		float errors[NumActuators];
		float maxError = 0.0f;
		for (size_t i = 0; i < NumActuators; ++i)
		{
			errors[i] = actuatorLengths[i] - ComputeActuatorLength(i, pose);
			const float absErr = fabsf(errors[i]);
			if (absErr > maxError)
			{
				maxError = absErr;
			}
		}

		if (maxError < Tolerance)
		{
			// Converged
			for (size_t i = 0; i < 6; ++i)
			{
				machinePos[i] = pose[i];
			}
			return true;
		}

		// Compute 6x6 Jacobian numerically: J[i][j] = d(actuatorLength_i) / d(pose_j)
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
				const float perturbedLength = ComputeActuatorLength(i, perturbedPose);
				const float currentLength = ComputeActuatorLength(i, pose);
				jacobian[i][j] = (perturbedLength - currentLength) / DeltaPos;
			}
		}

		// Solve J * delta = errors using Gaussian elimination with partial pivoting
		// Augmented matrix [J | errors]
		float aug[6][7];
		for (size_t i = 0; i < 6; ++i)
		{
			for (size_t j = 0; j < 6; ++j)
			{
				aug[i][j] = jacobian[i][j];
			}
			aug[i][6] = errors[i];
		}

		// Forward elimination with partial pivoting
		bool singular = false;
		for (size_t col = 0; col < 6; ++col)
		{
			// Find pivot
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

			// Swap rows
			if (maxRow != col)
			{
				for (size_t j = 0; j < 7; ++j)
				{
					const float tmp = aug[col][j];
					aug[col][j] = aug[maxRow][j];
					aug[maxRow][j] = tmp;
				}
			}

			// Eliminate below
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

		// Back substitution
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

		// Update pose
		for (size_t i = 0; i < 6; ++i)
		{
			pose[i] += delta[i];
		}
	}

	// Did not converge
	return false;
}

// Forward kinematics: motor steps -> Cartesian coordinates
void HexapodKinematics::MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[],
	size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const noexcept
{
	// Convert motor steps to actuator lengths
	float actuatorLengths[NumActuators];
	for (size_t i = 0; i < NumActuators; ++i)
	{
		actuatorLengths[i] = (float)motorPos[i] / stepsPerMm[i];
	}

	// Use current machinePos as initial guess for Newton-Raphson
	if (!ForwardKinematics(actuatorLengths, machinePos))
	{
		// If forward kinematics fails to converge, leave machinePos unchanged
		// (it retains the previous position as best estimate)
	}

	// Convert any additional axes linearly
	for (size_t axis = NumActuators; axis < numVisibleAxes; ++axis)
	{
		machinePos[axis] = (float)motorPos[axis] / stepsPerMm[axis];
	}
}

// If both X and Y (and ideally Z, A, B, C) are set via G92, we know the full pose
AxesBitmap HexapodKinematics::AxesAssumedHomed(AxesBitmap g92Axes) const noexcept
{
	// All 6 actuators depend on all 6 DOF, so unless all primary axes are set we can't assume anything
	const AxesBitmap allSixAxes = AxesBitmap::MakeLowestNBits(NumActuators);
	if ((g92Axes & allSixAxes) != allSixAxes)
	{
		g92Axes &= ~allSixAxes;
	}
	return g92Axes;
}

// All 6 DOF axes must be homed before any movement
AxesBitmap HexapodKinematics::MustBeHomedAxes(AxesBitmap axesMoving, bool disallowMovesBeforeHoming) const noexcept
{
	const AxesBitmap allSixAxes = AxesBitmap::MakeLowestNBits(NumActuators);
	if (axesMoving.Intersects(allSixAxes))
	{
		return allSixAxes;
	}
	return axesMoving;
}

// Limit position using M208 limits
LimitPositionResult HexapodKinematics::LimitPosition(float finalCoords[], const float *_ecv_array _ecv_null initialCoords,
	size_t numVisibleAxes, AxesBitmap axesToLimit, bool isCoordinated, bool applyM208Limits) const noexcept
{
	return (applyM208Limits && LimitPositionFromAxis(finalCoords, 0, numVisibleAxes, axesToLimit))
			? LimitPositionResult::adjusted
			: LimitPositionResult::ok;
}

// Initial position: platform centred at (0, 0, initialHeight) with no rotation
void HexapodKinematics::GetAssumedInitialPosition(size_t numAxes, float positions[]) const noexcept
{
	for (size_t i = 0; i < numAxes; ++i)
	{
		positions[i] = 0.0f;
	}
	if (numAxes > Z_AXIS)
	{
		positions[Z_AXIS] = initialHeight;
	}
}

// All 6 actuators contribute to all 6 axes
LogicalDrivesBitmap HexapodKinematics::GetControllingDrives(size_t axis, bool forHoming) const noexcept
{
	if (forHoming || axis >= NumActuators)
	{
		return LogicalDrivesBitmap::MakeFromBits(axis);
	}
	// All 6 drives affect each of the 6 DOF axes
	return LogicalDrivesBitmap::MakeLowestNBits(NumActuators);
}

#endif // SUPPORT_HEXAPOD

// End
