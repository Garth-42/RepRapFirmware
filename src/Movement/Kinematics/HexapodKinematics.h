/*
 * HexapodKinematics.h
 *
 * Kinematics for a Stewart platform (hexapod) with 6 linear actuators
 * connecting a fixed base to a moving platform, providing 6 DOF.
 *
 * Axes 0-2 (X, Y, Z) are translational in mm.
 * Axes 3-5 (A, B, C) are rotational in degrees (roll, pitch, yaw).
 */

#ifndef SRC_MOVEMENT_KINEMATICS_HEXAPODKINEMATICS_H_
#define SRC_MOVEMENT_KINEMATICS_HEXAPODKINEMATICS_H_

#include "Kinematics.h"

#if SUPPORT_HEXAPOD

class HexapodKinematics : public Kinematics
{
public:
	HexapodKinematics() noexcept;

	// Overridden base class functions. See Kinematics.h for descriptions.
	const char *_ecv_array GetName(bool forStatusReport) const noexcept override;
	bool Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply, bool& error) THROWS(GCodeException) override;
	MovementError CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const noexcept override;
	void MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const noexcept override;
	HomingMode GetHomingMode() const noexcept override { return HomingMode::homeIndividualDrives; }
	AxesBitmap AxesAssumedHomed(AxesBitmap g92Axes) const noexcept override;
	AxesBitmap MustBeHomedAxes(AxesBitmap axesMoving, bool disallowMovesBeforeHoming) const noexcept override;
	LimitPositionResult LimitPosition(float finalCoords[], const float *_ecv_array _ecv_null initialCoords, size_t numVisibleAxes, AxesBitmap axesToLimit, bool isCoordinated, bool applyM208Limits) const noexcept override;
	void GetAssumedInitialPosition(size_t numAxes, float positions[]) const noexcept override;
	LogicalDrivesBitmap GetControllingDrives(size_t axis, bool forHoming) const noexcept override;

protected:
	DECLARE_OBJECT_MODEL

private:
	static constexpr size_t NumActuators = 6;

	// Default geometry: semi-regular hexagonal arrangement
	// Base joints on circle of radius baseRadius, platform joints on circle of radius platformRadius
	// Adjacent pairs of joints are separated by baseHalfAngle (base) or platformHalfAngle (platform) degrees
	static constexpr float DefaultBaseRadius = 200.0;			// mm
	static constexpr float DefaultPlatformRadius = 100.0;		// mm
	static constexpr float DefaultBaseHalfAngle = 15.0;			// degrees, half-angle between adjacent base joint pairs
	static constexpr float DefaultPlatformHalfAngle = 40.0;		// degrees, half-angle between adjacent platform joint pairs
	static constexpr float DefaultInitialHeight = 250.0;		// mm, home Z height
	static constexpr float DefaultMinActuatorLength = 150.0;	// mm
	static constexpr float DefaultMaxActuatorLength = 350.0;	// mm

	// Compute base and platform joint positions from radii and angles
	void Recalc() noexcept;

	// Compute the actuator length for one strut given a platform pose
	float ComputeActuatorLength(size_t actuator, const float machinePos[]) const noexcept;

	// Forward kinematics using Newton-Raphson iteration
	bool ForwardKinematics(const float actuatorLengths[NumActuators], float machinePos[]) const noexcept;

	// Configurable parameters
	float baseRadius;
	float platformRadius;
	float baseHalfAngle;			// degrees
	float platformHalfAngle;		// degrees
	float initialHeight;			// home Z position
	float minActuatorLength;
	float maxActuatorLength;

	// Derived: joint positions in the base and platform frames
	// Base joints are in the fixed XY plane at Z=0
	// Platform joints are relative to the platform centre
	float baseJoints[NumActuators][3];		// [actuator][xyz]
	float platformJoints[NumActuators][3];	// [actuator][xyz] in platform-local frame
};

#endif // SUPPORT_HEXAPOD

#endif /* SRC_MOVEMENT_KINEMATICS_HEXAPODKINEMATICS_H_ */
