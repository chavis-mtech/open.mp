#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

// The two pieces of arithmetic that decide where a driven NPC's wheels are, kept free of the
// component so they can be tested without a server. Both were wrong in ways that only show on
// a real client, which is why they are pure now.
namespace npc_navigation
{

// What a navi node says about the road it sits on. Bits 0-7 of the flags are the path WIDTH
// (in 1/8 m), bits 8-10 the lanes to the left, bits 11-13 the lanes to the right - verified
// against every NODES*.DAT the game ships rather than read off a wiki: 97% of navi nodes carry
// a zero width, ~19,000 are two-way (1,1), and ~12,000 are one-way with lanes on one side only.
struct RoadLanes
{
	float pathWidth = 0.0f; // metres; 0 when the node file does not say
	uint8_t leftLanes = 0;
	uint8_t rightLanes = 0;

	static RoadLanes fromNaviFlags(uint32_t flags)
	{
		RoadLanes lanes;
		lanes.pathWidth = static_cast<float>(flags & 0xFFu) / 8.0f;
		lanes.leftLanes = static_cast<uint8_t>((flags >> 8) & 0x7u);
		lanes.rightLanes = static_cast<uint8_t>((flags >> 11) & 0x7u);
		return lanes;
	}

	bool oneWay() const { return leftLanes == 0 || rightLanes == 0; }
};

constexpr float LaneWidth = 3.5f;

// Lateral offset, in metres to the RIGHT of the navi node, for the lane a driver should use.
//
// A navi node sits on the centre of the link. On a two-way road that is the road's
// centreline, so right-hand traffic starts half a lane to the right of it. On a ONE-WAY road
// - every freeway carriageway, every ramp, every one-way street - the node is the centre of
// the carriageway itself, and lanes are spread around it, not stacked to one side of it.
//
// The old formula was `width + 3.5 * (0.5 + lane)` for every road. Two things wrong with it:
// the width byte is the road's total width, not a median, so any non-zero value pushed the
// car most of the way off the road; and on a one-way road with two lanes the outer lane
// landed 1.75 m past the carriageway edge - onto the kerb, the barrier, or the retaining
// wall, which is exactly where a player photographed one.
inline float laneOffset(const RoadLanes& road, uint8_t laneCount, uint8_t laneIndex)
{
	if (laneCount == 0)
	{
		return 0.0f;
	}
	const uint8_t lane = std::min<uint8_t>(laneIndex, laneCount - 1);
	float offset;
	if (road.oneWay())
	{
		// Centred: for two lanes that is -1.75 and +1.75, never +5.25.
		offset = LaneWidth * (static_cast<float>(lane) - (static_cast<float>(laneCount) - 1.0f) / 2.0f);
	}
	else
	{
		offset = LaneWidth * (0.5f + static_cast<float>(lane));
	}
	// When the file does state a width, no lane may sit outside it. Half the width minus half
	// a lane is the furthest a lane centre can be from the road centre.
	if (road.pathWidth > LaneWidth)
	{
		const float limit = road.pathWidth / 2.0f - LaneWidth / 2.0f;
		offset = std::clamp(offset, -limit, limit);
	}
	return offset;
}

// The height a vehicle should be at while travelling a link, given how far along it is.
//
// Two nodes are joined by a straight piece of road, and the road's height changes linearly
// between them; the node files are dense enough on hills and ramps that this is true to
// within the chassis clearance. The old code moved z at a rate clamped to 35% of the
// horizontal speed, which on any ramp steeper than that meant the car climbed slower than the
// road and drove into it, then snapped up at the node - and on the way down floated above it.
inline float heightAlongLink(float startZ, float endZ, float horizontalProgress)
{
	const float t = std::clamp(horizontalProgress, 0.0f, 1.0f);
	return startZ + (endZ - startZ) * t;
}

// The chord between two path nodes sits under the road on a crest and above it in a dip;
// nodes are 10-30m apart, so a car on a hill drove half-buried or on air for most of a
// link. With the previous node known, the link is a quadratic that leaves the start at
// the slope the car arrived on and still reaches the end node exactly. The bulge over
// the chord is capped: a stairway or a bad previous link must not fling the car upward.
// A ramp into a tunnel or off a bridge is a steep link followed by a flat one: carrying
// the whole incoming slope into the flat link buried a car to its windows at the LS
// tunnel mouth. Half the tangent, and a bulge no deeper than a wheel.
constexpr float TangentCarry = 0.5f;
constexpr float MaxLinkBulge = 0.35f;

inline float heightAlongLinkCurved(
	float previousZ, float previousRun, float startZ, float endZ, float run, float horizontalProgress)
{
	const float t = std::clamp(horizontalProgress, 0.0f, 1.0f);
	if (previousRun < 0.5f || run < 0.5f)
	{
		return heightAlongLink(startZ, endZ, t);
	}
	const float rise = endZ - startZ;
	// Tangent at the start, expressed as a rise over this link's whole run.
	float startTangent = (startZ - previousZ) / previousRun * run * TangentCarry;
	// z(t) = startZ + a*t + b*t^2 with a = tangent, a + b = rise. Deviation from the chord
	// is (a - rise) * t * (1 - t), peaking at a quarter of (a - rise) at mid-link.
	const float maxTangentExcess = MaxLinkBulge * 4.0f;
	startTangent = std::clamp(startTangent, rise - maxTangentExcess, rise + maxTangentExcess);
	return startZ + startTangent * t + (rise - startTangent) * t * t;
}

// How far along a link the vehicle is, from the horizontal distance covered and the link's
// horizontal length. Horizontal on purpose: measuring progress in 3D lets the z error feed
// back into itself.
inline float linkProgress(float horizontalTravelled, float horizontalLength)
{
	if (horizontalLength <= 0.001f)
	{
		return 1.0f;
	}
	return std::clamp(horizontalTravelled / horizontalLength, 0.0f, 1.0f);
}

// --- how a car gets up to speed, and how hard it may turn once it has --------------------
//
// A driven NPC's speed used to be rebuilt from scratch on every tick:
//
//     speed = moveSpeed * cornerScale(headingError)
//
// with no memory of the previous tick. That is not how anything with mass moves, and it is
// the single biggest reason a server-driven vehicle reads as a prop being dragged along a
// line: it leaves a standstill already at full speed, loses a third of that the instant a
// bend appears, has it all back the instant the bend ends, and stops dead on arrival.
// Players report this as the car being "position-locked" and they are reading real
// information off the screen - every one of those transitions is a step no vehicle can make.
//
// Speed is a state here instead: carried between ticks, and moved toward what the corner
// allows at an acceleration a car can actually deliver. The corner scale itself is left
// exactly as it was - it was tuned against real driving and it is not what was wrong.
//
// Everything in this block is SI: metres per second, metres per second squared, degrees per
// second. The component's internal velocity is metres per MILLISECOND, so it converts at the
// call site rather than spreading a factor of 1000 through the arithmetic.
constexpr float DriveAcceleration = 4.2f; // m/s^2 - a saloon pulling away without drama
constexpr float DriveBraking = 7.5f; // m/s^2 - firm, still short of locking the wheels
// What holds a car in a bend. A yaw rate is bought with lateral grip, so the faster it goes
// the less of one it can afford: omega = lateral grip / speed.
constexpr float DriveLateralGrip = 7.0f; // m/s^2
// The steering rack's own limit, which is what bounds a car at parking speed, and a floor so
// a vehicle at motorway speed can still follow a motorway bend.
constexpr float MaxSteeringDegreesPerSecond = 105.0f;
constexpr float MinSteeringDegreesPerSecond = 18.0f;
constexpr float DegreesPerRadian = 57.2957795f;

// How fast the car may rotate at this speed. At a crawl the rack decides; by 50 km/h grip
// does, and the limit has fallen to around a quarter of the old fixed 105 deg/s - which is
// why every bend used to look like the car had been picked up and twisted.
inline float steeringLimitDegreesPerSecond(float speedMetresPerSecond)
{
	if (!(speedMetresPerSecond > 0.5f)) // also catches NaN
	{
		return MaxSteeringDegreesPerSecond;
	}
	const float grip = DriveLateralGrip / speedMetresPerSecond * DegreesPerRadian;
	return std::clamp(grip, MinSteeringDegreesPerSecond, MaxSteeringDegreesPerSecond);
}

// The longest tick the driving model will integrate its CONTROLS over. A server hitch - a
// slow query, a map load - hands advance() a delta of whole seconds, and an unclamped rate
// limit stops being a limit at that point: the heading would swing 200 degrees and the
// speed jump 8 m/s in one step, which is precisely the "grabbed and rotated in place"
// artefact this whole block exists to remove. A hitch means the car carries on doing what
// it was doing, not that it snaps to where it would have got to.
constexpr float DriveMaxTickSeconds = 0.2f;

inline float driveTickSeconds(float deltaSeconds)
{
	if (!(deltaSeconds > 0.0f)) // also catches NaN
	{
		return 0.0f;
	}
	return std::min(deltaSeconds, DriveMaxTickSeconds);
}

// How much of the requested speed a bend this sharp leaves. Unchanged behaviour, named.
inline float corneringSpeedScale(float headingErrorDegrees)
{
	return std::clamp(1.0f - std::abs(headingErrorDegrees) / 135.0f, 0.25f, 1.0f);
}

// The speed after `deltaSeconds` of accelerating (or braking) toward `wanted`. Never
// negative: a driven NPC has no reverse gear, it turns around instead.
//
// Guarded against a non-finite input because this value is STATE. A single NaN reaching it
// would survive every clamp below and wedge that NPC's speed for the rest of the session,
// where a NaN in the old stateless expression lasted one tick.
inline float approachDriveSpeed(float current, float wanted, float deltaSeconds)
{
	if (!std::isfinite(current))
	{
		current = 0.0f;
	}
	if (!std::isfinite(wanted))
	{
		return current;
	}
	const float rate = wanted > current ? DriveAcceleration : DriveBraking;
	const float limit = rate * driveTickSeconds(deltaSeconds);
	const float step = std::clamp(wanted - current, -limit, limit);
	return std::max(current + step, 0.0f);
}

// A car creeps rather than stops when geometry says it cannot have any speed at all. Zero
// would be a deadlock: no speed means no progress, no progress means the heading error
// never resolves, and the NPC sits at the node forever.
constexpr float DriveCreepSpeed = 1.5f; // m/s

// The fastest this car may approach a target it still has to turn toward.
//
// Grip-limited steering buys realism at the cost of a failure the old flat 105 deg/s could
// not have: arriving at a junction still pointing the wrong way. The arrival test requires a
// driver to be within 10 degrees of its target, so a car that could not finish the turn in
// the distance it had drove straight past the node, watched the error grow to 180, and had
// to come round the block. Braking only once the bend is visible is too late - so cap the
// speed at the one that leaves just enough time to finish turning before arriving.
inline float turnCompletionSpeedLimit(float distanceToTarget, float headingErrorDegrees, float currentSpeed)
{
	const float error = std::abs(headingErrorDegrees);
	if (!std::isfinite(distanceToTarget) || !std::isfinite(error) || error <= 1.0f || distanceToTarget <= 0.01f)
	{
		return std::numeric_limits<float>::max(); // nothing to turn, or nowhere to turn it
	}
	const float turnSeconds = error / steeringLimitDegreesPerSecond(currentSpeed);
	if (turnSeconds <= 0.001f)
	{
		return std::numeric_limits<float>::max();
	}
	return std::max(distanceToTarget / turnSeconds, DriveCreepSpeed);
}

} // namespace npc_navigation
