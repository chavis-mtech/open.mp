#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

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

} // namespace npc_navigation
