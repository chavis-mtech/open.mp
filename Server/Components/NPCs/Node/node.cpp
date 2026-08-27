/*
 *  This Source Code Form is subject to the terms of the Mozilla Public License,
 *  v. 2.0. If a copy of the MPL was not distributed with this file, You can
 *  obtain one at http://mozilla.org/MPL/2.0/.
 *
 *  The original code is copyright (c) 2025, open.mp team and contributors.
 */

#include "node.hpp"
#include "../NPC/npc.hpp"
#include <algorithm>
#include <random>
#include <ghc/filesystem.hpp>
#include <httplib.h>

NPCNode::NPCNode(int nodeId)
	: nodeId_(nodeId)
	, initialized_(false)
	, currentPointId_(0)
	, currentLinkId_(0)
{
	nodeHeader_ = {};
}

NPCNode::~NPCNode()
{
}

bool NPCNode::initialize(ICore* core)
{
	if (nodeId_ < 0 || nodeId_ >= 64)
	{
		return false;
	}

	std::string filePath = "scriptfiles/NPCs/nodes/NODES" + std::to_string(nodeId_) + ".DAT";

	if (!ghc::filesystem::exists(filePath))
	{
		std::string dirPath = "scriptfiles/NPCs/nodes";
		if (!ghc::filesystem::exists(dirPath))
		{
			ghc::filesystem::create_directories(dirPath);
		}

		std::string url = "assets.open.mp";
		std::string path = "/npc_nodes/NODES" + std::to_string(nodeId_) + ".DAT";

		httplib::Client client(url);
		client.set_connection_timeout(10, 0);
		client.set_read_timeout(30, 0);

		auto result = client.Get(path.c_str());
		if (result && result->status == 200)
		{
			std::ofstream outFile(filePath, std::ios::binary);
			if (outFile.is_open())
			{
				outFile.write(result->body.c_str(), result->body.size());
				outFile.close();
				core->logLn(LogLevel::Message, "[NPCs] Downloaded node file: NODES%d.DAT", nodeId_);
			}
			else
			{
				core->logLn(LogLevel::Warning, "[NPCs] Failed to save downloaded node file: NODES%d.DAT", nodeId_);
				core->logLn(LogLevel::Message, "[NPCs] Download the package manually from https://assets.open.mp/npc_nodes/NODES.zip and extract the contents in `scriptfiles/NPCs/nodes/NODES`");
				return false;
			}
		}
		else
		{
			core->logLn(LogLevel::Warning, "[NPCs] Failed to download node file: NODES%d.DAT (HTTP status: %d)", nodeId_, result ? result->status : -1);
			core->logLn(LogLevel::Message, "[NPCs] Download the package manually from https://assets.open.mp/npc_nodes/NODES.zip and extract the contents in `scriptfiles/NPCs/nodes/NODES`");
			return false;
		}
	}

	std::ifstream file(filePath, std::ios::binary);
	if (!file.is_open())
	{
		return false;
	}

	file.seekg(0, std::ios::end);
	std::streamsize fileSize = file.tellg();
	file.seekg(0, std::ios::beg);

	if (fileSize == 0)
	{
		return false;
	}

	if (!file.read(reinterpret_cast<char*>(&nodeHeader_), sizeof(NodeHeader)))
	{
		return false;
	}

	uint32_t totalPathNodes = nodeHeader_.vehicleNodesNumber + nodeHeader_.pedNodesNumber;
	pathNodes_.resize(totalPathNodes);
	if (totalPathNodes > 0)
	{
		if (!file.read(reinterpret_cast<char*>(pathNodes_.data()), totalPathNodes * sizeof(PathNode)))
		{
			return false;
		}
	}

	naviNodes_.resize(nodeHeader_.naviNodesNumber);
	if (nodeHeader_.naviNodesNumber > 0)
	{
		if (!file.read(reinterpret_cast<char*>(naviNodes_.data()), nodeHeader_.naviNodesNumber * sizeof(NaviNode)))
		{
			return false;
		}
	}

	linkNodes_.resize(nodeHeader_.linksNumber);
	if (nodeHeader_.linksNumber > 0)
	{
		if (!file.read(reinterpret_cast<char*>(linkNodes_.data()), nodeHeader_.linksNumber * sizeof(LinkNode)))
		{
			return false;
		}
	}

	// Section 3 is followed by a fixed 768-byte filler, then one packed navi-link
	// for every path link. The referenced NaviNode carries direction and lane counts.
	file.seekg(768, std::ios::cur);
	if (!file.good())
	{
		return false;
	}
	naviLinks_.resize(nodeHeader_.linksNumber);
	if (nodeHeader_.linksNumber > 0
		&& !file.read(reinterpret_cast<char*>(naviLinks_.data()), nodeHeader_.linksNumber * sizeof(uint16_t)))
	{
		return false;
	}

	file.close();

	if (!pathNodes_.empty())
	{
		currentPointId_ = 0;
	}

	initialized_ = true;
	return true;
}

uint16_t NPCNode::process(NPC* npc, uint16_t pointId, uint16_t lastArea, uint16_t lastPoint, uint16_t& currentLinkId)
{
	if (!initialized_)
	{
		return InvalidPoint;
	}

	uint16_t startLink = getLinkId(pointId);
	uint16_t linkCount = getLinkCount(pointId);
	if (linkCount == 0 || startLink >= linkNodes_.size())
	{
		return InvalidPoint;
	}

	DynamicArray<uint16_t> forwardLinks;
	DynamicArray<uint16_t> backtrackLinks;
	const uint32_t endLink = std::min<uint32_t>(
		static_cast<uint32_t>(startLink) + linkCount, static_cast<uint32_t>(linkNodes_.size()));
	for (uint32_t candidate = startLink; candidate < endLink; ++candidate)
	{
		const LinkNode& link = linkNodes_[candidate];
		if (link.areaId == 65535 || !npc->isNodeLinkTraversable(*this, static_cast<uint16_t>(candidate), pointId))
		{
			continue;
		}
		const bool backtrack = link.areaId == lastArea && link.nodeId == lastPoint;
		(backtrack ? backtrackLinks : forwardLinks).push_back(static_cast<uint16_t>(candidate));
	}

	// Exhaustively inspect every edge rather than hoping ten random draws find a valid
	// directional lane. Only U-turn when this is a real dead end.
	const auto& candidates = !forwardLinks.empty() ? forwardLinks : backtrackLinks;
	if (candidates.empty())
	{
		return InvalidPoint;
	}
	const uint16_t linkId = candidates[static_cast<std::size_t>(rand()) % candidates.size()];
	const LinkNode& currentLink = linkNodes_[linkId];
	currentLinkId = linkId;

	if (currentLink.areaId != nodeId_)
	{
		if (currentLink.areaId != 65535)
		{
			return ChangeArea;
		}
		return InvalidPoint;
	}

	npc->updateNodePoint(currentLink.nodeId);
	return currentLink.nodeId;
}

uint16_t NPCNode::processNodeChange(NPC* npc, uint16_t targetPointId)
{
	if (!initialized_ || targetPointId >= pathNodes_.size())
	{
		return InvalidPoint;
	}

	npc->updateNodePoint(targetPointId);
	return targetPointId;
}

Vector3 NPCNode::getPosition()
{
	if (!initialized_ || currentPointId_ >= pathNodes_.size())
	{
		return Vector3(0.0f, 0.0f, 0.0f);
	}

	const PathNode& pathNode = pathNodes_[currentPointId_];
	Vector3 normalPosition = Vector3(
		static_cast<float>(pathNode.positionX) / 8.0f,
		static_cast<float>(pathNode.positionY) / 8.0f,
		static_cast<float>(pathNode.positionZ) / 8.0f + pointZOffset(currentPointId_));
	return normalPosition;
}

Vector3 NPCNode::getPosition(uint16_t pointId) const
{
	if (!initialized_ || pointId >= pathNodes_.size())
	{
		return Vector3(0.0f, 0.0f, 0.0f);
	}

	const PathNode& pathNode = pathNodes_[pointId];
	return Vector3(
		static_cast<float>(pathNode.positionX) / 8.0f,
		static_cast<float>(pathNode.positionY) / 8.0f,
		static_cast<float>(pathNode.positionZ) / 8.0f + pointZOffset(pointId));
}

float NPCNode::pointZOffset(uint16_t pointId) const
{
	// Path-node z is stored at road level. The historical +1.2 matches the ped sync
	// origin (mid-torso) and is correct for PED points only. Node files store vehicle
	// points first, then ped points, so the point id tells the two apart. A vehicle
	// spawned or driver-synced at the ped offset hangs ~0.7m above the tarmac — and
	// because clients keep unoccupied-vehicle physics asleep until something touches
	// the car, a parked or abandoned one never settles: it visibly floats. Vehicle
	// points therefore carry the chassis rest height instead.
	constexpr float VehiclePointZOffset = 0.5f;
	constexpr float PedPointZOffset = 1.2f;
	return pointId < nodeHeader_.vehicleNodesNumber ? VehiclePointZOffset : PedPointZOffset;
}

int NPCNode::getNodesNumber() const
{
	return initialized_ ? nodeHeader_.nodesNumber : 0;
}

void NPCNode::getHeaderInfo(uint32_t& vehicleNodes, uint32_t& pedNodes, uint32_t& naviNodes) const
{
	if (initialized_)
	{
		vehicleNodes = nodeHeader_.vehicleNodesNumber;
		pedNodes = nodeHeader_.pedNodesNumber;
		naviNodes = nodeHeader_.naviNodesNumber;
	}
	else
	{
		vehicleNodes = pedNodes = naviNodes = 0;
	}
}

int NPCNode::getNodeId() const
{
	return nodeId_;
}

uint16_t NPCNode::getLinkId() const
{
	if (!initialized_ || currentPointId_ >= pathNodes_.size())
		return 0;
	return pathNodes_[currentPointId_].linkId;
}

uint16_t NPCNode::getAreaId() const
{
	if (!initialized_ || currentPointId_ >= pathNodes_.size())
		return 0;
	return pathNodes_[currentPointId_].areaId;
}

uint16_t NPCNode::getPointId() const
{
	if (!initialized_ || currentPointId_ >= pathNodes_.size())
		return 0;
	return pathNodes_[currentPointId_].nodeId;
}

uint16_t NPCNode::getLinkCount() const
{
	if (!initialized_ || currentPointId_ >= pathNodes_.size())
		return 0;
	return static_cast<uint16_t>(pathNodes_[currentPointId_].flags & 0xF);
}

uint16_t NPCNode::getLinkId(uint16_t pointId) const
{
	if (!initialized_ || pointId >= pathNodes_.size())
		return 0;
	return pathNodes_[pointId].linkId;
}

uint16_t NPCNode::getLinkCount(uint16_t pointId) const
{
	if (!initialized_ || pointId >= pathNodes_.size())
		return 0;
	return static_cast<uint16_t>(pathNodes_[pointId].flags & 0xF);
}

bool NPCNode::getLinkTarget(uint16_t linkId, uint16_t& areaId, uint16_t& pointId) const
{
	if (!initialized_ || linkId >= linkNodes_.size())
		return false;
	areaId = linkNodes_[linkId].areaId;
	pointId = linkNodes_[linkId].nodeId;
	return true;
}

bool NPCNode::getNaviLinkTarget(uint16_t linkId, uint16_t& areaId, uint16_t& naviId) const
{
	if (!initialized_ || linkId >= naviLinks_.size())
		return false;
	const uint16_t packed = naviLinks_[linkId];
	areaId = packed >> 10;
	naviId = packed & 0x03FF;
	return true;
}

bool NPCNode::getNaviNode(uint16_t naviId, NaviNode& naviNode) const
{
	if (!initialized_ || naviId >= naviNodes_.size())
		return false;
	naviNode = naviNodes_[naviId];
	return true;
}

bool NPCNode::isVehiclePoint(uint16_t pointId) const
{
	return initialized_ && pointId < nodeHeader_.vehicleNodesNumber;
}

uint8_t NPCNode::getPathWidth() const
{
	if (!initialized_ || currentPointId_ >= pathNodes_.size())
		return 0;
	return pathNodes_[currentPointId_].pathWidth;
}

uint8_t NPCNode::getNodeType() const
{
	if (!initialized_ || currentPointId_ >= pathNodes_.size())
		return 0;
	return pathNodes_[currentPointId_].nodeType;
}

uint16_t NPCNode::getLinkPoint() const
{
	if (!initialized_ || currentLinkId_ >= linkNodes_.size())
		return 0;
	return linkNodes_[currentLinkId_].nodeId;
}

uint16_t NPCNode::getLastLinkTargetNodeId() const
{
	if (!initialized_ || currentLinkId_ >= linkNodes_.size())
		return 0;
	return linkNodes_[currentLinkId_].areaId;
}

uint16_t NPCNode::getLastLinkTargetPointId() const
{
	if (!initialized_ || currentLinkId_ >= linkNodes_.size())
		return 0;
	return linkNodes_[currentLinkId_].nodeId;
}

bool NPCNode::setLink(uint16_t linkId)
{
	if (!initialized_ || linkId >= linkNodes_.size())
	{
		return false;
	}

	currentLinkId_ = linkId;
	return true;
}

bool NPCNode::setPoint(uint16_t pointId)
{
	if (!initialized_ || pointId >= pathNodes_.size())
	{
		return false;
	}

	currentPointId_ = pointId;
	return true;
}
