/*
 *  This Source Code Form is subject to the terms of the Mozilla Public License,
 *  v. 2.0. If a copy of the MPL was not distributed with this file, You can
 *  obtain one at http://mozilla.org/MPL/2.0/.
 *
 *  The original code is copyright (c) 2025, open.mp team and contributors.
 */

#include "node.hpp"
#include "../NPC/npc.hpp"
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

	file.close();

	if (!pathNodes_.empty())
	{
		currentPointId_ = 0;
	}

	// Index navi nodes by the path node they point at. Cross-area navi entries are skipped:
	// their target ids belong to another NODES file and edge lookups here are same-area only.
	naviByTargetPoint_.clear();
	for (uint16_t i = 0; i < naviNodes_.size(); ++i)
	{
		const NaviNode& naviNode = naviNodes_[i];
		if (naviNode.areaId == nodeId_ && naviNode.nodeId < pathNodes_.size())
		{
			naviByTargetPoint_[naviNode.nodeId].push_back(i);
		}
	}

	initialized_ = true;
	return true;
}

uint16_t NPCNode::process(NPC* npc, uint16_t pointId, uint16_t lastPoint, uint16_t& currentLinkId)
{
	if (!initialized_)
	{
		return 0;
	}

	bool linkRead = false;

	setPoint(pointId);

	uint16_t startLink = getLinkId();
	uint16_t linkCount = getLinkCount();
	uint8_t attempts = 0;
	uint16_t linkId = startLink;

	while (true)
	{
		do
		{
			attempts++;
			if (attempts > 10)
			{
				return 0;
			}

			if (linkCount > 0)
			{
				linkId = startLink + (rand() % linkCount);
			}
			else
			{
				linkId = startLink;
			}

			linkRead = setLink(linkId);
		} while (!linkRead || (linkId < linkNodes_.size() && linkNodes_[linkId].nodeId == lastPoint && linkCount > 1));

		if (linkId >= linkNodes_.size())
		{
			return 0;
		}

		const LinkNode& currentLink = linkNodes_[linkId];
		currentLinkId = linkId;

		if (currentLink.areaId != nodeId_)
		{
			if (currentLink.areaId != 65535)
			{
				currentLinkId_ = linkId;
				return 0xFFFF;
			}
			else
			{
				return 0;
			}
		}
		else
		{
			currentLinkId_ = linkId;
			npc->updateNodePoint(currentLink.nodeId);
			return currentLink.nodeId;
		}
	}

	return 0;
}

uint16_t NPCNode::processNodeChange(NPC* npc, uint16_t targetPointId)
{
	if (!initialized_ || targetPointId >= pathNodes_.size())
	{
		return 0;
	}

	currentPointId_ = targetPointId;
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
		static_cast<float>(pathNode.positionZ) / 8.0f + 1.2f);
	return normalPosition;
}

Vector3 NPCNode::getPositionOf(uint16_t pointId) const
{
	if (!initialized_ || pointId >= pathNodes_.size())
	{
		return Vector3(0.0f, 0.0f, 0.0f);
	}

	const PathNode& pathNode = pathNodes_[pointId];
	return Vector3(
		static_cast<float>(pathNode.positionX) / 8.0f,
		static_cast<float>(pathNode.positionY) / 8.0f,
		static_cast<float>(pathNode.positionZ) / 8.0f);
}

NodeEdgeLaneInfo NPCNode::getEdgeLaneInfo(uint16_t fromPointId, uint16_t toPointId) const
{
	NodeEdgeLaneInfo info;
	if (!initialized_ || fromPointId >= pathNodes_.size() || toPointId >= pathNodes_.size())
	{
		return info;
	}

	const Vector3 fromPosition = getPositionOf(fromPointId);
	const Vector3 toPosition = getPositionOf(toPointId);
	const float midX = (fromPosition.x + toPosition.x) * 0.5f;
	const float midY = (fromPosition.y + toPosition.y) * 0.5f;
	const float edgeDx = toPosition.x - fromPosition.x;
	const float edgeDy = toPosition.y - fromPosition.y;
	// A navi node sits on its edge, so the matching candidate must lie near this edge's
	// midpoint; anything farther than the edge itself (plus slack) belongs to another edge.
	const float maxDistanceSq = (edgeDx * edgeDx + edgeDy * edgeDy) + 25.0f;

	float bestDistanceSq = maxDistanceSq;
	// Each undirected edge has one navi node, pointing at one of the two endpoints. Its
	// bits 11-13 count lanes running in its own direction, bits 8-10 the oncoming lanes.
	const auto consider = [&](uint16_t targetPointId, bool naviPointsAlongTravel)
	{
		auto it = naviByTargetPoint_.find(targetPointId);
		if (it == naviByTargetPoint_.end())
		{
			return;
		}
		for (uint16_t naviIndex : it->second)
		{
			const NaviNode& naviNode = naviNodes_[naviIndex];
			const float dx = static_cast<float>(naviNode.positionX) / 8.0f - midX;
			const float dy = static_cast<float>(naviNode.positionY) / 8.0f - midY;
			const float distanceSq = dx * dx + dy * dy;
			if (distanceSq < bestDistanceSq)
			{
				bestDistanceSq = distanceSq;
				const int lanesWithNavi = static_cast<int>((naviNode.flags >> 11) & 0x7);
				const int lanesAgainstNavi = static_cast<int>((naviNode.flags >> 8) & 0x7);
				info.forwardLanes = naviPointsAlongTravel ? lanesWithNavi : lanesAgainstNavi;
				info.oncomingLanes = naviPointsAlongTravel ? lanesAgainstNavi : lanesWithNavi;
				info.found = true;
			}
		}
	};
	consider(toPointId, true);
	consider(fromPointId, false);
	return info;
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