#pragma once
// The device tree and its builder from a .mrsd definition (spec 04 §1-§3).
#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "mrs/device/DeviceNode.h"
#include "mrs/device/DeviceRegistry.h"
#include "mrs/protocol/Record.h"

namespace MRS {
	namespace Device {
		// True when `ref` names the node with full name `full`: the full name itself or a '.'-suffix of it
		// (spec 16 §5).
		bool NameMatches(const std::string& full, const std::string& ref);

		class DeviceTree {
		public:
			DeviceTree() = default;
			explicit DeviceTree(std::unique_ptr<HeadNode> head) : head_(std::move(head)) {}

			HeadNode& Head() const { return *head_; }
			bool Empty() const { return !head_; }
			// The node a reference names (spec 16 §5): its full name, or a '.'-suffix that matches exactly one
			// node. Null when no node or several match.
			DeviceNode* Find(const std::string& ref) const;
			// Every node the reference matches: the exact full name alone if there is one, else all suffix matches.
			std::vector<DeviceNode*> Matches(const std::string& ref) const;
			// Every node, depth first, the head first.
			std::vector<DeviceNode*> Nodes() const;

		private:
			std::unique_ptr<HeadNode> head_;
		};

		class TreeBuilder {
		public:
			explicit TreeBuilder(const DeviceRegistry& registry) : registry_(registry) {}

			// Builds the tree: one top-level D_H; every node created through the registry with a kind that
			// matches its code; K and P_R overrides; complex nodes expanded; joint rules and unique names
			// checked. Throws BuildError.
			DeviceTree Build(const Protocol::Document& definition) const;
			DeviceTree Build(std::string_view mrsd_text) const;

		private:
			std::unique_ptr<DeviceNode> BuildNode(const Protocol::Record& rec) const;
			const DeviceRegistry& registry_;
		};
	}
}
