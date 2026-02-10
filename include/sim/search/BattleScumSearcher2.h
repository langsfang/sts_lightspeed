//
// Created by keega on 9/17/2021.
//

#ifndef STS_LIGHTSPEED_BATTLESCUMSEARCHER2_H
#define STS_LIGHTSPEED_BATTLESCUMSEARCHER2_H

#include "sim/search/Action.h"

#include <chrono>
#include <functional>
#include <memory>
#include <random>
#include <iostream>
#include <limits>
#include <cstdint>
#include <unordered_set>

namespace sts::search {

    enum class SearchIntent {
        NONE,
        PREFER_ATTACK,
        PREFER_DEFENSE,
        PREFER_ABILITY
    };

    typedef std::function<double (const BattleContext&, const BattleContext&)> EvalFnc;

    // to find a solution to a battle with tree pruning
    struct BattleScumSearcher2 {
        class Edge;
        enum class NodeType {
            DECISION,
            CHANCE,
        };

        struct Node {
            std::int64_t simulationCount = 0;
            double evaluationSum = 0;
            NodeType nodeType = NodeType::DECISION;
            std::vector<Edge> edges;
        };

        struct Edge {
            Action action;
            Node node;
        };

        std::unique_ptr<const BattleContext> rootState;
        Node root;
        std::chrono::milliseconds startTime;

        EvalFnc evalFnc;
        double unexploredNodeValueParameter = 100.0; // only needs to be large enough to be larger than any realistic value of the quality term + the exploration term
        double explorationParameter = 3*sqrt(2);

        double bestActionValue = std::numeric_limits<double>::lowest();
        double minActionValue = std::numeric_limits<double>::max();
        int outcomePlayerHp = 0;

        bool allowPotions = false;
        SearchIntent intent = SearchIntent::NONE;

        std::vector<Action> bestActionSequence;
        std::default_random_engine randGen;

        std::vector<Node*> searchStack;
        std::vector<Action> actionStack;
        std::unordered_set<std::uint64_t> visitedStateKeys;

        explicit BattleScumSearcher2(const BattleContext &bc, EvalFnc evalFnc=&evaluateEndState);

        // public methods
        void search(int64_t simulations, long maxTimeMillis);
        void step();

        // private helpers
        void updateFromPlayout(const std::vector<Node*> &stack, const std::vector<Action> &actionStack, const BattleContext &endState);
        [[nodiscard]] bool isTerminalState(const BattleContext &bc) const;

        double evaluateEdge(const Node &parent, int edgeIdx);
        int selectBestEdgeToSearch(const Node &cur);
        int selectFirstActionForLeafNode(const Node &leafNode, const BattleContext &state);

        void playoutRandom(BattleContext &state, std::vector<Action> &actionStack);

        void enumerateActionsForNode(Node &node, const BattleContext &bc, const bool forRandom);
        void enumerateActionsForRollout(Node &node, const BattleContext &bc);
        void enumerateCardActions(Node &node, const BattleContext &bc);
        void enumeratePotionActions(Node &node, const BattleContext &bc);
        void enumerateCardSelectActions(Node &node, const BattleContext &bc);
        [[nodiscard]] std::uint64_t buildStateKey(const BattleContext &bc) const;
        [[nodiscard]] bool shouldDedupState(const BattleContext &bc) const;
        void pruneDuplicateEdges(Node &node, const BattleContext &bc);
        static double evaluateEndState(const BattleContext &rootBc, const BattleContext &bc);

        void printSearchTree(std::ostream &os, int levels);
        void printSearchStack(std::ostream &os, bool skipLast=false);
    };

    extern thread_local BattleScumSearcher2 *g_debug_scum_search;

}


#endif //STS_LIGHTSPEED_BATTLESCUMSEARCHER2_H
