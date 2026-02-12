//
// Created by keega on 9/18/2021.
//

#include "sim/search/BattleScumSearcher2.h"
#include "sim/search/ExpertKnowledge.h"

#include <utility>
#include <string>
#include <memory>
#include <sstream>
#include <algorithm>
#include <tuple>

using namespace sts;

std::int64_t simulationIdx = 0; // for debugging

double getNonMinionMonsterCurHpTotal(const BattleContext &bc);

namespace sts::search {
    thread_local search::BattleScumSearcher2 *g_debug_scum_search;
}

namespace {
    constexpr std::uint64_t kHashSeed = 0x9e3779b97f4a7c15ULL;

    void hashCombine(std::uint64_t &seed, std::uint64_t value) {
        seed ^= value + kHashSeed + (seed << 6) + (seed >> 2);
    }

    struct CardKey {
        int id;
        int upgradeCount;
        int specialData;
        int cost;
        int costForTurn;
        bool freeToPlayOnce;
        bool retain;
    };

    CardKey toCardKey(const CardInstance &card) {
        return {
            static_cast<int>(card.getId()),
            card.getUpgradeCount(),
            card.specialData,
            card.cost,
            card.costForTurn,
            card.freeToPlayOnce,
            card.retain
        };
    }

    void hashCardKey(std::uint64_t &seed, const CardKey &card) {
        hashCombine(seed, static_cast<std::uint64_t>(card.id));
        hashCombine(seed, static_cast<std::uint64_t>(card.upgradeCount));
        hashCombine(seed, static_cast<std::uint64_t>(card.specialData));
        hashCombine(seed, static_cast<std::uint64_t>(card.cost));
        hashCombine(seed, static_cast<std::uint64_t>(card.costForTurn));
        hashCombine(seed, static_cast<std::uint64_t>(card.freeToPlayOnce));
        hashCombine(seed, static_cast<std::uint64_t>(card.retain));
    }

    void hashCardQueueItem(std::uint64_t &seed, const CardQueueItem &item) {
        hashCardKey(seed, toCardKey(item.card));
        hashCombine(seed, static_cast<std::uint64_t>(item.target));
        hashCombine(seed, static_cast<std::uint64_t>(item.isEndTurn));
        hashCombine(seed, static_cast<std::uint64_t>(item.triggerOnUse));
        hashCombine(seed, static_cast<std::uint64_t>(item.ignoreEnergyTotal));
        hashCombine(seed, static_cast<std::uint64_t>(item.energyOnUse));
        hashCombine(seed, static_cast<std::uint64_t>(item.freeToPlay));
        hashCombine(seed, static_cast<std::uint64_t>(item.randomTarget));
        hashCombine(seed, static_cast<std::uint64_t>(item.autoplay));
        hashCombine(seed, static_cast<std::uint64_t>(item.regretCardCount));
        hashCombine(seed, static_cast<std::uint64_t>(item.purgeOnUse));
        hashCombine(seed, static_cast<std::uint64_t>(item.exhaustOnUse));
    }

    void hashRngState(std::uint64_t &seed, const Random &rng) {
        hashCombine(seed, static_cast<std::uint64_t>(rng.counter));
        hashCombine(seed, rng.seed0);
        hashCombine(seed, rng.seed1);
    }

    void hashMonsterState(std::uint64_t &seed, const Monster &m) {
        hashCombine(seed, static_cast<std::uint64_t>(m.idx));
        hashCombine(seed, static_cast<std::uint64_t>(m.id));
        hashCombine(seed, static_cast<std::uint64_t>(m.curHp));
        hashCombine(seed, static_cast<std::uint64_t>(m.maxHp));
        hashCombine(seed, static_cast<std::uint64_t>(m.block));

        hashCombine(seed, static_cast<std::uint64_t>(m.isEscapingB));
        hashCombine(seed, static_cast<std::uint64_t>(m.halfDead));
        hashCombine(seed, static_cast<std::uint64_t>(m.escapeNext));
        hashCombine(seed, static_cast<std::uint64_t>(m.moveHistory[0]));
        hashCombine(seed, static_cast<std::uint64_t>(m.moveHistory[1]));

        hashCombine(seed, m.statusBits);
        hashCombine(seed, static_cast<std::uint64_t>(m.artifact));
        hashCombine(seed, static_cast<std::uint64_t>(m.blockReturn));
        hashCombine(seed, static_cast<std::uint64_t>(m.choked));
        hashCombine(seed, static_cast<std::uint64_t>(m.corpseExplosion));
        hashCombine(seed, static_cast<std::uint64_t>(m.lockOn));
        hashCombine(seed, static_cast<std::uint64_t>(m.mark));
        hashCombine(seed, static_cast<std::uint64_t>(m.metallicize));
        hashCombine(seed, static_cast<std::uint64_t>(m.platedArmor));
        hashCombine(seed, static_cast<std::uint64_t>(m.poison));
        hashCombine(seed, static_cast<std::uint64_t>(m.regen));
        hashCombine(seed, static_cast<std::uint64_t>(m.shackled));
        hashCombine(seed, static_cast<std::uint64_t>(m.strength));
        hashCombine(seed, static_cast<std::uint64_t>(m.vulnerable));
        hashCombine(seed, static_cast<std::uint64_t>(m.weak));
        hashCombine(seed, static_cast<std::uint64_t>(m.uniquePower0));
        hashCombine(seed, static_cast<std::uint64_t>(m.uniquePower1));
        hashCombine(seed, static_cast<std::uint64_t>(m.miscInfo));
    }

    void hashMonsterGroupState(std::uint64_t &seed, const MonsterGroup &g) {
        hashCombine(seed, static_cast<std::uint64_t>(g.monsterCount));
        hashCombine(seed, static_cast<std::uint64_t>(g.monstersAlive));
        hashCombine(seed, static_cast<std::uint64_t>(g.extraRollMoveOnTurn.to_ullong()));

        for (int i = 0; i < g.monsterCount; ++i) {
            hashMonsterState(seed, g.arr[i]);
        }
    }

    template <typename Iterator>
    void hashCardGroupInOrder(std::uint64_t &seed, Iterator begin, Iterator end) {
        for (auto it = begin; it != end; ++it) {
            hashCardKey(seed, toCardKey(*it));
        }
    }

    double evaluateActionHeuristic(const BattleContext &before, const search::Action &action) {
        BattleContext after(before);
        action.execute(after);

        const double hpDelta = static_cast<double>(after.player.curHp - before.player.curHp);
        const double blockDelta = static_cast<double>(after.player.block - before.player.block);
        const double enemyHpDelta = static_cast<double>(getNonMinionMonsterCurHpTotal(before)
                                                      - getNonMinionMonsterCurHpTotal(after));

        // return (hpDelta * 5.0) + (blockDelta * 0.5) + enemyHpDelta;
        return (hpDelta * 5.0) + (blockDelta * 0.5) + enemyHpDelta;
    }
}



search::BattleScumSearcher2::BattleScumSearcher2(const BattleContext &bc, search::EvalFnc _evalFnc)
    : rootState(new BattleContext(bc)), evalFnc(std::move(_evalFnc)), randGen(bc.seed+bc.floorNum) {
    transpositionTable.emplace(buildStateKey(*rootState), std::shared_ptr<Node>(&root, [] (Node*) {}));
}

void search::BattleScumSearcher2::search(int64_t simulations, long maxTimeMillis) {
    g_debug_scum_search = this;
    startTime = std::chrono::duration_cast< std::chrono::milliseconds >(
        std::chrono::system_clock::now().time_since_epoch()
    );

    if (isTerminalState(*rootState)) {
        auto evaluation = evaluateEndState(*rootState, *rootState);
        outcomePlayerHp = rootState->player.curHp;
        bestActionSequence = {};

        root.evaluationSum = evaluation;
        root.simulationCount = 1;
    }

    for (std::int64_t simCount = 0; simCount < simulations; ++simCount) {
        if (simCount % 100 == 0) {
            std::chrono::milliseconds curTime = std::chrono::duration_cast< std::chrono::milliseconds >(
                std::chrono::system_clock::now().time_since_epoch()
            );

            // early termination if the time limit is reached
            if ((curTime - startTime).count() >= maxTimeMillis) {
                return;
            }
        }
        step();
    }
}

void search::BattleScumSearcher2::step() {
    searchStack = {&root};
    actionStack.clear();
    BattleContext curState;
    curState = *rootState;

    while (true) {
        auto &curNode = *searchStack.back();

        if (isTerminalState(curState)) {
            updateFromPlayout(searchStack, actionStack, curState);
            return;
        }

        pruneInvalidEdgesForState(curNode, curState);

        const bool isLeaf = curNode.edges.empty();
        if (isLeaf) {

            ++simulationIdx;
            curNode.nodeType = NodeType::DECISION;
            enumerateActionsForNode(curNode, curState, false);
            pruneDuplicateEdges(curNode, curState);
            if (curNode.edges.empty()) {
                updateFromPlayout(searchStack, actionStack, curState);
                return;
            }
            const auto selectIdx = selectFirstActionForLeafNode(curNode, curState);
            auto &edgeTaken = curNode.edges[selectIdx];

            const BattleContext prevState(curState);
            edgeTaken.action.execute(curState);
            edgeTaken.node->nodeType = transitionHasRandomEvent(prevState, curState)
                                      ? NodeType::CHANCE
                                      : NodeType::DECISION;
            actionStack.push_back(edgeTaken.action);
            searchStack.push_back(edgeTaken.node.get());

            playoutRandom(curState, actionStack);
            updateFromPlayout(searchStack, actionStack, curState);
            return;

        } else {
            const auto selectIdx = selectBestEdgeToSearch(curNode);
            auto &edgeTaken = curNode.edges[selectIdx];

            const BattleContext prevState(curState);
            edgeTaken.action.execute(curState);
            edgeTaken.node->nodeType = transitionHasRandomEvent(prevState, curState)
                                      ? NodeType::CHANCE
                                      : NodeType::DECISION;
            actionStack.push_back(edgeTaken.action);
            searchStack.push_back(edgeTaken.node.get());
        }
    }
}

void search::BattleScumSearcher2::pruneInvalidEdgesForState(search::BattleScumSearcher2::Node &node,
                                                            const BattleContext &bc) {
    if (node.edges.empty()) {
        return;
    }

    std::vector<Edge> validEdges;
    validEdges.reserve(node.edges.size());
    for (auto &edge : node.edges) {
        if (edge.action.isValidAction(bc)) {
            validEdges.push_back(std::move(edge));
        }
    }

    node.edges = std::move(validEdges);
}

std::uint64_t search::BattleScumSearcher2::buildStateKey(const BattleContext &bc) const {
    std::uint64_t seed = 0;
    hashCombine(seed, static_cast<std::uint64_t>(bc.energyWasted));
    hashCombine(seed, static_cast<std::uint64_t>(bc.cardsDrawn));
    hashCombine(seed, static_cast<std::uint64_t>(bc.outcome));
    hashCombine(seed, static_cast<std::uint64_t>(bc.inputState));
    hashCombine(seed, static_cast<std::uint64_t>(bc.monsterTurnIdx));
    hashCombine(seed, static_cast<std::uint64_t>(bc.turn));
    hashCombine(seed, static_cast<std::uint64_t>(bc.isBattleOver));
    hashCombine(seed, static_cast<std::uint64_t>(bc.endTurnQueued));
    hashCombine(seed, static_cast<std::uint64_t>(bc.turnHasEnded));
    hashCombine(seed, static_cast<std::uint64_t>(bc.skipMonsterTurn));
    hashCombine(seed, static_cast<std::uint64_t>(bc.miscBits.to_ullong()));
    hashCombine(seed, static_cast<std::uint64_t>(bc.actionQueue.size));
    hashCombine(seed, static_cast<std::uint64_t>(bc.actionQueue.front));
    hashCombine(seed, static_cast<std::uint64_t>(bc.actionQueue.back));
    hashCombine(seed, static_cast<std::uint64_t>(bc.actionQueue.bits.to_ullong()));
    hashCombine(seed, static_cast<std::uint64_t>(bc.cardQueue.size));
    hashCombine(seed, static_cast<std::uint64_t>(bc.cardQueue.frontIdx));
    hashCombine(seed, static_cast<std::uint64_t>(bc.cardQueue.backIdx));

    hashCombine(seed, static_cast<std::uint64_t>(bc.cardSelectInfo.cardSelectTask));
    hashCombine(seed, static_cast<std::uint64_t>(bc.cardSelectInfo.canPickZero));
    hashCombine(seed, static_cast<std::uint64_t>(bc.cardSelectInfo.canPickAnyNumber));
    hashCombine(seed, static_cast<std::uint64_t>(bc.cardSelectInfo.pickCount));
    hashCombine(seed, static_cast<std::uint64_t>(bc.cardSelectInfo.data0));
    for (const auto &cardId : bc.cardSelectInfo.cards) {
        hashCombine(seed, static_cast<std::uint64_t>(cardId));
    }

    hashCombine(seed, static_cast<std::uint64_t>(bc.potionCount));
    hashCombine(seed, static_cast<std::uint64_t>(bc.potionCapacity));
    for (int i = 0; i < bc.potionCapacity; ++i) {
        hashCombine(seed, static_cast<std::uint64_t>(bc.potions[i]));
    }

    hashRngState(seed, bc.aiRng);
    hashRngState(seed, bc.cardRandomRng);
    hashRngState(seed, bc.miscRng);
    hashRngState(seed, bc.monsterHpRng);
    hashRngState(seed, bc.potionRng);
    hashRngState(seed, bc.shuffleRng);

    if (bc.cardQueue.size > 0) {
        int idx = bc.cardQueue.frontIdx;
        for (int i = 0; i < bc.cardQueue.size; ++i) {
            if (idx >= CardQueue::capacity) {
                idx = 0;
            }
            hashCardQueueItem(seed, bc.cardQueue.arr[idx]);
            ++idx;
        }
    }
    hashCardQueueItem(seed, bc.curCardQueueItem);

    hashCombine(seed, static_cast<std::uint64_t>(bc.cards.nextUniqueCardId));
    hashCombine(seed, static_cast<std::uint64_t>(bc.cards.handNormalityCount));
    hashCombine(seed, static_cast<std::uint64_t>(bc.cards.handPainCount));
    hashCombine(seed, static_cast<std::uint64_t>(bc.cards.strikeCount));
    hashCombine(seed, static_cast<std::uint64_t>(bc.cards.handBloodCardCount));
    hashCombine(seed, static_cast<std::uint64_t>(bc.cards.drawPileBloodCardCount));
    hashCombine(seed, static_cast<std::uint64_t>(bc.cards.discardPileBloodCardCount));

    hashCardGroupInOrder(seed, bc.cards.hand.begin(), bc.cards.hand.begin() + bc.cards.cardsInHand);
    hashCardGroupInOrder(seed, bc.cards.drawPile.begin(), bc.cards.drawPile.end());
    hashCardGroupInOrder(seed, bc.cards.discardPile.begin(), bc.cards.discardPile.end());
    hashCardGroupInOrder(seed, bc.cards.exhaustPile.begin(), bc.cards.exhaustPile.end());
    hashCardGroupInOrder(seed, bc.cards.limbo.begin(), bc.cards.limbo.end());
    hashCardGroupInOrder(seed, bc.cards.stasisCards.begin(), bc.cards.stasisCards.end());

    hashMonsterGroupState(seed, bc.monsters);

    std::ostringstream os;
    os << bc.player;
    hashCombine(seed, std::hash<std::string>{}(os.str()));

    return seed;
}

bool search::BattleScumSearcher2::shouldDedupState(const BattleContext &bc) const {
    return bc.actionQueue.size == 0
           && bc.cardQueue.size == 0
           // CARD_SELECT states are index-sensitive and can vary by transient selection context.
           // Restrict transposition merges to normal player states to avoid cross-state edge reuse.
           && bc.inputState == InputState::PLAYER_NORMAL;
}

void search::BattleScumSearcher2::pruneDuplicateEdges(search::BattleScumSearcher2::Node &node, const BattleContext &bc) {
    if (!shouldDedupState(bc)) {
        return;
    }

    for (auto &edge : node.edges) {
        BattleContext nextState(bc);
        edge.action.execute(nextState);

        if (!shouldDedupState(nextState)) {
            continue;
        }

        const auto key = buildStateKey(nextState);
        const auto [it, inserted] = transpositionTable.emplace(key, edge.node);
        if (!inserted) {
            edge.node = it->second;
        }
    }
}

void search::BattleScumSearcher2::updateFromPlayout(const std::vector<Node *> &stack, const std::vector<Action> &actionStack, const BattleContext &endState) {
    const auto evaluation = evaluateEndState(*rootState, endState);

    if (evaluation > bestActionValue) {
        bestActionSequence = actionStack;
        bestActionValue = evaluation;
        outcomePlayerHp = endState.player.curHp;
    }

    if (evaluation < minActionValue) {
        minActionValue = evaluation;
    }

    double backedUpValue = evaluation;
    for (auto it = stack.rbegin(); it != stack.rend(); ++it) {
        auto &node = *(*it);
        if (node.nodeType == NodeType::CHANCE && backedUpValue > 0) {
            backedUpValue *= chanceNodeBackpropWeight;
        }
        ++node.simulationCount;
        node.evaluationSum += backedUpValue;
    }
}

bool search::BattleScumSearcher2::transitionHasRandomEvent(const BattleContext &before, const BattleContext &after) const {
    auto rngChanged = [](const Random &lhs, const Random &rhs) {
        return lhs.counter != rhs.counter || lhs.seed0 != rhs.seed0 || lhs.seed1 != rhs.seed1;
    };

    return rngChanged(before.aiRng, after.aiRng)
           || rngChanged(before.cardRandomRng, after.cardRandomRng)
           || rngChanged(before.miscRng, after.miscRng)
           || rngChanged(before.monsterHpRng, after.monsterHpRng)
           || rngChanged(before.potionRng, after.potionRng)
           || rngChanged(before.shuffleRng, after.shuffleRng);
}

bool search::BattleScumSearcher2::isTerminalState(const BattleContext &bc) const { // maybe can optimize by making this evaluate directly if score cannot possibly be higher than best
    return bc.outcome != Outcome::UNDECIDED;
}

double search::BattleScumSearcher2::evaluateEdge(const search::BattleScumSearcher2::Node &parent, int edgeIdx) {

    const auto &edge = parent.edges[edgeIdx];

    // unexplored nodes must be assigned a sufficiently large value
    // that they are explored at a priority over any other node
    if (edge.node->simulationCount == 0) {
        return unexploredNodeValueParameter;
    }

    double qualityValue = edge.node->evaluationSum / (edge.node->simulationCount+1);

    double explorationValue = explorationParameter *
            std::sqrt(std::log(parent.simulationCount+1) / (edge.node->simulationCount+1));

    return qualityValue + explorationValue;
}

int search::BattleScumSearcher2::selectBestEdgeToSearch(const search::BattleScumSearcher2::Node &cur) {
    if (cur.edges.size() == 1) {
        return 0;
    }

    auto bestEdge = 0;
    auto bestEdgeValue = evaluateEdge(cur, bestEdge);

    for (int i = 1; i < cur.edges.size(); ++i) {
        const auto value = evaluateEdge(cur, i);
        if (value > bestEdgeValue) {
            bestEdge = i;
            bestEdgeValue = value;
        }
    }
    return bestEdge;
}

int search::BattleScumSearcher2::selectFirstActionForLeafNode(const search::BattleScumSearcher2::Node &leafNode,
                                                              const BattleContext &state) {
    if (leafNode.edges.size() == 1) {
        return 0;
    }

    std::vector<int> bestIdxs;
    double bestScore = std::numeric_limits<double>::lowest();
    for (int i = 0; i < leafNode.edges.size(); ++i) {
        const auto score = evaluateActionHeuristic(state, leafNode.edges[i].action);
        if (score > bestScore) {
            bestScore = score;
            bestIdxs.clear();
            bestIdxs.push_back(i);
        } else if (score == bestScore) {
            bestIdxs.push_back(i);
        }
    }

    auto dist = std::uniform_int_distribution<int>(0, static_cast<int>(bestIdxs.size()) - 1);
    return bestIdxs[dist(randGen)];
}

void search::BattleScumSearcher2::playoutRandom(BattleContext &state, std::vector<Action> &actionStack) {
    Node tempNode; // temp
    while (!isTerminalState(state)) {
        ++simulationIdx;

        enumerateActionsForRollout(tempNode, state);
        if (tempNode.edges.empty()) {
            std::cerr << state.seed << " " << simulationIdx << std::endl;
            std::cerr << state.monsters.arr[0].getName() << " " << state.floorNum << " " << monsterEncouterNames[static_cast<int>(state.encounter)] << std::endl;
            assert(false);
        }

        constexpr double kRandomRolloutChance = 0.2;
        auto distChance = std::uniform_real_distribution<double>(0.0, 1.0);
        int selectedIdx = 0;

        if (distChance(randGen) < kRandomRolloutChance) {
            auto dist = std::uniform_int_distribution<int>(0, static_cast<int>(tempNode.edges.size()) - 1);
            selectedIdx = dist(randGen);
        } else {
            std::vector<int> bestIdxs;
            double bestScore = std::numeric_limits<double>::lowest();
            for (int i = 0; i < tempNode.edges.size(); ++i) {
                const auto score = evaluateActionHeuristic(state, tempNode.edges[i].action);
                if (score > bestScore) {
                    bestScore = score;
                    bestIdxs.clear();
                    bestIdxs.push_back(i);
                } else if (score == bestScore) {
                    bestIdxs.push_back(i);
                }
            }
            auto dist = std::uniform_int_distribution<int>(0, static_cast<int>(bestIdxs.size()) - 1);
            selectedIdx = bestIdxs[dist(randGen)];
        }

        const auto action = tempNode.edges[selectedIdx].action;
        actionStack.push_back(action);
        action.execute(state);

        tempNode.edges.clear();
    }
}

void search::BattleScumSearcher2::enumerateActionsForNode(search::BattleScumSearcher2::Node &node,
                                                               const BattleContext &bc, const bool forRandom) {
    switch (bc.inputState) {
        case InputState::PLAYER_NORMAL:
            enumerateCardActions(node, bc);
            if (allowPotions) {
                enumeratePotionActions(node, bc);
            }

            node.edges.push_back({Action(ActionType::END_TURN)});
            break;

        case InputState::CARD_SELECT:
            enumerateCardSelectActions(node, bc);
            break;

        default:
#ifdef sts_asserts
            std::cerr << "enumerateActionsForNode: invalid input state: " << static_cast<int>(bc.inputState) << std::endl;
            assert(false);
#endif
            break;
    }

#ifdef sts_print_debug
    std::cout << "{ (" << node.edges.size() << ") ";
    for (int i = 0; i < node.edges.size(); ++i) {
        node.edges[i].action.printDesc(std::cout, bc) << ", ";
    }
    std::cout << " }" << std::endl;
#endif
}

void search::BattleScumSearcher2::enumerateActionsForRollout(search::BattleScumSearcher2::Node &node,
                                                             const BattleContext &bc) {
    node.edges.clear();
    switch (bc.inputState) {
        case InputState::PLAYER_NORMAL:
            enumerateCardActions(node, bc);
            node.edges.push_back({Action(ActionType::END_TURN)});
            break;

        case InputState::CARD_SELECT:
            enumerateCardSelectActions(node, bc);
            break;

        default:
#ifdef sts_asserts
            std::cerr << "enumerateActionsForRollout: invalid input state: " << static_cast<int>(bc.inputState) << std::endl;
            assert(false);
#endif
            break;
    }
}
void search::BattleScumSearcher2::enumerateCardActions(search::BattleScumSearcher2::Node &node,
                                                            const BattleContext &bc) {
    if (!bc.isCardPlayAllowed()) {
        return;
    }

    struct CardOption {
        int handIdx;
        int playOrdering;
        int intentPriority;
    };

    fixed_list<CardOption, 10> playableHandIdxs;
    for (int handIdx = 0; handIdx < bc.cards.cardsInHand; ++handIdx) {
        const auto &c = bc.cards.hand[handIdx];
        if (!c.canUseOnAnyTarget(bc)) {
            continue;
        }

        bool isUniqueAction = true;

        // this doesn't make any sense. duplicate cards can appear in more locations than just next to each other
        // this only handles like literally just dual wield on the turn it is played
        if (handIdx > 0) {
            const auto &lastCard = bc.cards.hand[handIdx-1];

            bool isEqualToLastCard = c.id == lastCard.id &&
                    c.getUpgradeCount() == lastCard.getUpgradeCount() &&
                    // both should be less than deck size c.uniqueId < bc.cards.deck
                    c.costForTurn == lastCard.costForTurn &&
                    c.cost == lastCard.cost &&
                    c.freeToPlayOnce == lastCard.freeToPlayOnce &&
                    c.specialData == lastCard.specialData;

            if (isEqualToLastCard) {
                isUniqueAction = false;
            }
        }

        if (isUniqueAction) {
            // this is being called a *lot* maybe swap it for a lookup table instead of a switch statement
            int intentPriority = 0;
            if (intent != SearchIntent::NONE) {
                const auto cardType = c.getType();
                const bool matchesIntent =
                        (intent == SearchIntent::PREFER_ATTACK && cardType == CardType::ATTACK) ||
                        (intent == SearchIntent::PREFER_DEFENSE && cardType == CardType::SKILL) ||
                        (intent == SearchIntent::PREFER_ABILITY && cardType == CardType::POWER);
                intentPriority = matchesIntent ? 0 : 1;
            }
            playableHandIdxs.push_back({handIdx, search::Expert::getPlayOrdering(c.getId()), intentPriority});
        }
    }

    std::sort(playableHandIdxs.begin(), playableHandIdxs.end(), [](const auto &a, const auto &b) {
        if (a.intentPriority != b.intentPriority) {
            return a.intentPriority < b.intentPriority;
        }
        return a.playOrdering < b.playOrdering;
    });

    for (const auto &option : playableHandIdxs) {
        const auto handIdx = option.handIdx;
        const auto &c = bc.cards.hand[handIdx];

        if (c.requiresTarget()) {
            for (int tIdx = bc.monsters.monsterCount-1; tIdx >= 0; --tIdx) {
                if (!bc.monsters.arr[tIdx].isTargetable()) {
                    continue;
                }
                node.edges.push_back({Action(ActionType::CARD, handIdx, tIdx)});
            }
        } else {
            node.edges.push_back({Action(ActionType::CARD, handIdx)});
        }
    }

}

void search::BattleScumSearcher2::enumeratePotionActions(search::BattleScumSearcher2::Node &node,
                                                              const BattleContext &bc) {

    const auto hasValidTarget = bc.monsters.getTargetableCount() > 0;

    int foundPotions = 0;
    for (int pIdx = 0; pIdx < bc.potionCapacity; ++pIdx) {

        const auto p = bc.potions[pIdx];
        if (p == Potion::EMPTY_POTION_SLOT) {
            continue;
        }
        ++foundPotions;

        // fairy potions cannot be used directly
        // TODO: smoke bombs are also not implemented lol
        if (p == Potion::FAIRY_POTION || p == Potion::SMOKE_BOMB) {
            continue;
        }

        // if the potion requires a valid target and there are none, it cannot be used
        if (potionRequiresTarget(p) && !hasValidTarget) {
            continue;
        }

        // otherwise enumerate all valid ways to use the potion
        if (!potionRequiresTarget(p)) {
            // non-targeted potions have one use action
            node.edges.push_back({Action(ActionType::POTION, pIdx)});
        } else {
            // targeted potions have one use action per valid monster target
            for (int tIdx = 0; tIdx < bc.monsters.monsterCount; ++tIdx) {
                if (bc.monsters.arr[tIdx].isTargetable()) {
                    node.edges.push_back({Action(ActionType::POTION, pIdx, tIdx)});
                }
            }
        }
    }
}

template <typename ForwardIt>
void setupCardOptionsHelper(search::BattleScumSearcher2::Node &node, const ForwardIt begin, const ForwardIt end, const std::function<bool(const CardInstance &)> &p= nullptr) {
    for (int i = 0; begin+i != end; ++i) {
        const auto &c = begin[i];
        if (!p || (p(c))) {
            node.edges.push_back(
                    {search::Action(search::ActionType::SINGLE_CARD_SELECT, i)}
                );
        }
    }
}

void search::BattleScumSearcher2::enumerateCardSelectActions(search::BattleScumSearcher2::Node &node,
                                                                  const BattleContext &bc) {

    switch (bc.cardSelectInfo.cardSelectTask) {
        case CardSelectTask::ARMAMENTS:
            setupCardOptionsHelper( node, bc.cards.hand.begin(), bc.cards.hand.begin() + bc.cards.cardsInHand,
                                    [] (const CardInstance &c) { return c.canUpgrade(); });
            break;

        case CardSelectTask::CODEX:
            for (int i = 0; i < 4; ++i) { // i -> 3 action means skip
                node.edges.push_back({Action(search::ActionType::SINGLE_CARD_SELECT, i)});
            }
            break;

        case CardSelectTask::DISCOVERY:
            for (int i = 0; i < 3; ++i) {
                node.edges.push_back({Action(search::ActionType::SINGLE_CARD_SELECT, i)});
            }
            break;

        case CardSelectTask::DUAL_WIELD:
            setupCardOptionsHelper( node, bc.cards.hand.begin(), bc.cards.hand.begin() + bc.cards.cardsInHand,
                                    [] (const CardInstance &c) {
                                        return c.getType() == CardType::POWER || c.getType() == CardType::ATTACK;
                                    });
            break;

        case CardSelectTask::EXHUME:
            setupCardOptionsHelper(node, bc.cards.exhaustPile.begin(), bc.cards.exhaustPile.end(),
                                   [](const auto &c) { return c.getId() != CardId::EXHUME; });
            break;

        case CardSelectTask::EXHAUST_ONE:
            setupCardOptionsHelper(node, bc.cards.hand.begin(), bc.cards.hand.begin() + bc.cards.cardsInHand);
            break;

        case CardSelectTask::FORETHOUGHT:
        case CardSelectTask::WARCRY:
            setupCardOptionsHelper(node, bc.cards.hand.begin(), bc.cards.hand.begin() + bc.cards.cardsInHand);
            break;

        case CardSelectTask::HEADBUTT:
        case CardSelectTask::LIQUID_MEMORIES_POTION:
            setupCardOptionsHelper(node, bc.cards.discardPile.begin(), bc.cards.discardPile.end());
            break;

        case CardSelectTask::SECRET_TECHNIQUE:
            setupCardOptionsHelper(node, bc.cards.drawPile.begin(), bc.cards.drawPile.end(),
                                    [] (const CardInstance &c) {
                                        return c.getType() == CardType::SKILL;
                                    });
            break;

        case CardSelectTask::SECRET_WEAPON:
            setupCardOptionsHelper(node, bc.cards.drawPile.begin(), bc.cards.drawPile.end(),
                                    [] (const CardInstance &c) {
                                        return c.getType() == CardType::ATTACK;
                                    });
            break;

        case CardSelectTask::EXHAUST_MANY:
        case CardSelectTask::GAMBLE:
            // just dont deal with this right now
            node.edges.push_back({search::Action(search::ActionType::MULTI_CARD_SELECT, 0)});
            break;

        default:
#ifdef sts_asserts
            assert(false);
#endif
            break;
    }
}

double getMonsterHpScale(const Monster &m) {
    // the HP amounts of splitting enemies need to be inflated in order to ensure the evaluation function
    // correctly recognizes that splitting the enemies constitutes progress towards a successful resolution
    // to the fight

    // this is necessary because otherwise the evaluation function will believe that a slime boss at 80/140 HP is a better
    // state than two large slimes both at 60/60 HP since that is 120HP total vs 80HP so the hitpoints of the bigger splitting
    // enemy need to be valued at twice the value of the hitpoints of the enemy they split into - the factor of 2 representing
    // the fact that it splits into two smaller enemies with the same HP amount as the large enemy had 
    switch (m.id) {
        case MonsterId::SLIME_BOSS:
            return 4.0;
        case MonsterId::ACID_SLIME_L:
        case MonsterId::SPIKE_SLIME_L:
            return 2.0;
        default:
            return 1.0;
    }
}

double getNonMinionMonsterCurHpTotal(const BattleContext &bc) {
    int curHpTotal = 0;

    for (int i = 0; i < bc.monsters.monsterCount; ++i) {
        const auto &m = bc.monsters.arr[i];
        if (!m.hasStatus<MS::MINION>() && m.id != sts::MonsterId::INVALID) {
            curHpTotal += m.curHp * getMonsterHpScale(m);
            if (m.id == sts::MonsterId::AWAKENED_ONE && !m.miscInfo) { // is awakened one stage 1 // todo change to status
                curHpTotal += m.maxHp * getMonsterHpScale(m);
            }
        }
    }

    return curHpTotal;
}

double getNonMinionMonsterMaxHpTotal(const BattleContext &bc) {
    int maxHpTotal = 0;

    for (int i = 0; i < bc.monsters.monsterCount; ++i) {
        const auto &m = bc.monsters.arr[i];
        if (!m.hasStatus<MS::MINION>() && m.id != sts::MonsterId::INVALID) {
            maxHpTotal += m.maxHp * getMonsterHpScale(m);
            if (m.id == sts::MonsterId::AWAKENED_ONE) {
                maxHpTotal += m.maxHp * getMonsterHpScale(m);
            }
        }
    }

    return maxHpTotal;
}

double search::BattleScumSearcher2::evaluateEndState(const BattleContext &rootBc, const BattleContext &bc) {
    // gives end state values normalized to the range (-1.0, 1.0)
    if (bc.outcome == Outcome::PLAYER_VICTORY) {
        // produces winning scores in the range (0.0, 1.0)
        return 1.*bc.player.curHp / rootBc.player.curHp; 
        // return bc.player.curHp / 100.0f; 
    } else {
        return -1;
        double curHpTotal = getNonMinionMonsterCurHpTotal(bc);
        double maxHpTotal = getNonMinionMonsterMaxHpTotal(rootBc);
        double hpRatio = 0.0;
        if (maxHpTotal != 0.0) {
            hpRatio = curHpTotal / maxHpTotal;
        }
        // produces losing scores in the range (-1.0, 0.0)
        return -hpRatio;
    }
}

struct LayerStruct {
    const search::BattleScumSearcher2::Node *node;
    BattleContext *bc;
    int edgeIdx;
};

typedef std::pair<search::BattleScumSearcher2::Edge, std::unique_ptr<const BattleContext>> EdgeInfo;

std::vector<EdgeInfo> getEdgesForLayer(const search::BattleScumSearcher2 &s, int layerNum) {
    if (layerNum <= 0) {
        return {};
    }

    std::vector<EdgeInfo> layerEdges;

    std::vector<LayerStruct> curStack { {&s.root, new BattleContext(*s.rootState), 0} };

    while (!curStack.empty()) {
        if (curStack.size() == layerNum) {
            for (const auto &edge : curStack.back().node->edges) {
                layerEdges.emplace_back(edge, new BattleContext(*curStack.back().bc));
            }
        }

       // curStack size less than layerNum
       const bool visitedAll = curStack.back().edgeIdx >= curStack.back().node->edges.size();
       if (visitedAll || curStack.size() == layerNum) {
           delete curStack.back().bc;
           curStack.pop_back();
           continue;
       }

        // visit next edge
        auto &nextIdx = curStack.back().edgeIdx;
        const auto action = curStack.back().node->edges[nextIdx].action;

        BattleContext bc(*curStack.back().bc);
        action.execute(bc);

        curStack.push_back( {curStack.back().node->edges[nextIdx++].node.get(), new BattleContext(bc), 0} );
    }

    return layerEdges;
}

void search::BattleScumSearcher2::printSearchTree(std::ostream &os, int levels) {
    std::vector<std::vector<EdgeInfo>> layerEdges;
    for (int depth = 1; depth <= levels; ++depth) {
        layerEdges.push_back(getEdgesForLayer(*this, depth));
    }

//    auto maxIt = std::max(layerEdges.begin(), layerEdges.end(), [](auto a, auto b) { return a->size() < b->size(); });
//    if (maxIt == layerEdges.end()) {
//        return;
//    }
//    // maxIt points to something
//    const auto maxSize = maxIt->size();
//    constexpr auto edgeWidth = 30;

    for (int depth = 0; depth < levels; ++depth) {
        for (const auto &x : layerEdges[depth]) {
            os << "(" << x.first.node->simulationCount << ")";
            x.first.action.printDesc(os, *x.second) << "\t";
        }
        std::cout << '\n';
    }

}

void search::BattleScumSearcher2::printSearchStack(std::ostream &os, bool skipLast) {
    for (int i = 0; i < actionStack.size(); ++i) {
        const auto &a = actionStack[i];
        os << std::hex << a.bits << '\n';
    }

    os.flush();

//    BattleContext curBc = *rootState;
//    os << "explorationParameter: " << explorationParameter << '\n';
//    os << "bestActionValue: " << bestActionValue << '\n';
//    os << "minActionValue: " << minActionValue << '\n';
//    os << "outcomePlayerHp: " << outcomePlayerHp << '\n';
//    os << "root node:\n";
//    os << curBc << "\n";
//
//    for (int i = 0; i < actionStack.size(); ++i) {
//        if (i < searchStack.size()) {
//            const auto &n = searchStack[i];
//            os << i << " nodeSearched: " << n->simulationCount << " { ";
//            for (const auto &edge : n->edges) {
//                os << "(" << edge.node.simulationCount << ")";
//                edge.action.printDesc(os, curBc) << " ";
//            }
//            os << "}\n";
//        }
//
//        const auto &a = actionStack[i];
//        os << i << " actionTaken: ";
//        a.printDesc(os, curBc) << '\n';
//
//        if (skipLast && (i + 1 >= actionStack.size())) {
//            break;
//        }
//
//        a.execute(curBc);
//        os << curBc << '\n';
//    }
//
//    os.flush();
}
