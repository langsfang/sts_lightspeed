#include <iostream>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <thread>
#include <sstream>

#include "combat/BattleContext.h"
#include "convert/BattleConverter.h"
#include "sim/ConsoleSimulator.h"
#include "sim/search/BattleScumSearcher2.h"
#include "constants/Potions.h"

#include <nlohmann/json.hpp>

#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
using namespace sts;
using namespace std::chrono;

void search2(search::BattleScumSearcher2 &searcher, int simulations) {
    searcher.search(simulations, 1000);
}

std::string selectedCardNameForTask(const BattleContext &bc, CardSelectTask task, int idx) {
    switch (task) {
        case CardSelectTask::CODEX:
            if (idx == 3) {
                return "skip";
            }
            return CardInstance(bc.cardSelectInfo.codexCards()[idx]).getName();
        case CardSelectTask::DISCOVERY:
            return CardInstance(bc.cardSelectInfo.discovery_Cards()[idx]).getName();
        case CardSelectTask::HOLOGRAM:
        case CardSelectTask::LIQUID_MEMORIES_POTION:
        case CardSelectTask::HEADBUTT:
        case CardSelectTask::MEDITATE:
            return bc.cards.discardPile[idx].getName();
        case CardSelectTask::EXHUME:
            return bc.cards.exhaustPile[idx].getName();
        case CardSelectTask::SECRET_TECHNIQUE:
        case CardSelectTask::SECRET_WEAPON:
        case CardSelectTask::SEEK:
            return bc.cards.drawPile[idx].getName();
        case CardSelectTask::EXHAUST_ONE:
        case CardSelectTask::FORETHOUGHT:
        case CardSelectTask::NIGHTMARE:
        case CardSelectTask::RECYCLE:
        case CardSelectTask::SETUP:
        case CardSelectTask::WARCRY:
        case CardSelectTask::ARMAMENTS:
        case CardSelectTask::DUAL_WIELD:
            return bc.cards.hand[idx].getName();
        default:
            return "";
    }
}

std::string describeAction(const search::Action &action, const BattleContext &bc) {
    if (!action.isValidAction(bc)) {
        return "{ INVALID ACTION }";
    }

    std::ostringstream os;
    switch (action.getActionType()) {
        case search::ActionType::CARD: {
            const auto &card = bc.cards.hand[action.getSourceIdx()];
            os << "play card " << action.getSourceIdx() << "(" << card.getName() << ")";
            if (card.requiresTarget()) {
                const auto &monster = bc.monsters.arr[action.getTargetIdx()];
                os << " to Monster " << action.getTargetIdx() << "(" << monster.getName() << ")";
            }
            return os.str();
        }
        case search::ActionType::POTION: {
            const auto potion = bc.potions[action.getSourceIdx()];
            if (action.getTargetIdx() == -1) {
                os << "discard potion ";
            } else {
                os << "drink potion ";
            }
            os << action.getSourceIdx() << "(" << getPotionName(potion) << ")";
            if (potionRequiresTarget(potion)) {
                const auto &monster = bc.monsters.arr[action.getTargetIdx()];
                os << " to Monster " << action.getTargetIdx() << "(" << monster.getName() << ")";
            }
            return os.str();
        }
        case search::ActionType::SINGLE_CARD_SELECT: {
            const auto task = bc.cardSelectInfo.cardSelectTask;
            const auto idx = action.getSelectIdx();
            os << "{ " << cardSelectTaskStrings[static_cast<int>(task)] << " (" << idx << ")";
            const auto name = selectedCardNameForTask(bc, task, idx);
            if (!name.empty()) {
                os << " " << name;
            }
            os << " }";
            return os.str();
        }
        case search::ActionType::MULTI_CARD_SELECT: {
            const auto task = bc.cardSelectInfo.cardSelectTask;
            const auto selected = action.getSelectedIdxs();
            os << "{ " << cardSelectTaskStrings[static_cast<int>(task)];
            if (selected.empty()) {
                os << " none";
            } else {
                for (int i = 0; i < selected.size(); ++i) {
                    const auto idx = selected[i];
                    os << " (" << idx << ") " << bc.cards.hand[idx].getName();
                    if (i + 1 < selected.size()) {
                        os << ",";
                    }
                }
            }
            os << " }";
            return os.str();
        }
        default:
            action.printDesc(os, bc);
            return os.str();
    }
}

int main() {
    std::string jsonString;
    std::getline(std::cin, jsonString);

    nlohmann::json json = nlohmann::json::parse(jsonString);
    BattleConverter converter;
    int monsterIdxMap[5];
    for (int i = 0; i < 5; ++i) {
        monsterIdxMap[i] = -1;
    }
    BattleContext baseBc = converter.convertFromJson(json, monsterIdxMap);

    milliseconds ms1 = duration_cast< milliseconds >(
        system_clock::now().time_since_epoch()
    );
    // std::cout << bc << std::endl;
    int thread_count = 12;
    search::BattleScumSearcher2 *searchers[thread_count];
    std::thread *threads[thread_count];
    for (int i = 0; i < thread_count; ++i) {
        int rngMod = i;
        BattleContext threadBc = baseBc;
        threadBc.cardRandomRng.setCounter(rngMod * 1000);
        threadBc.aiRng.setCounter(rngMod * 1000);
        threadBc.shuffleRng.setCounter(rngMod * 1000);
        threadBc.miscRng.setCounter(rngMod * 1000);
        threadBc.potionRng.setCounter(rngMod * 1000);
        searchers[i] = new search::BattleScumSearcher2(threadBc);
        threads[i] = new std::thread(search2, std::ref(*searchers[i]), 100000);
    }
    for (int i = 0; i < thread_count; ++i) {
        threads[i]->join();
    }
    milliseconds ms2 = duration_cast< milliseconds >(
        system_clock::now().time_since_epoch()
    );
    // std::cout << "took " << (ms2 - ms1).count() << "ms\n";

    // std::cout << searcher.bestActionValue << '\n';
    // std::cout << searcher.root.simulationCount << '\n';
    // searcher.bestActionSequence[0].printDesc(std::cout, bc);
    // std::cout << '\n' << bc << '\n';
    // searcher.printSearchTree(std::cout, 1);

    std::ofstream outfile;
    outfile.open("test.log", std::ios_base::app);
    outfile << "====" << std::endl;
    outfile << baseBc << std::endl;
    for (int j = 0; j < searchers[0]->root.edges.size(); ++j) {
        double evaluationSum = 0;
        int simulationCount = 0;
        for (int i = 0; i < thread_count; ++i) {
            evaluationSum += searchers[i]->root.edges[j].node.evaluationSum;
            simulationCount += searchers[i]->root.edges[j].node.simulationCount;
        }
        outfile << simulationCount << " visits / " << std::fixed << std::setprecision(3) << (evaluationSum / simulationCount) << " value for ";
        searchers[0]->root.edges[j].action.printDesc(outfile, baseBc);
        outfile << std::endl;
    }
    nlohmann::json monsterIdxMapJson = nlohmann::json::array();
    for (int i = 0; i < 5; ++i) {
        monsterIdxMapJson[i] = monsterIdxMap[i];
    }

    int bestSearcherIdx = 0;
    double bestValue = searchers[0]->bestActionValue;
    for (int i = 1; i < thread_count; ++i) {
        if (searchers[i]->bestActionValue > bestValue) {
            bestSearcherIdx = i;
            bestValue = searchers[i]->bestActionValue;
        }
    }

    BattleContext replayState = *searchers[bestSearcherIdx]->rootState;
    nlohmann::json actionDescriptions = nlohmann::json::array();
    for (const auto &action : searchers[bestSearcherIdx]->bestActionSequence) {
        actionDescriptions.push_back(describeAction(action, replayState));
        action.execute(replayState);
    }

    nlohmann::json output = nlohmann::json::object();
    output["monsterIdxMap"] = monsterIdxMapJson;
    output["actions"] = actionDescriptions;
    std::cout << output.dump() << std::endl;
    return 0;
}

#pragma clang diagnostic pop
