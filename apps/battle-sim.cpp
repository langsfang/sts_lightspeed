#include <iostream>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <thread>
#include <fstream>
#include <sstream>
#include <string>


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
    searcher.search(simulations, 10000);
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

int getGameTargetIdx(const int *monsterIdxMap, int targetIdx) {
    if (targetIdx < 0 || targetIdx >= 5 || monsterIdxMap == nullptr) {
        return targetIdx;
    }
    return monsterIdxMap[targetIdx];
}

std::string describeAction(const search::Action &action, const BattleContext &bc, const int *monsterIdxMap) {
    if (!action.isValidAction(bc)) {
        return "{ INVALID ACTION }";
    }

    std::ostringstream os;
    switch (action.getActionType()) {
        case search::ActionType::CARD: {
            const auto &card = bc.cards.hand[action.getSourceIdx()];
            os << "play " << action.getSourceIdx() + 1;
            if (card.requiresTarget()) {
                // const auto &monster = bc.monsters.arr[action.getTargetIdx()];
                os << " " << getGameTargetIdx(monsterIdxMap, action.getTargetIdx());
            }
            return os.str();
        }
        case search::ActionType::POTION: {
            const auto potion = bc.potions[action.getSourceIdx()];
            if (action.getTargetIdx() == -1) {
                os << "potion discard ";
            } else {
                os << "potion use ";
            }
            os << action.getSourceIdx();
            if (potionRequiresTarget(potion)) {
                // const auto &monster = bc.monsters.arr[action.getTargetIdx()];
                os << " " << getGameTargetIdx(monsterIdxMap, action.getTargetIdx());
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
        case search::ActionType::END_TURN: {
            os << "end";
            return os.str();
        }
        default:
            action.printDesc(os, bc);
            return os.str();
    }
}

int main(int argc, char *argv[]) {
    std::string jsonString;
    std::ifstream file(argv[1]);
    if (file) {
        std::ostringstream ss;
        ss << file.rdbuf();
        jsonString = ss.str();
    }

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

    int bestActionIdx = 0;
    float bestScore = -1.;
    for (int j = 0; j < searchers[0]->root.edges.size(); ++j) {
        double evaluationSum = 0;
        int simulationCount = 0;
        for (int i = 0; i < thread_count; ++i) {
            evaluationSum += searchers[i]->root.edges[j].node.evaluationSum;
            simulationCount += searchers[i]->root.edges[j].node.simulationCount;
        }
        double score = evaluationSum/(1.0*simulationCount);
        outfile <<j <<":"<< simulationCount << " visits / " << std::fixed << std::setprecision(5) << score << " value for ";
        if(bestScore < score) {
            bestActionIdx = j;
            bestScore = score;
        }

        searchers[0]->root.edges[j].action.printDesc(outfile, baseBc);
        outfile << std::endl;
    }
    nlohmann::json monsterIdxMapJson = nlohmann::json::array();
    for (int i = 0; i < 5; ++i) {
        monsterIdxMapJson[i] = monsterIdxMap[i];
    }

    nlohmann::json actionDescriptions = nlohmann::json::array();
    auto action = searchers[0]->root.edges[bestActionIdx].action;
    outfile <<"choose:\t"<<bestActionIdx<<"\n";
    action.printDesc(outfile, baseBc);
    actionDescriptions.push_back(describeAction(action, baseBc, monsterIdxMap));

    action.execute(baseBc);

    // search the next action if it's card selection
    // TODO

    nlohmann::json output = nlohmann::json::object();
    output["monsterIdxMap"] = monsterIdxMapJson;
    output["actions"] = actionDescriptions;
    std::cout << output.dump() << std::endl;
    return 0;
}

#pragma clang diagnostic pop
