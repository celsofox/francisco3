#include "Esbr.h"
#include <WaveShortMessage_m.h>
#include <iostream>

using Veins::TraCIMobility;
using Veins::TraCIMobilityAccess;
using std::list;
using std::string;

Define_Module(Esbr)

void Esbr::initialize(int stage)
{
    BaseWaveApplLayer::initialize(stage);
    if (stage == 0) {

//        std::cerr << "In Esbr::initialize()" << endl;
        senderReceiverDistanceThreshold = par("senderReceiverDistanceThreshold").doubleValue();
        sendWarningInterval = par("sendWarningInterval").doubleValue();
        hostToJunctionDistanceThreshold = par("hostToJunctionDistanceThreshold").doubleValue();
        neighborLifetimeThreshold = par("neighborLifetimeThreshold").doubleValue();
        indexOfAccidentNode = par("indexOfAccidentNode").longValue();

        traci = TraCIMobilityAccess().get(getParentModule());
        stats = FranciscoStatisticsAccess().getIfExists();
        ASSERT(stats);

        beaconReceivedSignal = registerSignal("beaconReceivedSignal");
        warningReceivedSignal = registerSignal("warningReceivedSignal");
        messageReceivedSignal = registerSignal("messageReceivedSignal");
        newWarningReceivedSignal = registerSignal("newWarningReceivedSignal");

        lastDroveAt = simTime();
        hostIsInWarningMode = false;
        sentMessage = false;
    }
    else if (stage == 1) {
        junctionIds = traci->getCommandInterface()->getJunctionIds();
        if (junctionIds.empty())
            error("[ERROR] did not get junction ids from traci server (at initialize.. stage == 1)");
    }
}

void Esbr::receiveSignal(cComponent *source, simsignal_t signalID, cComponent::cObject *obj)
{
    Enter_Method_Silent();
    if (signalID == mobilityStateChangedSignal) {
        handlePositionUpdate(obj);
    }
//	else if (signalID == parkingStateChangedSignal) {
//		handleParkingUpdate(obj);
//	}
}


void Esbr::onBeacon(WaveShortMessage *wsm)
{
//    std::cerr << "In Esbr::onBeacon()" << endl;
    // handle stats
    stats->updateAllBeaconsReceived();
    stats->updateAllMessagesReceived();
    emit(beaconReceivedSignal, 1);
    emit(messageReceivedSignal, 1);

    // if host is in warning mode no need for beacons
    if (hostIsInWarningMode)
        return;

    bool isNewNeighbor = true;

    // is this a new neighbor?
    for (uint i = 0; i < neighbors.size(); ++i) {
        WaveShortMessage* neighbor = neighbors[i];
        if (neighbor->getTreeId() == wsm->getTreeId()) {
            isNewNeighbor = false;
        }
    }

    if (isNewNeighbor) {
        neighbors.push_back(wsm->dup());
    }

    // are neighbors old?
    std::vector<int> indices;
    for (uint i = 0; i < neighbors.size(); ++i) {
        WaveShortMessage* neighbor = neighbors[i];
        if (simTime() > neighbor->getArrivalTime() + neighborLifetimeThreshold)
            indices.push_back(i);
    }

//    // remove old neighbors
//    for (uint i = 0; i < indices.size(); ++i) {
//        neighbors.erase(neighbors.begin() + i);
//    }

    vector<WaveShortMessage*> newNeighbors;
    for (uint i = 0; i < indices.size(); ++i) {
        newNeighbors.push_back(neighbors[i]->dup());
    }

    neighbors = newNeighbors;
}


void Esbr::onData(WaveShortMessage *wsm)
{
//    std::cerr << "In Esbr::onData()" << std::endl;
    // handle stats
    emit(warningReceivedSignal, 1);
    emit(messageReceivedSignal, 1);
    stats->updateAllWarningsReceived();
    stats->updateAllMessagesReceived();

    // prevent originating disseminator from participating in further dissemination attempts
    if (sentMessage)
        return;

    receivedMessageMap[wsm->getTreeId()].push_back(wsm->dup());

    std::cerr << "[DEBUG] receivedMessageMap[wsm->getTreeId()].size(): " << receivedMessageMap[wsm->getTreeId()].size() << std::endl;

    // is this the first time the wsm was received?
    bool isNewWarningMessage = false;

    if (receivedMessageMap[wsm->getTreeId()].size() == 1) {

        std::cerr << "[DEBUG] isNewWarningMessage.. node: " << getParentModule()->getIndex() << " simTime(): " << simTime() << " wsm->getTreeId(): " << wsm->getTreeId() << std::endl;
        isNewWarningMessage = true;
        stats->updateNewWarningsReceived();
        emit(newWarningReceivedSignal, 1);
    }

    // rebroadcast under conditions:
    if (isNewWarningMessage) {
        if ((traci->getPositionAt(simTime()).distance(wsm->getSenderPos()) > senderReceiverDistanceThreshold)
                        || (traci->getRoadId() != wsm->getWsmData())
                        || ((traci->getRoadId() == wsm->getWsmData()) && hostIsInJunction())) {
//            sendMessage(wsm->getWsmData());
            sendWSM(wsm->dup());
        }
    }
}

void Esbr::handlePositionUpdate(cComponent::cObject *obj)
{
    // stopped for at least 10s?
    if (traci->getSpeed() < 1) {
        if ((simTime() - lastDroveAt >= 10)
                && (!sentMessage)
                && (indexOfAccidentNode == getParentModule()->getIndex())) {

            std::cerr << "[DEBUG] ACCIDENT STARTED @simTime: " << simTime().str() << " for node: " << getParentModule()->getIndex() << endl;

            findHost()->getDisplayString().updateWith("r=16,red");
//            stats->updateNewWarningsReceived();
            if (!hostIsInWarningMode) {
                hostIsInWarningMode = true;
                sendMessage(traci->getRoadId());
                startWarningModeIntervalBroadcast();
            }
        }
    }
}

void Esbr::sendMessage(std::string blockedRoadId)
{
    t_channel channel = dataOnSch ? type_SCH : type_CCH;
    WaveShortMessage* wsm = prepareWSM("data", dataLengthBits, channel, dataPriority, -1,2);
    wsm->setWsmData(blockedRoadId.c_str());
    sendWSM(wsm);
    sentMessage = true;
}

void Esbr::handleSelfMsg(cMessage *msg)
{
    if (!strcmp(msg->getName(), "rebroadcast")) {
        sendMessage(traci->getRoadId());
        startWarningModeIntervalBroadcast();
        return;
    }
    BaseWaveApplLayer::handleSelfMsg(msg);
}


void Esbr::startWarningModeIntervalBroadcast()
{
    cMessage* msg = new cMessage("rebroadcast");
    scheduleAt(simTime() + sendWarningInterval, msg);
}


bool Esbr::hostIsInJunction()
{
    // check to see if this host is near an intersection
    bool isInIntersection = false;
    for (list<string>::iterator i = junctionIds.begin(); i != junctionIds.end(); ++i) {
        string jId = *i;
        Coord jPos = traci->getManager()->traci2omnet(traci->getCommandInterface()->getJunctionPosition(jId));
        if (traci->getPositionAt(simTime()).distance(jPos) < hostToJunctionDistanceThreshold)
            isInIntersection = true;
    }
    return isInIntersection;
}
