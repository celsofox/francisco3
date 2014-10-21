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

        // configuration variables found in omnetpp.ini
        senderReceiverDistanceThreshold = par("senderReceiverDistanceThreshold").doubleValue();
        sendWarningInterval = par("sendWarningInterval").doubleValue();
        hostToJunctionDistanceThreshold = par("hostToJunctionDistanceThreshold").doubleValue();
        neighborLifetimeThreshold = par("neighborLifetimeThreshold").doubleValue();
        indexOfAccidentNode = par("indexOfAccidentNode").longValue();
        //

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
}


void Esbr::onBeacon(WaveShortMessage *wsm)
{
    // statistics recording
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

    // if it is a new neighbor, add it's message to the list of neighbors.
    if (isNewNeighbor) {
        neighbors.push_back(wsm->dup());
    }

    // remove neighbors that have exceeded the neighborLifetimeThreshold found in omnetpp.ini
    std::vector<int> indices;
    for (uint i = 0; i < neighbors.size(); ++i) {
        WaveShortMessage* neighbor = neighbors[i];
        if (simTime() > neighbor->getArrivalTime() + neighborLifetimeThreshold)
            indices.push_back(i);
    }

    vector<WaveShortMessage*> newNeighbors;
    for (uint i = 0; i < indices.size(); ++i) {
        newNeighbors.push_back(neighbors[i]->dup());
    }
    neighbors = newNeighbors;
    // end removing neighbors
}


void Esbr::onData(WaveShortMessage *wsm)
{
    // statistics recording
    emit(warningReceivedSignal, 1);
    emit(messageReceivedSignal, 1);
    stats->updateAllWarningsReceived();
    stats->updateAllMessagesReceived();

    // prevent originating disseminator from participating in further dissemination attempts
    if (sentMessage)
        return;

    // add message to receivedMessageMap for this warnings index
    receivedMessageMap[wsm->getTreeId()].push_back(wsm->dup());

    // is this the first time the warning was received?
    bool isNewWarningMessage = false;

    if (receivedMessageMap[wsm->getTreeId()].size() == 1) {
        isNewWarningMessage = true;
        // record statistics
        stats->updateNewWarningsReceived();
        emit(newWarningReceivedSignal, 1);
    }

    // rebroadcast only under conditions:
    if (isNewWarningMessage) {
        if ((traci->getPositionAt(simTime()).distance(wsm->getSenderPos()) > senderReceiverDistanceThreshold)
                        || (traci->getRoadId() != wsm->getWsmData())
                        || ((traci->getRoadId() == wsm->getWsmData()) && hostIsInJunction())) {
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

            std::cerr << "[INFO] ACCIDENT STARTED @simTime: " << simTime().str() << " for node: " << getParentModule()->getIndex() << endl;

            findHost()->getDisplayString().updateWith("r=16,red");
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
