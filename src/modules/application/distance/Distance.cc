#include "Distance.h"
#include <WaveShortMessage_m.h>
#include <iostream>
#include <cstring>
#include <cstdio>

using Veins::TraCIMobility;
using Veins::TraCIMobilityAccess;
using std::strcmp;
using std::sprintf;

Define_Module(Distance)

void Distance::initialize(int stage)
{
    BaseWaveApplLayer::initialize(stage);
    if (stage == 0) {

//        std::cerr << "In Distance::initialize()" << endl;

        distanceThreshold = par("distanceThreshold").doubleValue();
        indexOfAccidentNode = par("indexOfAccidentNode").longValue();
        randomRebroadcastDelay = par("randomRebroadcastDelay").doubleValue();

        traci = TraCIMobilityAccess().get(getParentModule());
        stats = FranciscoStatisticsAccess().getIfExists();
        ASSERT(stats);
        beaconReceivedSignal = registerSignal("beaconReceivedSignal");
        warningReceivedSignal = registerSignal("warningReceivedSignal");
        messageReceivedSignal = registerSignal("messageReceivedSignal");
        newWarningReceivedSignal = registerSignal("newWarningReceivedSignal");

        lastDroveAt = simTime();
        sentMessage = false;
    }
}

void Distance::receiveSignal(cComponent *source, simsignal_t signalID, cComponent::cObject *obj)
{
    Enter_Method_Silent();
    if (signalID == mobilityStateChangedSignal) {
        handlePositionUpdate(obj);
    }
//	else if (signalID == parkingStateChangedSignal) {
//		handleParkingUpdate(obj);
//	}
}


void Distance::onBeacon(WaveShortMessage *wsm)
{
//    std::cerr << "In Distance::onBeacon()" << endl;
}


void Distance::onData(WaveShortMessage *wsm)
{
//    std::cerr << "In Distance::onData()" << std::endl;

    emit(warningReceivedSignal, 1);
    emit(messageReceivedSignal, 1);
    stats->updateAllWarningsReceived();
    stats->updateAllMessagesReceived();

    // prevent originating disseminator from participating in further dissemination attempts
    if (sentMessage)
        return;

    // add message to receivedMessages
    receivedMessages[wsm->getTreeId()].push_back(wsm->dup());

    // is it a new warning message?
    if (receivedMessages[wsm->getTreeId()].size() == 1) {
        stats->updateNewWarningsReceived();
        emit(newWarningReceivedSignal, 1);

        // Cancel rebroadcast if dMin < distanceThreshold
        if (traci->getPositionAt(simTime()).distance(wsm->getSenderPos()) < distanceThreshold)
            return;

        char buf[64];
        sprintf(buf, "%ld", wsm->getTreeId());
        scheduleAt(simTime() + SimTime(randomRebroadcastDelay, SIMTIME_MS), new cMessage(buf));
    }
}

void Distance::handlePositionUpdate(cComponent::cObject *obj)
{
    // stopped for for at least 10s?
    if (traci->getSpeed() < 1) {
        if ((simTime() - lastDroveAt >= 10)
                && (!sentMessage)
                && (indexOfAccidentNode == getParentModule()->getIndex())) {

            std::cerr << "[DEBUG] ACCIDENT STARTED @simTime: " << simTime().str() << " for node: " << getParentModule()->getIndex() << endl;

            findHost()->getDisplayString().updateWith("r=16,red");
            if (!sentMessage)
                sendMessage(traci->getRoadId());
        }
    }
    else {
        lastDroveAt = simTime();
    }
}

void Distance::sendMessage(std::string blockedRoadId)
{
    sentMessage = true;

    t_channel channel = dataOnSch ? type_SCH : type_CCH;
    WaveShortMessage* wsm = prepareWSM("data", dataLengthBits, channel, dataPriority, -1,2);
    wsm->setWsmData(blockedRoadId.c_str());
    sendWSM(wsm);
}

void Distance::handleSelfMsg(cMessage *msg)
{
    if ((!strcmp(msg->getName(), "data")) || (!strcmp(msg->getName(), "beacon"))) {
        BaseWaveApplLayer::handleSelfMsg(msg);
        return;
    }
    else {          // is a "rebroadcast" timer
        const char* key = msg->getName();
        WaveShortMessages msgs = receivedMessages[atol(key)];
        double dMin = 1000000.0;

        for (uint i = 0; i < msgs.size(); ++i) {
            WaveShortMessage* m = msgs[i];
            Coord receiverPosition = traci->getPositionAt(simTime());
            Coord senderPosition = m->getSenderPos();
            double distance = senderPosition.distance(receiverPosition);
            if (distance < dMin) {
                dMin = distance;
            }
        }
        if (dMin < distanceThreshold)
            return;

        sendWSM(msgs[0]->dup());
    }
}
