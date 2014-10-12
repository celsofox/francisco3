#include "Counter.h"
#include <WaveShortMessage_m.h>
#include <iostream>
#include <cstdio>
#include <cstring>

using Veins::TraCIMobility;
using Veins::TraCIMobilityAccess;
using std::sprintf;

Define_Module(Counter)

void Counter::initialize(int stage)
{
    BaseWaveApplLayer::initialize(stage);
    if (stage == 0) {

//        std::cerr << "In Counter::initialize()" << endl;

        counterThreshold = par("counterThreshold").longValue();
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
//        isRepeat = false;
    }
}

void Counter::receiveSignal(cComponent *source, simsignal_t signalID, cComponent::cObject *obj)
{
    Enter_Method_Silent();
    if (signalID == mobilityStateChangedSignal) {
        handlePositionUpdate(obj);
    }
//	else if (signalID == parkingStateChangedSignal) {
//		handleParkingUpdate(obj);
//	}
}


void Counter::onBeacon(WaveShortMessage *wsm)
{
//    std::cerr << "In Counter::onBeacon()" << endl;
}


void Counter::onData(WaveShortMessage *wsm)
{
//    std::cerr << "In Counter::onData()" << std::endl;
    emit(warningReceivedSignal, 1);
    emit(messageReceivedSignal, 1);
    stats->updateAllWarningsReceived();
    stats->updateAllMessagesReceived();

    receivedMessages[wsm->getTreeId()].push_back(wsm->dup());

    // is it a new warning message?
    if (receivedMessages[wsm->getTreeId()].size() == 1) {
        stats->updateNewWarningsReceived();
        emit(newWarningReceivedSignal, 1);

        char buf[64];
        sprintf(buf, "%ld", wsm->getTreeId());
        scheduleAt(simTime() + SimTime(randomRebroadcastDelay, SIMTIME_MS), new cMessage(buf));
    }
}

void Counter::handlePositionUpdate(cComponent::cObject *obj)
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

void Counter::sendMessage(std::string blockedRoadId)
{
    t_channel channel = dataOnSch ? type_SCH : type_CCH;
    WaveShortMessage* wsm = prepareWSM("data", dataLengthBits, channel, dataPriority, -1,2);
    wsm->setWsmData(blockedRoadId.c_str());
    sendWSM(wsm);

    sentMessage = true;
}

void Counter::handleSelfMsg(cMessage *msg)
{
    if ((!strcmp(msg->getName(), "data")) || (!strcmp(msg->getName(), "beacon"))) {
        BaseWaveApplLayer::handleSelfMsg(msg);
        return;
    }
    else {          // IS A REBROADCAST
        if (receivedMessages[atol(msg->getName())].size() >= (uint)counterThreshold)
            return;
        sendWSM(receivedMessages[atol(msg->getName())][0]->dup());
    }
}


