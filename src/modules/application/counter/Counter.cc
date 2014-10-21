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

        // configurable variables in omnetpp.ini
        counterThreshold = par("counterThreshold").longValue();
        indexOfAccidentNode = par("indexOfAccidentNode").longValue();
        randomRebroadcastDelay = par("randomRebroadcastDelay").doubleValue();
        //

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
    // Beacons are not used in this algorithm
}


void Counter::onData(WaveShortMessage *wsm)
{
    // statistics recording
    emit(warningReceivedSignal, 1);
    emit(messageReceivedSignal, 1);
    stats->updateAllWarningsReceived();
    stats->updateAllMessagesReceived();

    // prevent originating disseminator from participating in further dissemination attempts
    if (sentMessage)
        return;

    // add the new message to storage
    receivedMessages[wsm->getTreeId()].push_back(wsm->dup());

    // is it a new warning message?
    if (receivedMessages[wsm->getTreeId()].size() == 1) {
        // statistics recording
        stats->updateNewWarningsReceived();
        emit(newWarningReceivedSignal, 1);

        // add a random waiting period before proceeding. Please see:
        //     * onSelfMsg for continuation.
        //     * .randomBroadcastDelay configuration in omnetpp.ini
        char buf[64];
        sprintf(buf, "%ld", wsm->getTreeId());
        // scheduleAt sends messege to self (see handleSelfMsg() below and randomRebroadcastDelay in omnetpp.ini
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

            std::cerr << "[INFO] ACCIDENT STARTED @simTime: " << simTime().str() << " for node: " << getParentModule()->getIndex() << endl;

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
    // for "data" and "beacon" self messages
    if ((!strcmp(msg->getName(), "data")) || (!strcmp(msg->getName(), "beacon"))) {
        BaseWaveApplLayer::handleSelfMsg(msg);
        return;
    }
    else {          // for "rebroadcast" self messages
        // if the number of times a warning message is received exceeds the counterThreshold
        // configuration variable, do not rebroadcast.
        if (receivedMessages[atol(msg->getName())].size() >= (uint)counterThreshold)
            return;
        // if greater than threshold.. rebroadcast.
        sendWSM(receivedMessages[atol(msg->getName())][0]->dup());
    }
}


