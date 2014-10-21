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

        // configuration variables found in omnetpp.ini
        distanceThreshold = par("distanceThreshold").doubleValue();
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

void Distance::receiveSignal(cComponent *source, simsignal_t signalID, cComponent::cObject *obj)
{
    Enter_Method_Silent();
    if (signalID == mobilityStateChangedSignal) {
        handlePositionUpdate(obj);
    }
}


void Distance::onBeacon(WaveShortMessage *wsm)
{
    // Not used for this algorithm
}


void Distance::onData(WaveShortMessage *wsm)
{
    // statistics recording
    emit(warningReceivedSignal, 1);
    emit(messageReceivedSignal, 1);
    stats->updateAllWarningsReceived();
    stats->updateAllMessagesReceived();

    // prevent originating disseminator from participating in further dissemination attempts
    if (sentMessage)
        return;

    // add message to receivedMessages storage
    receivedMessages[wsm->getTreeId()].push_back(wsm->dup());

    // is it a new warning message?
    if (receivedMessages[wsm->getTreeId()].size() == 1) {
        // statistics recording
        stats->updateNewWarningsReceived();
        emit(newWarningReceivedSignal, 1);

        // Cancel rebroadcast if dMin < distanceThreshold
        if (traci->getPositionAt(simTime()).distance(wsm->getSenderPos()) < distanceThreshold)
            return;

        // induce a random period waiting time, as per white paper
        char buf[64];
        sprintf(buf, "%ld", wsm->getTreeId());
        // see: onSelfMsg() below and randomRebroadcastDelay configuration variable in omnetpp.ini
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
    // for "normally sent" self messages
    if ((!strcmp(msg->getName(), "data")) || (!strcmp(msg->getName(), "beacon"))) {
        BaseWaveApplLayer::handleSelfMsg(msg);
        return;
    }
    else {          // for "rebroacast" messages
        const char* key = msg->getName();
        // get a list of all messages similar to "msg"
        WaveShortMessages msgs = receivedMessages[atol(key)];

        // initialize dMin to high value.
        double dMin = 1000000.0;


        for (uint i = 0; i < msgs.size(); ++i) {
            WaveShortMessage* m = msgs[i];
            // get the position of this host
            Coord receiverPosition = traci->getPositionAt(simTime());
            // get the position of the sender
            Coord senderPosition = m->getSenderPos();
            // calculate the distance between the two
            double distance = senderPosition.distance(receiverPosition);
            // set dMin to the closest vehicle sending the same message
            if (distance < dMin) {
                dMin = distance;
            }
        }
        // if the dMin host is closer than distanceThreshold than do not disseminate.
        if (dMin < distanceThreshold)
            return;
        // if the dMin host is farther away than the distanceThreshold.. rebroadcast the message
        sendWSM(msgs[0]->dup());
    }
}
