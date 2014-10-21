#include "Flooding.h"
#include <WaveShortMessage_m.h>
#include <iostream>

using Veins::TraCIMobility;
using Veins::TraCIMobilityAccess;

Define_Module(Flooding)

void Flooding::initialize(int stage)
{
    BaseWaveApplLayer::initialize(stage);
    if (stage == 0) {

        traci = TraCIMobilityAccess().get(getParentModule());
        stats = FranciscoStatisticsAccess().getIfExists();
        ASSERT(stats);
        beaconReceivedSignal = registerSignal("beaconReceivedSignal");
        warningReceivedSignal = registerSignal("warningReceivedSignal");
        messageReceivedSignal = registerSignal("messageReceivedSignal");
        newWarningReceivedSignal = registerSignal("newWarningReceivedSignal");

        indexOfAccidentNode = par("indexOfAccidentNode").longValue();

        lastDroveAt = simTime();
        sentMessage = false;
    }
}

void Flooding::receiveSignal(cComponent *source, simsignal_t signalID, cComponent::cObject *obj)
{
    Enter_Method_Silent();
    if (signalID == mobilityStateChangedSignal) {
        handlePositionUpdate(obj);
    }
//	else if (signalID == parkingStateChangedSignal) {
//		handleParkingUpdate(obj);
//	}
}


void Flooding::onBeacon(WaveShortMessage *wsm)
{
    // not used for this algorithm
}


void Flooding::onData(WaveShortMessage *wsm)
{
    // statistics recording
    emit(warningReceivedSignal, 1);
    emit(messageReceivedSignal, 1);
    stats->updateAllWarningsReceived();
    stats->updateAllMessagesReceived();

    // prevent originating disseminator from participating in further dissemination attempts
    if (sentMessage)
        return;

    bool messageIsRepeat = false;

    // is this a new warning message?
    for (uint i = 0; i < warningMessages.size(); ++i) {
        WaveShortMessage* warningMessage = warningMessages[i];
        if (wsm->getTreeId() == warningMessage->getTreeId()) {
            messageIsRepeat = true;
        }
    }

    if (traci->getRoadId()[0] != ':')
        traci->commandChangeRoute(wsm->getWsmData(), 9999);

    // rebroadcast only if new message
    if (!messageIsRepeat) {
        sendWSM(wsm->dup());

        stats->updateNewWarningsReceived();
        emit(newWarningReceivedSignal, 1);

        warningMessages.push_back(wsm->dup());
    }
}

void Flooding::handlePositionUpdate(cComponent::cObject *obj)
{
    // stopped for for at least 10s?
    if (traci->getSpeed() < 1) {
        if ((simTime() - lastDroveAt >= 10)
                && (!sentMessage)
                && (indexOfAccidentNode == getParentModule()->getIndex())) {

            std::cerr << "[DEBUG] ACCIDENT STARTED @simTime: " << simTime().str() << " for node: " << getParentModule()->getIndex() << endl;

            findHost()->getDisplayString().updateWith("r=16,red");
            if (!sentMessage) sendMessage(traci->getRoadId());
        }
    }
    else {
        lastDroveAt = simTime();
    }
}

void Flooding::sendMessage(std::string blockedRoadId)
{
    sentMessage = true;

    t_channel channel = dataOnSch ? type_SCH : type_CCH;
    WaveShortMessage* wsm = prepareWSM("data", dataLengthBits, channel, dataPriority, -1,2);
    wsm->setWsmData(blockedRoadId.c_str());
    sendWSM(wsm);
}
