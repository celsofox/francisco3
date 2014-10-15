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

//        std::cerr << "In Flooding::initialize()" << endl;

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
//    std::cerr << "In Flooding::onBeacon()" << endl;
}


void Flooding::onData(WaveShortMessage *wsm)
{
//    std::cerr << "In Flooding::onData()" << std::endl;

    emit(warningReceivedSignal, 1);
    emit(messageReceivedSignal, 1);
    stats->updateAllWarningsReceived();
    stats->updateAllMessagesReceived();

    // prevent originating disseminator from participating in further dissemination attempts
    if (sentMessage)
        return;

    bool messageIsRepeat = false;

    for (uint i = 0; i < warningMessages.size(); ++i) {
        WaveShortMessage* warningMessage = warningMessages[i];

//        std::cerr << "[DEBUG] wsm->getTreeId(): " << wsm->getTreeId() << " warningMessages[" << i << "]->getTreeId(): " << warningMessage->getTreeId();

        if (wsm->getTreeId() == warningMessage->getTreeId()) {
            messageIsRepeat = true;
        }
//        std::cerr << "   " << messageIsRepeat << std::endl;
    }

    if (traci->getRoadId()[0] != ':')
        traci->commandChangeRoute(wsm->getWsmData(), 9999);

    if (!messageIsRepeat /*&& !isRepeat*/) {
        sendWSM(wsm->dup());

        stats->updateNewWarningsReceived();
        emit(newWarningReceivedSignal, 1);

        warningMessages.push_back(wsm->dup());

//        isRepeat = true;
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
