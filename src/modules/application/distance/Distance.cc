#include "Distance.h"
#include <WaveShortMessage_m.h>
#include <iostream>

using Veins::TraCIMobility;
using Veins::TraCIMobilityAccess;

Define_Module(Distance)

void Distance::initialize(int stage)
{
    BaseWaveApplLayer::initialize(stage);
    if (stage == 0) {

//        std::cerr << "In Distance::initialize()" << endl;

        distanceThreshold = par("distanceThreshold").doubleValue();
        indexOfAccidentNode = par("indexOfAccidentNode").longValue();

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

    Coord coord = traci->getPositionAt(simTime());

    // dMin == the shortest d of a any neighbor who has broadcast the same message

    // add message to receivedMessages
    receivedMessages[wsm->getTreeId()].push_back(wsm->dup());

    // get dMin
    WaveShortMessages mv = receivedMessages[wsm->getTreeId()];
    double dMin = 100000.0;
    WaveShortMessage* dMinMessage;

    // is it a new warning message?
    if (mv.size() == 1) {
        stats->updateNewWarningsReceived();
        emit(newWarningReceivedSignal, 1);
    }

    for (uint i = 0; i < mv.size(); ++i) {
        WaveShortMessage* m = mv[i];
        double d = coord.distance(m->getSenderPos());
        if (d < dMin) {
            dMin = d;
            dMinMessage = m;
        }
    }

    // rebroadcast?
    bool rebroadcast = false;
    if (dMin > distanceThreshold)
        rebroadcast = true;

    if (rebroadcast) {
        sendWSM(dMinMessage->dup());

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
