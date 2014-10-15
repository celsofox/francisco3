#include "Nsf.h"
#include <WaveShortMessage_m.h>
#include <iostream>

using Veins::TraCIMobility;
using Veins::TraCIMobilityAccess;
using std::map;
using std::vector;
using std::string;


Define_Module(Nsf)

void Nsf::initialize(int stage)
{
    BaseWaveApplLayer::initialize(stage);
    if (stage == 0) {

//        std::cerr << "In Nsf::initialize()" << endl;

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
        sentMessage = false;
    }
}

void Nsf::receiveSignal(cComponent *source, simsignal_t signalID, cComponent::cObject *obj)
{
    Enter_Method_Silent();
    if (signalID == mobilityStateChangedSignal) {
        handlePositionUpdate(obj);
    }
//	else if (signalID == parkingStateChangedSignal) {
//		handleParkingUpdate(obj);
//	}
}


void Nsf::onBeacon(WaveShortMessage *wsm)
{
    // handle stats
    stats->updateAllBeaconsReceived();
    stats->updateAllMessagesReceived();
    emit(beaconReceivedSignal, 1);
    emit(messageReceivedSignal, 1);

    // is it a new neighbor?
    bool isNewNeighbor = true;
    vector<uint> indices;
    for (uint i = 0; i < neighbors.size(); ++i) {
        WaveShortMessage* neighbor = neighbors[i];

        if (neighbor->getTreeId() == wsm->getTreeId()) {
            isNewNeighbor = false;
            neighbors[i] = wsm;
        }
        else {
            // check for removal
            if (simTime() - neighbor->getArrivalTime() > neighborLifetimeThreshold)
                indices.push_back(i);
        }
    }

    // if it is a new neighbor
    if (isNewNeighbor) {
        for (map<long,WaveShortMessages>::iterator i = receivedWarningMessageMap.begin(); i != receivedWarningMessageMap.end(); ++i) {
            WaveShortMessage* msg = i->second[0];
            ASSERT(msg);
            sendWSM(msg->dup());
        }
        neighbors.push_back(wsm->dup());
    }

    // remove the old neighbors

//        neighbors.erase(neighbors.begin() + i);
    WaveShortMessages newNeighborList;
    for (uint i = 0; i < neighbors.size(); ++i) {
        bool keepNeighbor = true;
        for (uint j = 0; j < indices.size(); ++j) {
            if (i == indices[j])
                keepNeighbor = false;
        }
        if (keepNeighbor)
            newNeighborList.push_back(neighbors[i]);
    }
    neighbors = newNeighborList;
}


void Nsf::onData(WaveShortMessage *wsm)
{
    // handle stats
    emit(warningReceivedSignal, 1);
    emit(messageReceivedSignal, 1);
    stats->updateAllWarningsReceived();
    stats->updateAllMessagesReceived();

    // prevent originating disseminator from participating in further dissemination attempts
    if (sentMessage)
        return;

    // check if new warning
    bool isNewWarning = true;
    for (map<long,WaveShortMessages>::iterator i = receivedWarningMessageMap.begin(); i != receivedWarningMessageMap.end(); ++i) {
        WaveShortMessage* msg = i->second[0];
        if (msg->getTreeId() == wsm->getTreeId())
            isNewWarning = false;
    }

    if (isNewWarning) {
        emit(newWarningReceivedSignal, 1);
        stats->updateNewWarningsReceived();
    }


    if (neighbors.size() > 1) {
        sendWSM(wsm->dup());
    }

    receivedWarningMessageMap[wsm->getTreeId()].push_back(wsm->dup());
}

void Nsf::handlePositionUpdate(cComponent::cObject *obj)
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

void Nsf::sendMessage(std::string blockedRoadId)
{
    sentMessage = true;

    t_channel channel = dataOnSch ? type_SCH : type_CCH;
    WaveShortMessage* wsm = prepareWSM("data", dataLengthBits, channel, dataPriority, -1,2);
    wsm->setWsmData(blockedRoadId.c_str());
    sendWSM(wsm);
}

//void Nsf::handleSelfMsg(cMessage *msg)
//{

//}
