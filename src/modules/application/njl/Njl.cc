#include "Njl.h"

Define_Module(Njl)


using std::vector;
using std::string;
using std::map;
using std::list;
using Veins::TraCIMobility;
using Veins::TraCIMobilityAccess;


void Njl::initialize(int stage)
{
    BaseWaveApplLayer::initialize(stage);

    if (stage == 0) {
        hostToJunctionDistanceThreshold = par("hostToJunctionDistanceThreshold").doubleValue();
        sendWarningInterval = par("sendWarningInterval").doubleValue();
        neighborLifetimeThreshold = par("neighborLifetimeThreshold").doubleValue();
        indexOfAccidentNode = par("indexOfAccidentNode").longValue();

        traci = TraCIMobilityAccess().get(getParentModule());
        stats = FranciscoStatisticsAccess().getIfExists();
        ASSERT(stats);

        beaconReceivedSignal = registerSignal("beaconReceivedSignal");
        warningReceivedSignal = registerSignal("warningReceivedSignal");
        messageReceivedSignal = registerSignal("messageReceivedSignal");
        newWarningReceivedSignal = registerSignal("newWarningReceivedSignal");

        sentMessage = false;
        lastDroveAt = simTime();
    }
    else if (stage == 1) {
        list<string> junctionIds = traci->getCommandInterface()->getJunctionIds();

        for (list<string>::iterator i = junctionIds.begin(); i != junctionIds.end(); ++i) {
            string jId = *i;
            Coord jPos = traci->getManager()->traci2omnet(traci->getCommandInterface()->getJunctionPosition(jId));
            junctionMap[jId] = jPos;
        }
    }
}

void Njl::onBeacon(WaveShortMessage *wsm)
{
    // handle stats
    stats->updateAllBeaconsReceived();
    stats->updateAllMessagesReceived();
    emit(beaconReceivedSignal, 1);
    emit(messageReceivedSignal, 1);

    // is it a new neighbor?
    bool isNewNeighbor = true;
    for (uint i = 0; i < neighbors.size(); ++i) {
        if (neighbors[i]->getTreeId() == wsm->getTreeId())
            isNewNeighbor = false;
    }

    // if it is a new neighbor
    if (isNewNeighbor) {
        neighbors.push_back(wsm->dup());
    }
}

void Njl::onData(WaveShortMessage *wsm)
{
    // handle stats
    emit(warningReceivedSignal, 1);
    emit(messageReceivedSignal, 1);
    stats->updateAllWarningsReceived();
    stats->updateAllMessagesReceived();

    // prevent originating disseminator from participating in further dissemination attempts
    if (sentMessage)
        return;

    receivedMessageMap[wsm->getTreeId()].push_back(wsm->dup());

    // is it a new message?
    if (receivedMessageMap[wsm->getTreeId()].size() == 1) {
        std::cerr << "[DEBUG] node: " << getParentModule()->getIndex() << " wsm->getTreeId(): " << wsm->getTreeId() << std::endl;
        emit(newWarningReceivedSignal, 1);
        stats->updateNewWarningsReceived();
    }

    string jId = hostIsInJunction();
    if (jId.empty())
        return;
    if (!hostIsClosestToJunction(jId)) {
        char treeIdStr[64];
        sprintf(treeIdStr, "%ld", wsm->getTreeId());
        cMessage* msg = new cMessage(treeIdStr, SCHEDULED_REBROADCAST_EVT);
        scheduleAt(simTime() + sendWarningInterval, msg);
        return;
    }
}

void Njl::handleSelfMsg(cMessage *msg)
{
    if (msg->getKind() == SEND_BEACON_EVT) {
        BaseWaveApplLayer::handleSelfMsg(msg);
        return;
    }

    else if (msg->getKind() == SCHEDULED_REBROADCAST_EVT) {
        int treeId = atoi(msg->getName());
        WaveShortMessages ms = receivedMessageMap[treeId];
        if ((ms.empty()) || (ms.size() > 1))
            return;
        sendWSM(ms[0]->dup());
    }
}



void Njl::handlePositionUpdate(cObject *obj)
{
    BaseWaveApplLayer::handlePositionUpdate(obj);

    // stopped for for at least 10s?
    if (traci->getSpeed() < 1) {
        if ((simTime() - lastDroveAt >= 10)
                && (!sentMessage)
                && (indexOfAccidentNode == getParentModule()->getIndex())) {

            std::cerr << "[DEBUG] ACCIDENT STARTED @simTime: " << simTime().str() << " for node: " << getParentModule()->getIndex() << endl;

            findHost()->getDisplayString().updateWith("r=16,red");
            sendMessage(traci->getRoadId());
        }
    }
    else {
        lastDroveAt = simTime();
    }
}


void Njl::sendMessage(std::string blockedRoadId)
{
    t_channel channel = dataOnSch ? type_SCH : type_CCH;
    WaveShortMessage* wsm = prepareWSM("data", dataLengthBits, channel, dataPriority, -1,2);
    wsm->setWsmData(blockedRoadId.c_str());
    sendWSM(wsm);
    sentMessage = true;
}

bool Njl::hostIsClosestToJunction(string junctionId)
{
    // check to see if this host is near an intersection

    Coord jPos = junctionMap[junctionId];

    double hDist = jPos.distance(traci->getPositionAt(simTime()));

    for (uint i = 0; i < neighbors.size(); ++i) {
        WaveShortMessage* neighbor = neighbors[i];
        if (jPos.distance(neighbor->getSenderPos()) < hDist) {
            return false;
        }
    }
    return true;
}



string Njl::hostIsInJunction()
{
    // check to see if this host is near an intersection

    for (map<string,Coord>::iterator i = junctionMap.begin(); i != junctionMap.end(); ++i) {
        string jId = i->first;
        Coord jPos = i->second;
        Coord hPos = traci->getPositionAt(simTime());
        if (jPos.distance(hPos) < hostToJunctionDistanceThreshold) {
            return jId;
        }
    }

    return string();
}
