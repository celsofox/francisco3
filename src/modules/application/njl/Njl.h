#ifndef Njl_H
#define Njl_H

#include "BaseWaveApplLayer.h"
#include "modules/mobility/traci/TraCIMobility.h"
#include "modules/stats/FranciscoStatistics.h"
#include <vector>
#include <map>
#include <string>
#include <list>

using Veins::TraCIMobility;
using Veins::AnnotationManager;
using std::vector;
using std::map;

#define SCHEDULED_REBROADCAST
typedef std::vector<WaveShortMessage*> WaveShortMessages;

class Njl : public BaseWaveApplLayer
{
public:
    enum NjlApplMessageKinds {
        DUMMY = SEND_BEACON_EVT,
        SCHEDULED_REBROADCAST_EVT
    };
protected:
    Veins::TraCIMobility* traci;
    FranciscoStatistics* stats;

//    std::list<std::string> junctionIds;
    std::map<std::string,Coord> junctionMap;

    std::map<long,WaveShortMessages> receivedMessageMap;
    std::vector<WaveShortMessage*> neighbors;

    double hostToJunctionDistanceThreshold;
    double sendWarningInterval;
    double neighborLifetimeThreshold;
    long indexOfAccidentNode;

    simsignal_t beaconReceivedSignal;
    simsignal_t newWarningReceivedSignal;
    simsignal_t warningReceivedSignal;
    simsignal_t messageReceivedSignal;

    simtime_t lastDroveAt;
    bool sentMessage;

protected:
    virtual void initialize(int stage);
    virtual void onBeacon(WaveShortMessage *wsm);
    virtual void onData(WaveShortMessage *wsm);
    virtual void handleSelfMsg(cMessage *msg);
    virtual void handlePositionUpdate(cObject *obj);

    void sendMessage(std::string blockedRoadId);
    bool hostIsClosestToJunction(std::string junctionId);
    std::string hostIsInJunction();
};

#endif // Njl_H
