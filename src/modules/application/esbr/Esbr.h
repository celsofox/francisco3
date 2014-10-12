#ifndef Esbr_H
#define Esbr_H

#include "BaseWaveApplLayer.h"
#include "modules/mobility/traci/TraCIMobility.h"
#include "modules/stats/FranciscoStatistics.h"
#include <vector>
#include <map>

using Veins::TraCIMobility;
using Veins::AnnotationManager;
using std::vector;
using std::map;

typedef std::vector<WaveShortMessage*> WaveShortMessages;

class Esbr : public BaseWaveApplLayer
{

public:
    virtual void initialize(int stage);
    virtual void receiveSignal(cComponent *source, simsignal_t signalID, cObject *obj);

protected:
    TraCIMobility* traci;
    FranciscoStatistics* stats;
//    vector<WaveShortMessage*> warningMessages;

    double senderReceiverDistanceThreshold;
    double sendWarningInterval;
    double hostToJunctionDistanceThreshold;
    double neighborLifetimeThreshold;
    long indexOfAccidentNode;


    simsignal_t beaconReceivedSignal;
    simsignal_t warningReceivedSignal;
    simsignal_t newWarningReceivedSignal;
    simsignal_t messageReceivedSignal;

    simtime_t lastDroveAt;
    bool sentMessage;
    bool hostIsInWarningMode;

    map<long,WaveShortMessages>     receivedMessageMap;    // treeId, WSM vector
    std::vector<WaveShortMessage*>  neighbors;
    std::list<std::string>          junctionIds;

protected:
    virtual void onBeacon(WaveShortMessage *wsm);
    virtual void onData(WaveShortMessage *wsm);
    virtual void handlePositionUpdate(cObject *obj);
    virtual void sendMessage(std::string blockedRoadId);
    virtual void handleSelfMsg(cMessage *msg);

    void startWarningModeIntervalBroadcast();
    bool hostIsInJunction();
};

#endif // Esbr_H
