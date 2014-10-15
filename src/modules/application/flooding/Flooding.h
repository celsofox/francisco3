#ifndef FLOODING_H
#define FLOODING_H

#include "BaseWaveApplLayer.h"
#include "modules/mobility/traci/TraCIMobility.h"
#include "modules/stats/FranciscoStatistics.h"
#include <vector>

using Veins::TraCIMobility;
using Veins::AnnotationManager;
using std::vector;

class Flooding : public BaseWaveApplLayer
{

public:
    virtual void initialize(int stage);
    virtual void receiveSignal(cComponent *source, simsignal_t signalID, cObject *obj);

protected:
    TraCIMobility* traci;
    FranciscoStatistics* stats;
    vector<WaveShortMessage*> warningMessages;
    simsignal_t beaconReceivedSignal;
    simsignal_t warningReceivedSignal;
    simsignal_t newWarningReceivedSignal;
    simsignal_t messageReceivedSignal;
    simtime_t lastDroveAt;
    bool sentMessage;
    long indexOfAccidentNode;    

protected:
    virtual void onBeacon(WaveShortMessage *wsm);
    virtual void onData(WaveShortMessage *wsm);
    virtual void handlePositionUpdate(cObject *obj);
    virtual void sendMessage(std::string blockedRoadId);
};

#endif // FLOODING_H
