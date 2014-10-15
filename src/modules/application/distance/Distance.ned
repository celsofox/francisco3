package org.car2x.veins.modules.application.distance;
import org.car2x.veins.modules.application.ieee80211p.BaseWaveApplLayer;

simple Distance extends BaseWaveApplLayer
{
    @class(Distance);
    @display("i=block/app2");

    double distanceThreshold = default(300.0);
    int indexOfAccidentNode = default(0);
    volatile double randomRebroadcastDelay = default(uniform(0,500));

    @signal[warningReceivedSignal](type=long);
    @statistic[warningReceivedSignal](record=count,vector; description="Warning Message Received");

    @signal[beaconReceivedSignal](type=long);
    @statistic[beaconReceivedSignal](record=count,vector; description="Beacon Message Received");

    @signal[messageReceivedSignal](type=long);
    @statistic[messageReceivedSignal](record=count,vector; description="Message Received");

    @signal[newWarningReceivedSignal](type=long);
    @statistic[newWarningReceivedSignal](record=count,vector; description="New Warning Message Received");
}
