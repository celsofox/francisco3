package org.car2x.veins.modules.application.counter;
import org.car2x.veins.modules.application.ieee80211p.BaseWaveApplLayer;

simple Counter extends BaseWaveApplLayer
{
    @class(Counter);
    @display("i=block/app2");

    int counterThreshold = default(3);
    volatile double randomRebroacastDelay @unit("s") = default(uniform(0,500));
    volatile double randomRebroadcastDelay = default(0.01);
    int indexOfAccidentNode = default(0);

    @signal[warningReceivedSignal](type=long);
    @statistic[warningReceivedSignal](record=count,vector; description="Warning Message Received");

    @signal[beaconReceivedSignal](type=long);
    @statistic[beaconReceivedSignal](record=count,vector; description="Beacon Message Received");

    @signal[messageReceivedSignal](type=long);
    @statistic[messageReceivedSignal](record=count,vector; description="Message Received");

    @signal[newWarningReceivedSignal](type=long);
    @statistic[newWarningReceivedSignal](record=count,vector; description="New Warning Message Received");
}
