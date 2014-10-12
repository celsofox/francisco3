package org.car2x.veins.modules.application.flooding;
import org.car2x.veins.modules.application.ieee80211p.BaseWaveApplLayer;

simple Flooding extends BaseWaveApplLayer
{
    @class(Flooding);
    @display("i=block/app2");

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
