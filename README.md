francisco3
=======


Usage
-------

* Clone repository to local file system.
* `$ cd <REPOSITORY_DIRECTORY>`
* Open "setenv" in text editor and configure envirnoment variables to your local file system.
* `$ source setenv`
* `$ ./configure`
* `$ make`
* `$ ./sumo-launchd.py -vv ${SUMO_HOME}/bin/sumo &`
* Choose a dissemination model in the "examples" directory.
* `$ cd examples/<MODEL_NAME>`
* `$ ./run`


Implementation Notes
---------------------

* Algorithm implementation code is found in src/modules/application directory and their applications are found in the examples directory.
* For each algorithm, the most pertinent code lies within the onBeacon and onData methods.
    * onBeacon is activated on a regularly timed signal (see `*.node[*].appl.beaconInterval` in the omnetpp.ini configuration file).
    * onData is activated upon other data reception.
    * Please also note that handleSelfMsg() is also pertinent to some of the algorithms when the scheduleAt() method is used in either onBeacon or onDemand.
    * Comments in these methods further explain the implementation.

