#
# OMNeT++/OMNEST Makefile for libveins
#
# This file was generated with the command:
#  opp_makemake -f --deep --make-so -o veins -O out -Xsrc/inet
#

# Name of target to be created (-o option)
TARGET = libveins$(SHARED_LIB_SUFFIX)

# C++ include paths (with -I)
INCLUDE_PATH = \
    -I. \
    -Ibase \
    -Ibase/connectionManager \
    -Ibase/messages \
    -Ibase/modules \
    -Ibase/phyLayer \
    -Ibase/utils \
    -Iinet \
    -Imodules \
    -Imodules/analogueModel \
    -Imodules/application \
    -Imodules/application/counter \
    -Imodules/application/distance \
    -Imodules/application/esbr \
    -Imodules/application/flooding \
    -Imodules/application/ieee80211p \
    -Imodules/application/njl \
    -Imodules/application/nsf \
    -Imodules/application/traci \
    -Imodules/mac \
    -Imodules/mac/ieee80211p \
    -Imodules/messages \
    -Imodules/mobility \
    -Imodules/mobility/traci \
    -Imodules/nic \
    -Imodules/obstacle \
    -Imodules/phy \
    -Imodules/stats \
    -Imodules/utility \
    -Imodules/world \
    -Imodules/world/annotations \
    -Inodes \
    -Iscripts

# Additional object and library files to link with
EXTRA_OBJS =

# Additional libraries (-L, -l options)
LIBS =

# Output directory
PROJECT_OUTPUT_DIR = out
PROJECTRELATIVE_PATH =
O = $(PROJECT_OUTPUT_DIR)/$(CONFIGNAME)/$(PROJECTRELATIVE_PATH)

# Object files for local .cc and .msg files
OBJS = \
    $O/base/connectionManager/BaseConnectionManager.o \
    $O/base/connectionManager/ChannelAccess.o \
    $O/base/connectionManager/ConnectionManager.o \
    $O/base/connectionManager/NicEntryDebug.o \
    $O/base/connectionManager/NicEntryDirect.o \
    $O/base/modules/BaseApplLayer.o \
    $O/base/modules/BaseBattery.o \
    $O/base/modules/BaseLayer.o \
    $O/base/modules/BaseMacLayer.o \
    $O/base/modules/BaseMobility.o \
    $O/base/modules/BaseModule.o \
    $O/base/modules/BaseWorldUtility.o \
    $O/base/modules/BatteryAccess.o \
    $O/base/phyLayer/BaseDecider.o \
    $O/base/phyLayer/BasePhyLayer.o \
    $O/base/phyLayer/ChannelInfo.o \
    $O/base/phyLayer/ChannelState.o \
    $O/base/phyLayer/Decider.o \
    $O/base/phyLayer/MappingBase.o \
    $O/base/phyLayer/MappingUtils.o \
    $O/base/phyLayer/PhyUtils.o \
    $O/base/phyLayer/Signal.o \
    $O/base/utils/asserts.o \
    $O/base/utils/NetwToMacControlInfo.o \
    $O/base/utils/SimpleAddress.o \
    $O/base/utils/winsupport.o \
    $O/inet/Coord.o \
    $O/modules/analogueModel/BreakpointPathlossModel.o \
    $O/modules/analogueModel/JakesFading.o \
    $O/modules/analogueModel/LogNormalShadowing.o \
    $O/modules/analogueModel/PERModel.o \
    $O/modules/analogueModel/SimpleObstacleShadowing.o \
    $O/modules/analogueModel/SimplePathlossModel.o \
    $O/modules/analogueModel/TwoRayInterferenceModel.o \
    $O/modules/application/counter/Counter.o \
    $O/modules/application/distance/Distance.o \
    $O/modules/application/esbr/Esbr.o \
    $O/modules/application/flooding/Flooding.o \
    $O/modules/application/ieee80211p/BaseWaveApplLayer.o \
    $O/modules/application/njl/Njl.o \
    $O/modules/application/nsf/Nsf.o \
    $O/modules/application/traci/TraCIDemo11p.o \
    $O/modules/application/traci/TraCIDemoRSU11p.o \
    $O/modules/application/traci/TraCITestApp.o \
    $O/modules/mac/ieee80211p/Mac1609_4.o \
    $O/modules/mobility/traci/TraCIBuffer.o \
    $O/modules/mobility/traci/TraCIColor.o \
    $O/modules/mobility/traci/TraCICommandInterface.o \
    $O/modules/mobility/traci/TraCIConnection.o \
    $O/modules/mobility/traci/TraCIMobility.o \
    $O/modules/mobility/traci/TraCIScenarioManager.o \
    $O/modules/mobility/traci/TraCIScenarioManagerInet.o \
    $O/modules/mobility/traci/TraCIScenarioManagerLaunchd.o \
    $O/modules/obstacle/Obstacle.o \
    $O/modules/obstacle/ObstacleControl.o \
    $O/modules/phy/Decider80211p.o \
    $O/modules/phy/PhyLayer80211p.o \
    $O/modules/phy/SNRThresholdDecider.o \
    $O/modules/stats/FranciscoStatistics.o \
    $O/modules/world/annotations/AnnotationDummy.o \
    $O/modules/world/annotations/AnnotationManager.o \
    $O/base/messages/AirFrame_m.o \
    $O/base/messages/BorderMsg_m.o \
    $O/base/messages/ChannelSenseRequest_m.o \
    $O/base/messages/MacPkt_m.o \
    $O/modules/messages/AirFrame11p_m.o \
    $O/modules/messages/Mac80211Pkt_m.o \
    $O/modules/messages/WaveShortMessage_m.o

# Message files
MSGFILES = \
    base/messages/AirFrame.msg \
    base/messages/BorderMsg.msg \
    base/messages/ChannelSenseRequest.msg \
    base/messages/MacPkt.msg \
    modules/messages/AirFrame11p.msg \
    modules/messages/Mac80211Pkt.msg \
    modules/messages/WaveShortMessage.msg

#------------------------------------------------------------------------------

# Pull in OMNeT++ configuration (Makefile.inc or configuser.vc)

ifneq ("$(OMNETPP_CONFIGFILE)","")
CONFIGFILE = $(OMNETPP_CONFIGFILE)
else
ifneq ("$(OMNETPP_ROOT)","")
CONFIGFILE = $(OMNETPP_ROOT)/Makefile.inc
else
CONFIGFILE = $(shell opp_configfilepath)
endif
endif

ifeq ("$(wildcard $(CONFIGFILE))","")
$(error Config file '$(CONFIGFILE)' does not exist -- add the OMNeT++ bin directory to the path so that opp_configfilepath can be found, or set the OMNETPP_CONFIGFILE variable to point to Makefile.inc)
endif

include $(CONFIGFILE)

# Simulation kernel and user interface libraries
OMNETPP_LIB_SUBDIR = $(OMNETPP_LIB_DIR)/$(TOOLCHAIN_NAME)
OMNETPP_LIBS = -L"$(OMNETPP_LIB_SUBDIR)" -L"$(OMNETPP_LIB_DIR)" -loppenvir$D $(KERNEL_LIBS) $(SYS_LIBS)

COPTS = $(CFLAGS)  $(INCLUDE_PATH) -I$(OMNETPP_INCL_DIR)
MSGCOPTS = $(INCLUDE_PATH)

# we want to recompile everything if COPTS changes,
# so we store COPTS into $COPTS_FILE and have object
# files depend on it (except when "make depend" was called)
COPTS_FILE = $O/.last-copts
ifneq ($(MAKECMDGOALS),depend)
ifneq ("$(COPTS)","$(shell cat $(COPTS_FILE) 2>/dev/null || echo '')")
$(shell $(MKPATH) "$O" && echo "$(COPTS)" >$(COPTS_FILE))
endif
endif

#------------------------------------------------------------------------------
# User-supplied makefile fragment(s)
# >>>
# <<<
#------------------------------------------------------------------------------

# Main target
all: $O/$(TARGET)
	$(Q)$(LN) $O/$(TARGET) .

$O/$(TARGET): $(OBJS)  $(wildcard $(EXTRA_OBJS)) Makefile
	@$(MKPATH) $O
	@echo Creating shared library: $@
	$(Q)$(SHLIB_LD) -o $O/$(TARGET)  $(OBJS) $(EXTRA_OBJS) $(AS_NEEDED_OFF) $(WHOLE_ARCHIVE_ON) $(LIBS) $(WHOLE_ARCHIVE_OFF) $(OMNETPP_LIBS) $(LDFLAGS)
	$(Q)$(SHLIB_POSTPROCESS) $O/$(TARGET)

.PHONY: all clean cleanall depend msgheaders

.SUFFIXES: .cc

$O/%.o: %.cc $(COPTS_FILE)
	@$(MKPATH) $(dir $@)
	$(qecho) "$<"
	$(Q)$(CXX) -c $(CXXFLAGS) $(COPTS) -o $@ $<

%_m.cc %_m.h: %.msg
	$(qecho) MSGC: $<
	$(Q)$(MSGC) -s _m.cc $(MSGCOPTS) $?

msgheaders: $(MSGFILES:.msg=_m.h)

clean:
	$(qecho) Cleaning...
	$(Q)-rm -rf $O
	$(Q)-rm -f veins veins.exe libveins.so libveins.a libveins.dll libveins.dylib
	$(Q)-rm -f ./*_m.cc ./*_m.h
	$(Q)-rm -f base/*_m.cc base/*_m.h
	$(Q)-rm -f base/connectionManager/*_m.cc base/connectionManager/*_m.h
	$(Q)-rm -f base/messages/*_m.cc base/messages/*_m.h
	$(Q)-rm -f base/modules/*_m.cc base/modules/*_m.h
	$(Q)-rm -f base/phyLayer/*_m.cc base/phyLayer/*_m.h
	$(Q)-rm -f base/utils/*_m.cc base/utils/*_m.h
	$(Q)-rm -f inet/*_m.cc inet/*_m.h
	$(Q)-rm -f modules/*_m.cc modules/*_m.h
	$(Q)-rm -f modules/analogueModel/*_m.cc modules/analogueModel/*_m.h
	$(Q)-rm -f modules/application/*_m.cc modules/application/*_m.h
	$(Q)-rm -f modules/application/counter/*_m.cc modules/application/counter/*_m.h
	$(Q)-rm -f modules/application/distance/*_m.cc modules/application/distance/*_m.h
	$(Q)-rm -f modules/application/esbr/*_m.cc modules/application/esbr/*_m.h
	$(Q)-rm -f modules/application/flooding/*_m.cc modules/application/flooding/*_m.h
	$(Q)-rm -f modules/application/ieee80211p/*_m.cc modules/application/ieee80211p/*_m.h
	$(Q)-rm -f modules/application/njl/*_m.cc modules/application/njl/*_m.h
	$(Q)-rm -f modules/application/nsf/*_m.cc modules/application/nsf/*_m.h
	$(Q)-rm -f modules/application/traci/*_m.cc modules/application/traci/*_m.h
	$(Q)-rm -f modules/mac/*_m.cc modules/mac/*_m.h
	$(Q)-rm -f modules/mac/ieee80211p/*_m.cc modules/mac/ieee80211p/*_m.h
	$(Q)-rm -f modules/messages/*_m.cc modules/messages/*_m.h
	$(Q)-rm -f modules/mobility/*_m.cc modules/mobility/*_m.h
	$(Q)-rm -f modules/mobility/traci/*_m.cc modules/mobility/traci/*_m.h
	$(Q)-rm -f modules/nic/*_m.cc modules/nic/*_m.h
	$(Q)-rm -f modules/obstacle/*_m.cc modules/obstacle/*_m.h
	$(Q)-rm -f modules/phy/*_m.cc modules/phy/*_m.h
	$(Q)-rm -f modules/stats/*_m.cc modules/stats/*_m.h
	$(Q)-rm -f modules/utility/*_m.cc modules/utility/*_m.h
	$(Q)-rm -f modules/world/*_m.cc modules/world/*_m.h
	$(Q)-rm -f modules/world/annotations/*_m.cc modules/world/annotations/*_m.h
	$(Q)-rm -f nodes/*_m.cc nodes/*_m.h
	$(Q)-rm -f scripts/*_m.cc scripts/*_m.h

cleanall: clean
	$(Q)-rm -rf $(PROJECT_OUTPUT_DIR)

depend:
	$(qecho) Creating dependencies...
	$(Q)$(MAKEDEPEND) $(INCLUDE_PATH) -f Makefile -P\$$O/ -- $(MSG_CC_FILES)  ./*.cc base/*.cc base/connectionManager/*.cc base/messages/*.cc base/modules/*.cc base/phyLayer/*.cc base/utils/*.cc inet/*.cc modules/*.cc modules/analogueModel/*.cc modules/application/*.cc modules/application/counter/*.cc modules/application/distance/*.cc modules/application/esbr/*.cc modules/application/flooding/*.cc modules/application/ieee80211p/*.cc modules/application/njl/*.cc modules/application/nsf/*.cc modules/application/traci/*.cc modules/mac/*.cc modules/mac/ieee80211p/*.cc modules/messages/*.cc modules/mobility/*.cc modules/mobility/traci/*.cc modules/nic/*.cc modules/obstacle/*.cc modules/phy/*.cc modules/stats/*.cc modules/utility/*.cc modules/world/*.cc modules/world/annotations/*.cc nodes/*.cc scripts/*.cc

# DO NOT DELETE THIS LINE -- make depend depends on it.
$O/base/connectionManager/BaseConnectionManager.o: base/connectionManager/BaseConnectionManager.cc \
  inet/FWMath.h \
  base/modules/BaseWorldUtility.h \
  base/utils/miximkerneldefs.h \
  base/connectionManager/BaseConnectionManager.h \
  base/utils/FindModule.h \
  inet/Coord.h \
  base/utils/MiXiMDefs.h \
  base/connectionManager/NicEntry.h \
  base/connectionManager/NicEntryDirect.h \
  base/connectionManager/NicEntryDebug.h
$O/base/connectionManager/ChannelAccess.o: base/connectionManager/ChannelAccess.cc \
  base/utils/HostState.h \
  base/modules/BaseModule.h \
  base/utils/MiXiMDefs.h \
  inet/Coord.h \
  base/connectionManager/NicEntry.h \
  base/utils/Move.h \
  base/modules/BaseBattery.h \
  base/modules/BaseWorldUtility.h \
  inet/FWMath.h \
  base/modules/BatteryAccess.h \
  base/utils/FindModule.h \
  ./base/connectionManager/ChannelAccess.h \
  base/utils/miximkerneldefs.h \
  base/connectionManager/BaseConnectionManager.h \
  base/modules/BaseMobility.h
$O/base/connectionManager/ConnectionManager.o: base/connectionManager/ConnectionManager.cc \
  base/connectionManager/BaseConnectionManager.h \
  base/utils/miximkerneldefs.h \
  inet/FWMath.h \
  base/connectionManager/ConnectionManager.h \
  base/modules/BaseWorldUtility.h \
  base/connectionManager/NicEntry.h \
  inet/Coord.h \
  base/utils/MiXiMDefs.h
$O/base/connectionManager/NicEntryDebug.o: base/connectionManager/NicEntryDebug.cc \
  base/utils/Move.h \
  base/connectionManager/NicEntryDebug.h \
  base/connectionManager/NicEntry.h \
  base/modules/BaseModule.h \
  base/utils/MiXiMDefs.h \
  inet/Coord.h \
  base/utils/HostState.h \
  base/modules/BaseMobility.h \
  base/modules/BatteryAccess.h \
  base/utils/FindModule.h \
  base/utils/miximkerneldefs.h \
  ./base/connectionManager/ChannelAccess.h \
  base/modules/BaseBattery.h \
  base/modules/BaseWorldUtility.h \
  inet/FWMath.h
$O/base/connectionManager/NicEntryDirect.o: base/connectionManager/NicEntryDirect.cc \
  base/modules/BatteryAccess.h \
  base/utils/FindModule.h \
  base/utils/miximkerneldefs.h \
  ./base/connectionManager/ChannelAccess.h \
  base/modules/BaseMobility.h \
  base/modules/BaseBattery.h \
  base/modules/BaseWorldUtility.h \
  inet/FWMath.h \
  base/connectionManager/NicEntry.h \
  base/connectionManager/NicEntryDirect.h \
  base/utils/Move.h \
  base/utils/HostState.h \
  base/modules/BaseModule.h \
  inet/Coord.h \
  base/utils/MiXiMDefs.h
$O/base/modules/BaseApplLayer.o: base/modules/BaseApplLayer.cc \
  base/utils/MiXiMDefs.h \
  base/modules/BaseModule.h \
  base/utils/HostState.h \
  base/utils/PassedMessage.h \
  base/utils/SimpleAddress.h \
  base/modules/BaseLayer.h \
  base/modules/BaseBattery.h \
  base/modules/BaseApplLayer.h \
  base/utils/miximkerneldefs.h \
  base/modules/BatteryAccess.h
$O/base/modules/BaseBattery.o: base/modules/BaseBattery.cc \
  base/utils/HostState.h \
  base/utils/MiXiMDefs.h \
  base/modules/BaseModule.h \
  base/modules/BaseBattery.h \
  base/utils/miximkerneldefs.h
$O/base/modules/BaseLayer.o: base/modules/BaseLayer.cc \
  base/modules/BatteryAccess.h \
  base/utils/miximkerneldefs.h \
  base/modules/BaseBattery.h \
  base/modules/BaseLayer.h \
  base/utils/PassedMessage.h \
  base/modules/BaseModule.h \
  base/utils/MiXiMDefs.h \
  base/utils/HostState.h
$O/base/modules/BaseMacLayer.o: base/modules/BaseMacLayer.cc \
  base/modules/BaseWorldUtility.h \
  base/modules/BaseBattery.h \
  base/modules/AddressingInterface.h \
  base/phyLayer/PhyUtils.h \
  inet/FWMath.h \
  base/modules/BaseLayer.h \
  base/utils/NetwToMacControlInfo.h \
  base/phyLayer/Mapping.h \
  base/utils/Move.h \
  base/messages/MacPkt_m.h \
  base/phyLayer/MappingBase.h \
  base/phyLayer/MacToPhyInterface.h \
  base/phyLayer/Interpolation.h \
  inet/Coord.h \
  base/modules/BaseMacLayer.h \
  base/utils/HostState.h \
  base/phyLayer/MacToPhyControlInfo.h \
  base/modules/BaseMobility.h \
  base/modules/BatteryAccess.h \
  base/utils/FindModule.h \
  ./base/connectionManager/ChannelAccess.h \
  base/utils/miximkerneldefs.h \
  base/phyLayer/Signal_.h \
  base/phyLayer/AnalogueModel.h \
  base/utils/MacToNetwControlInfo.h \
  base/utils/SimpleAddress.h \
  base/utils/PassedMessage.h \
  base/phyLayer/MappingUtils.h \
  base/phyLayer/ChannelState.h \
  base/modules/BaseModule.h \
  base/utils/MiXiMDefs.h
$O/base/modules/BaseMobility.o: base/modules/BaseMobility.cc \
  base/modules/BaseModule.h \
  base/utils/MiXiMDefs.h \
  base/utils/FindModule.h \
  base/modules/BatteryAccess.h \
  base/utils/miximkerneldefs.h \
  base/modules/BaseMobility.h \
  base/utils/HostState.h \
  base/messages/BorderMsg_m.h \
  inet/Coord.h \
  base/utils/Move.h \
  base/modules/BaseWorldUtility.h \
  base/modules/BaseBattery.h \
  inet/FWMath.h
$O/base/modules/BaseModule.o: base/modules/BaseModule.cc \
  base/utils/MiXiMDefs.h \
  base/modules/BaseModule.h \
  base/utils/miximkerneldefs.h \
  base/utils/FindModule.h \
  base/utils/HostState.h
$O/base/modules/BaseWorldUtility.o: base/modules/BaseWorldUtility.cc \
  inet/Coord.h \
  base/connectionManager/NicEntry.h \
  base/modules/BaseWorldUtility.h \
  inet/FWMath.h \
  base/utils/MiXiMDefs.h \
  base/utils/FindModule.h \
  base/connectionManager/BaseConnectionManager.h \
  base/utils/miximkerneldefs.h
$O/base/modules/BatteryAccess.o: base/modules/BatteryAccess.cc \
  base/modules/BatteryAccess.h \
  base/utils/FindModule.h \
  base/utils/miximkerneldefs.h \
  base/modules/BaseModule.h \
  base/utils/MiXiMDefs.h \
  base/modules/BaseBattery.h \
  base/utils/HostState.h
$O/base/phyLayer/BaseDecider.o: base/phyLayer/BaseDecider.cc \
  base/phyLayer/Decider.h \
  base/phyLayer/DeciderToPhyInterface.h \
  base/phyLayer/ChannelState.h \
  base/phyLayer/MappingUtils.h \
  base/utils/MiXiMDefs.h \
  base/utils/miximkerneldefs.h \
  base/phyLayer/Signal_.h \
  base/phyLayer/BaseDecider.h \
  base/phyLayer/Mapping.h \
  base/phyLayer/Interpolation.h \
  base/phyLayer/MappingBase.h \
  ./base/messages/AirFrame_m.h \
  base/messages/ChannelSenseRequest_m.h
$O/base/phyLayer/BasePhyLayer.o: base/phyLayer/BasePhyLayer.cc \
  base/phyLayer/BasePhyLayer.h \
  base/modules/BaseBattery.h \
  base/modules/BaseWorldUtility.h \
  base/phyLayer/MappingBase.h \
  base/phyLayer/Interpolation.h \
  base/connectionManager/NicEntry.h \
  base/utils/HostState.h \
  base/phyLayer/ChannelInfo.h \
  base/utils/miximkerneldefs.h \
  ./base/connectionManager/ChannelAccess.h \
  base/phyLayer/AnalogueModel.h \
  base/phyLayer/MappingUtils.h \
  base/phyLayer/Decider.h \
  base/phyLayer/PhyToMacControlInfo.h \
  base/modules/BaseModule.h \
  base/utils/MiXiMDefs.h \
  ./base/messages/AirFrame_m.h \
  base/messages/ChannelSenseRequest_m.h \
  base/phyLayer/PhyUtils.h \
  inet/FWMath.h \
  base/phyLayer/MacToPhyInterface.h \
  base/phyLayer/Mapping.h \
  base/utils/Move.h \
  inet/Coord.h \
  base/utils/FindModule.h \
  base/modules/BatteryAccess.h \
  base/connectionManager/BaseConnectionManager.h \
  base/phyLayer/Signal_.h \
  base/phyLayer/MacToPhyControlInfo.h \
  base/modules/BaseMobility.h \
  base/phyLayer/DeciderToPhyInterface.h \
  base/phyLayer/ChannelState.h
$O/base/phyLayer/ChannelInfo.o: base/phyLayer/ChannelInfo.cc \
  base/utils/miximkerneldefs.h \
  base/phyLayer/ChannelInfo.h \
  base/phyLayer/MappingUtils.h \
  base/utils/MiXiMDefs.h \
  base/phyLayer/Interpolation.h \
  base/phyLayer/MappingBase.h \
  base/phyLayer/Signal_.h \
  ./base/messages/AirFrame_m.h \
  base/phyLayer/Mapping.h
$O/base/phyLayer/ChannelState.o: base/phyLayer/ChannelState.cc \
  base/utils/miximkerneldefs.h \
  base/utils/MiXiMDefs.h \
  base/phyLayer/ChannelState.h
$O/base/phyLayer/Decider.o: base/phyLayer/Decider.cc \
  base/phyLayer/ChannelState.h \
  base/phyLayer/DeciderToPhyInterface.h \
  base/messages/ChannelSenseRequest_m.h \
  base/utils/miximkerneldefs.h \
  base/phyLayer/Decider.h \
  base/utils/MiXiMDefs.h
$O/base/phyLayer/MappingBase.o: base/phyLayer/MappingBase.cc \
  base/utils/MiXiMDefs.h \
  base/utils/miximkerneldefs.h \
  base/phyLayer/Interpolation.h \
  base/phyLayer/MappingBase.h
$O/base/phyLayer/MappingUtils.o: base/phyLayer/MappingUtils.cc \
  base/phyLayer/MappingBase.h \
  base/phyLayer/Interpolation.h \
  base/utils/miximkerneldefs.h \
  base/utils/MiXiMDefs.h \
  base/phyLayer/MappingUtils.h
$O/base/phyLayer/PhyUtils.o: base/phyLayer/PhyUtils.cc \
  base/phyLayer/MappingBase.h \
  base/phyLayer/Interpolation.h \
  base/utils/miximkerneldefs.h \
  base/phyLayer/AnalogueModel.h \
  base/phyLayer/MappingUtils.h \
  base/utils/MiXiMDefs.h \
  ./base/messages/AirFrame_m.h \
  inet/FWMath.h \
  base/phyLayer/PhyUtils.h \
  base/phyLayer/Mapping.h \
  inet/Coord.h \
  base/phyLayer/Signal_.h
$O/base/phyLayer/Signal.o: base/phyLayer/Signal.cc \
  base/utils/miximkerneldefs.h \
  base/phyLayer/MappingUtils.h \
  base/utils/MiXiMDefs.h \
  base/phyLayer/MappingBase.h \
  base/phyLayer/Interpolation.h \
  base/phyLayer/Signal_.h \
  base/phyLayer/Mapping.h
$O/base/utils/NetwToMacControlInfo.o: base/utils/NetwToMacControlInfo.cc \
  base/utils/MiXiMDefs.h \
  base/utils/miximkerneldefs.h \
  base/utils/NetwToMacControlInfo.h \
  base/utils/SimpleAddress.h
$O/base/utils/SimpleAddress.o: base/utils/SimpleAddress.cc \
  base/utils/miximkerneldefs.h \
  base/utils/MiXiMDefs.h \
  base/utils/SimpleAddress.h
$O/base/utils/asserts.o: base/utils/asserts.cc \
  base/utils/asserts.h
$O/base/utils/winsupport.o: base/utils/winsupport.cc \
  base/utils/winsupport.h
$O/inet/Coord.o: inet/Coord.cc \
  inet/Coord.h \
  inet/FWMath.h \
  base/utils/MiXiMDefs.h \
  base/utils/miximkerneldefs.h
$O/modules/analogueModel/BreakpointPathlossModel.o: modules/analogueModel/BreakpointPathlossModel.cc \
  base/phyLayer/Signal_.h \
  base/phyLayer/Mapping.h \
  inet/Coord.h \
  modules/analogueModel/BreakpointPathlossModel.h \
  ./base/messages/AirFrame_m.h \
  inet/FWMath.h \
  base/phyLayer/MappingUtils.h \
  base/utils/MiXiMDefs.h \
  base/utils/miximkerneldefs.h \
  base/phyLayer/AnalogueModel.h \
  base/phyLayer/Interpolation.h \
  base/phyLayer/MappingBase.h
$O/modules/analogueModel/JakesFading.o: modules/analogueModel/JakesFading.cc \
  base/phyLayer/AnalogueModel.h \
  ./base/connectionManager/ChannelAccess.h \
  base/utils/miximkerneldefs.h \
  base/modules/BaseModule.h \
  base/utils/MiXiMDefs.h \
  base/phyLayer/MappingUtils.h \
  base/modules/BaseWorldUtility.h \
  base/modules/BaseBattery.h \
  base/utils/HostState.h \
  base/phyLayer/MappingBase.h \
  base/phyLayer/Interpolation.h \
  base/utils/FindModule.h \
  base/modules/BatteryAccess.h \
  base/phyLayer/Signal_.h \
  base/modules/BaseMobility.h \
  inet/FWMath.h \
  ./base/messages/AirFrame_m.h \
  modules/analogueModel/JakesFading.h \
  inet/Coord.h \
  base/utils/Move.h \
  base/phyLayer/Mapping.h
$O/modules/analogueModel/LogNormalShadowing.o: modules/analogueModel/LogNormalShadowing.cc \
  base/phyLayer/Interpolation.h \
  base/phyLayer/MappingBase.h \
  base/phyLayer/MappingUtils.h \
  base/utils/MiXiMDefs.h \
  base/utils/miximkerneldefs.h \
  base/phyLayer/AnalogueModel.h \
  base/phyLayer/Mapping.h \
  inet/Coord.h \
  modules/analogueModel/LogNormalShadowing.h \
  ./base/messages/AirFrame_m.h \
  inet/FWMath.h \
  base/phyLayer/Signal_.h
$O/modules/analogueModel/PERModel.o: modules/analogueModel/PERModel.cc \
  base/phyLayer/Interpolation.h \
  base/phyLayer/MappingBase.h \
  modules/analogueModel/PERModel.h \
  base/phyLayer/MappingUtils.h \
  base/utils/MiXiMDefs.h \
  base/utils/miximkerneldefs.h \
  base/phyLayer/AnalogueModel.h \
  base/phyLayer/Mapping.h \
  inet/Coord.h \
  ./base/messages/AirFrame_m.h \
  inet/FWMath.h \
  base/phyLayer/Signal_.h
$O/modules/analogueModel/SimpleObstacleShadowing.o: modules/analogueModel/SimpleObstacleShadowing.cc \
  base/utils/MiXiMDefs.h \
  modules/analogueModel/SimpleObstacleShadowing.h \
  base/phyLayer/MappingUtils.h \
  base/phyLayer/AnalogueModel.h \
  ./modules/obstacle/Obstacle.h \
  base/utils/miximkerneldefs.h \
  ./modules/world/annotations/AnnotationManager.h \
  base/phyLayer/Interpolation.h \
  base/phyLayer/MappingBase.h \
  base/modules/BaseWorldUtility.h \
  base/phyLayer/Signal_.h \
  base/utils/FindModule.h \
  inet/Coord.h \
  ./modules/obstacle/ObstacleControl.h \
  base/phyLayer/Mapping.h \
  base/utils/Move.h \
  inet/FWMath.h \
  ./base/messages/AirFrame_m.h
$O/modules/analogueModel/SimplePathlossModel.o: modules/analogueModel/SimplePathlossModel.cc \
  base/phyLayer/AnalogueModel.h \
  base/utils/miximkerneldefs.h \
  base/utils/MiXiMDefs.h \
  base/phyLayer/MappingUtils.h \
  base/modules/BaseWorldUtility.h \
  base/phyLayer/MappingBase.h \
  base/phyLayer/Interpolation.h \
  base/phyLayer/Signal_.h \
  modules/analogueModel/SimplePathlossModel.h \
  inet/FWMath.h \
  ./base/messages/AirFrame_m.h \
  inet/Coord.h \
  base/phyLayer/Mapping.h
$O/modules/analogueModel/TwoRayInterferenceModel.o: modules/analogueModel/TwoRayInterferenceModel.cc \
  base/phyLayer/Signal_.h \
  base/phyLayer/Mapping.h \
  inet/Coord.h \
  ./base/messages/AirFrame_m.h \
  inet/FWMath.h \
  base/phyLayer/MappingUtils.h \
  base/utils/MiXiMDefs.h \
  base/utils/miximkerneldefs.h \
  base/phyLayer/AnalogueModel.h \
  base/phyLayer/Interpolation.h \
  base/phyLayer/MappingBase.h \
  modules/analogueModel/TwoRayInterferenceModel.h \
  base/modules/BaseWorldUtility.h
$O/modules/application/counter/Counter.o: modules/application/counter/Counter.cc \
  base/modules/BatteryAccess.h \
  base/utils/FindModule.h \
  base/connectionManager/BaseConnectionManager.h \
  base/modules/BaseMobility.h \
  modules/mac/ieee80211p/WaveAppToMac1609_4Interface.h \
  modules/utility/Consts80211p.h \
  modules/application/ieee80211p/BaseWaveApplLayer.h \
  base/utils/SimpleAddress.h \
  base/utils/NetwToMacControlInfo.h \
  base/modules/BaseLayer.h \
  inet/FWMath.h \
  ./modules/mobility/traci/TraCICommandInterface.h \
  ./modules/mobility/traci/TraCIConnection.h \
  inet/Coord.h \
  base/utils/Move.h \
  ./modules/obstacle/ObstacleControl.h \
  modules/application/counter/Counter.h \
  ./modules/obstacle/Obstacle.h \
  ./modules/stats/FranciscoStatistics.h \
  base/utils/miximkerneldefs.h \
  ./base/connectionManager/ChannelAccess.h \
  modules/messages/WaveShortMessage_m.h \
  base/modules/BaseModule.h \
  base/utils/MiXiMDefs.h \
  base/utils/PassedMessage.h \
  ./modules/mobility/traci/TraCIMobility.h \
  base/modules/BaseBattery.h \
  base/modules/BaseWorldUtility.h \
  ./modules/mobility/traci/TraCICoord.h \
  base/modules/BaseApplLayer.h \
  base/utils/HostState.h \
  ./modules/world/annotations/AnnotationManager.h \
  ./modules/mobility/traci/TraCIScenarioManager.h \
  base/connectionManager/NicEntry.h \
  ./modules/mobility/traci/TraCIColor.h \
  ./modules/mobility/traci/TraCIBuffer.h
$O/modules/application/distance/Distance.o: modules/application/distance/Distance.cc \
  base/utils/miximkerneldefs.h \
  ./base/connectionManager/ChannelAccess.h \
  ./modules/obstacle/Obstacle.h \
  ./modules/stats/FranciscoStatistics.h \
  ./modules/mobility/traci/TraCIMobility.h \
  base/utils/PassedMessage.h \
  modules/messages/WaveShortMessage_m.h \
  base/utils/MiXiMDefs.h \
  base/modules/BaseModule.h \
  ./modules/mobility/traci/TraCICoord.h \
  base/modules/BaseApplLayer.h \
  base/modules/BaseBattery.h \
  base/modules/BaseWorldUtility.h \
  base/connectionManager/NicEntry.h \
  ./modules/mobility/traci/TraCIScenarioManager.h \
  ./modules/mobility/traci/TraCIColor.h \
  ./modules/mobility/traci/TraCIBuffer.h \
  base/utils/HostState.h \
  modules/application/distance/Distance.h \
  ./modules/world/annotations/AnnotationManager.h \
  base/connectionManager/BaseConnectionManager.h \
  base/modules/BatteryAccess.h \
  base/utils/FindModule.h \
  modules/mac/ieee80211p/WaveAppToMac1609_4Interface.h \
  modules/utility/Consts80211p.h \
  base/modules/BaseMobility.h \
  modules/application/ieee80211p/BaseWaveApplLayer.h \
  base/utils/SimpleAddress.h \
  ./modules/mobility/traci/TraCICommandInterface.h \
  ./modules/mobility/traci/TraCIConnection.h \
  base/modules/BaseLayer.h \
  inet/FWMath.h \
  base/utils/NetwToMacControlInfo.h \
  ./modules/obstacle/ObstacleControl.h \
  base/utils/Move.h \
  inet/Coord.h
$O/modules/application/esbr/Esbr.o: modules/application/esbr/Esbr.cc \
  inet/Coord.h \
  ./modules/obstacle/ObstacleControl.h \
  base/utils/Move.h \
  base/utils/NetwToMacControlInfo.h \
  inet/FWMath.h \
  base/modules/BaseLayer.h \
  ./modules/mobility/traci/TraCIConnection.h \
  ./modules/mobility/traci/TraCICommandInterface.h \
  modules/application/esbr/Esbr.h \
  modules/application/ieee80211p/BaseWaveApplLayer.h \
  base/utils/SimpleAddress.h \
  base/connectionManager/BaseConnectionManager.h \
  base/modules/BatteryAccess.h \
  base/utils/FindModule.h \
  modules/mac/ieee80211p/WaveAppToMac1609_4Interface.h \
  modules/utility/Consts80211p.h \
  base/modules/BaseMobility.h \
  base/utils/HostState.h \
  ./modules/world/annotations/AnnotationManager.h \
  base/connectionManager/NicEntry.h \
  ./modules/mobility/traci/TraCIScenarioManager.h \
  ./modules/mobility/traci/TraCIColor.h \
  ./modules/mobility/traci/TraCIBuffer.h \
  base/modules/BaseWorldUtility.h \
  base/modules/BaseBattery.h \
  ./modules/mobility/traci/TraCICoord.h \
  base/modules/BaseApplLayer.h \
  modules/messages/WaveShortMessage_m.h \
  base/utils/MiXiMDefs.h \
  base/modules/BaseModule.h \
  ./modules/mobility/traci/TraCIMobility.h \
  base/utils/PassedMessage.h \
  ./modules/obstacle/Obstacle.h \
  ./modules/stats/FranciscoStatistics.h \
  base/utils/miximkerneldefs.h \
  ./base/connectionManager/ChannelAccess.h
$O/modules/application/flooding/Flooding.o: modules/application/flooding/Flooding.cc \
  base/utils/miximkerneldefs.h \
  ./base/connectionManager/ChannelAccess.h \
  ./modules/stats/FranciscoStatistics.h \
  ./modules/obstacle/Obstacle.h \
  base/utils/PassedMessage.h \
  ./modules/mobility/traci/TraCIMobility.h \
  base/modules/BaseModule.h \
  base/utils/MiXiMDefs.h \
  modules/messages/WaveShortMessage_m.h \
  base/modules/BaseApplLayer.h \
  ./modules/mobility/traci/TraCICoord.h \
  base/modules/BaseWorldUtility.h \
  base/modules/BaseBattery.h \
  ./modules/mobility/traci/TraCIBuffer.h \
  ./modules/mobility/traci/TraCIColor.h \
  ./modules/mobility/traci/TraCIScenarioManager.h \
  base/connectionManager/NicEntry.h \
  ./modules/world/annotations/AnnotationManager.h \
  base/utils/HostState.h \
  base/modules/BaseMobility.h \
  modules/utility/Consts80211p.h \
  modules/mac/ieee80211p/WaveAppToMac1609_4Interface.h \
  base/utils/FindModule.h \
  base/modules/BatteryAccess.h \
  base/connectionManager/BaseConnectionManager.h \
  base/utils/SimpleAddress.h \
  modules/application/ieee80211p/BaseWaveApplLayer.h \
  ./modules/mobility/traci/TraCIConnection.h \
  ./modules/mobility/traci/TraCICommandInterface.h \
  base/utils/NetwToMacControlInfo.h \
  base/modules/BaseLayer.h \
  inet/FWMath.h \
  modules/application/flooding/Flooding.h \
  base/utils/Move.h \
  ./modules/obstacle/ObstacleControl.h \
  inet/Coord.h
$O/modules/application/ieee80211p/BaseWaveApplLayer.o: modules/application/ieee80211p/BaseWaveApplLayer.cc \
  modules/messages/WaveShortMessage_m.h \
  base/utils/MiXiMDefs.h \
  base/modules/BaseModule.h \
  base/utils/PassedMessage.h \
  ./base/connectionManager/ChannelAccess.h \
  base/utils/miximkerneldefs.h \
  base/utils/HostState.h \
  base/modules/BaseBattery.h \
  base/modules/BaseWorldUtility.h \
  base/modules/BaseApplLayer.h \
  modules/application/ieee80211p/BaseWaveApplLayer.h \
  base/utils/SimpleAddress.h \
  base/utils/FindModule.h \
  base/modules/BatteryAccess.h \
  modules/mac/ieee80211p/WaveAppToMac1609_4Interface.h \
  modules/utility/Consts80211p.h \
  base/modules/BaseMobility.h \
  inet/Coord.h \
  base/utils/Move.h \
  base/modules/BaseLayer.h \
  base/utils/NetwToMacControlInfo.h \
  inet/FWMath.h
$O/modules/application/njl/Njl.o: modules/application/njl/Njl.cc \
  modules/application/ieee80211p/BaseWaveApplLayer.h \
  base/utils/SimpleAddress.h \
  base/connectionManager/BaseConnectionManager.h \
  base/modules/BatteryAccess.h \
  base/utils/FindModule.h \
  modules/mac/ieee80211p/WaveAppToMac1609_4Interface.h \
  modules/utility/Consts80211p.h \
  base/modules/BaseMobility.h \
  ./modules/obstacle/ObstacleControl.h \
  base/utils/Move.h \
  inet/Coord.h \
  ./modules/mobility/traci/TraCICommandInterface.h \
  ./modules/mobility/traci/TraCIConnection.h \
  base/modules/BaseLayer.h \
  inet/FWMath.h \
  base/utils/NetwToMacControlInfo.h \
  ./modules/mobility/traci/TraCIMobility.h \
  base/utils/PassedMessage.h \
  modules/messages/WaveShortMessage_m.h \
  base/utils/MiXiMDefs.h \
  base/modules/BaseModule.h \
  ./base/connectionManager/ChannelAccess.h \
  base/utils/miximkerneldefs.h \
  modules/application/njl/Njl.h \
  ./modules/obstacle/Obstacle.h \
  ./modules/stats/FranciscoStatistics.h \
  base/connectionManager/NicEntry.h \
  ./modules/mobility/traci/TraCIScenarioManager.h \
  ./modules/mobility/traci/TraCIColor.h \
  ./modules/mobility/traci/TraCIBuffer.h \
  base/utils/HostState.h \
  ./modules/world/annotations/AnnotationManager.h \
  ./modules/mobility/traci/TraCICoord.h \
  base/modules/BaseApplLayer.h \
  base/modules/BaseBattery.h \
  base/modules/BaseWorldUtility.h
$O/modules/application/nsf/Nsf.o: modules/application/nsf/Nsf.cc \
  base/modules/BaseWorldUtility.h \
  base/modules/BaseBattery.h \
  ./modules/mobility/traci/TraCICoord.h \
  base/modules/BaseApplLayer.h \
  base/utils/HostState.h \
  modules/application/nsf/Nsf.h \
  ./modules/world/annotations/AnnotationManager.h \
  base/connectionManager/NicEntry.h \
  ./modules/mobility/traci/TraCIScenarioManager.h \
  ./modules/mobility/traci/TraCIColor.h \
  ./modules/mobility/traci/TraCIBuffer.h \
  ./modules/obstacle/Obstacle.h \
  ./modules/stats/FranciscoStatistics.h \
  ./base/connectionManager/ChannelAccess.h \
  base/utils/miximkerneldefs.h \
  modules/messages/WaveShortMessage_m.h \
  base/utils/MiXiMDefs.h \
  base/modules/BaseModule.h \
  ./modules/mobility/traci/TraCIMobility.h \
  base/utils/PassedMessage.h \
  base/modules/BaseLayer.h \
  inet/FWMath.h \
  base/utils/NetwToMacControlInfo.h \
  ./modules/mobility/traci/TraCICommandInterface.h \
  ./modules/mobility/traci/TraCIConnection.h \
  inet/Coord.h \
  ./modules/obstacle/ObstacleControl.h \
  base/utils/Move.h \
  base/connectionManager/BaseConnectionManager.h \
  base/modules/BatteryAccess.h \
  base/utils/FindModule.h \
  modules/utility/Consts80211p.h \
  modules/mac/ieee80211p/WaveAppToMac1609_4Interface.h \
  base/modules/BaseMobility.h \
  modules/application/ieee80211p/BaseWaveApplLayer.h \
  base/utils/SimpleAddress.h
$O/modules/application/traci/TraCIDemo11p.o: modules/application/traci/TraCIDemo11p.cc \
  ./modules/mobility/traci/TraCIConnection.h \
  ./modules/mobility/traci/TraCICommandInterface.h \
  base/utils/NetwToMacControlInfo.h \
  base/modules/BaseLayer.h \
  inet/FWMath.h \
  base/utils/Move.h \
  ./modules/obstacle/ObstacleControl.h \
  inet/Coord.h \
  base/modules/BaseMobility.h \
  modules/utility/Consts80211p.h \
  modules/mac/ieee80211p/WaveAppToMac1609_4Interface.h \
  base/utils/FindModule.h \
  base/modules/BatteryAccess.h \
  modules/application/traci/TraCIDemo11p.h \
  base/connectionManager/BaseConnectionManager.h \
  base/utils/SimpleAddress.h \
  modules/application/ieee80211p/BaseWaveApplLayer.h \
  base/modules/BaseApplLayer.h \
  ./modules/mobility/traci/TraCICoord.h \
  base/modules/BaseWorldUtility.h \
  base/modules/BaseBattery.h \
  ./modules/mobility/traci/TraCIColor.h \
  ./modules/mobility/traci/TraCIBuffer.h \
  ./modules/mobility/traci/TraCIScenarioManager.h \
  base/connectionManager/NicEntry.h \
  ./modules/world/annotations/AnnotationManager.h \
  base/utils/HostState.h \
  ./base/connectionManager/ChannelAccess.h \
  base/utils/miximkerneldefs.h \
  ./modules/obstacle/Obstacle.h \
  base/utils/PassedMessage.h \
  ./modules/mobility/traci/TraCIMobility.h \
  base/modules/BaseModule.h \
  base/utils/MiXiMDefs.h \
  modules/messages/WaveShortMessage_m.h
$O/modules/application/traci/TraCIDemoRSU11p.o: modules/application/traci/TraCIDemoRSU11p.cc \
  ./base/connectionManager/ChannelAccess.h \
  base/utils/miximkerneldefs.h \
  base/utils/PassedMessage.h \
  modules/messages/WaveShortMessage_m.h \
  base/utils/MiXiMDefs.h \
  base/modules/BaseModule.h \
  base/modules/BaseApplLayer.h \
  base/modules/BaseBattery.h \
  base/modules/BaseWorldUtility.h \
  base/utils/HostState.h \
  ./modules/world/annotations/AnnotationManager.h \
  base/modules/BatteryAccess.h \
  base/utils/FindModule.h \
  modules/utility/Consts80211p.h \
  modules/mac/ieee80211p/WaveAppToMac1609_4Interface.h \
  modules/application/traci/TraCIDemoRSU11p.h \
  base/modules/BaseMobility.h \
  modules/application/ieee80211p/BaseWaveApplLayer.h \
  base/utils/SimpleAddress.h \
  base/utils/NetwToMacControlInfo.h \
  base/modules/BaseLayer.h \
  inet/FWMath.h \
  base/utils/Move.h \
  inet/Coord.h
$O/modules/application/traci/TraCITestApp.o: modules/application/traci/TraCITestApp.cc \
  ./modules/obstacle/Obstacle.h \
  base/utils/miximkerneldefs.h \
  base/utils/MiXiMDefs.h \
  base/modules/BaseModule.h \
  ./modules/mobility/traci/TraCIMobility.h \
  base/utils/PassedMessage.h \
  base/utils/asserts.h \
  base/modules/BaseWorldUtility.h \
  base/modules/BaseBattery.h \
  modules/application/traci/TraCITestApp.h \
  base/modules/BaseApplLayer.h \
  ./modules/mobility/traci/TraCICoord.h \
  ./modules/world/annotations/AnnotationManager.h \
  base/utils/HostState.h \
  ./modules/mobility/traci/TraCIColor.h \
  ./modules/mobility/traci/TraCIBuffer.h \
  base/connectionManager/NicEntry.h \
  ./modules/mobility/traci/TraCIScenarioManager.h \
  base/modules/BaseMobility.h \
  modules/mobility/traci/TraCIColor.h \
  base/connectionManager/BaseConnectionManager.h \
  base/utils/FindModule.h \
  base/modules/BatteryAccess.h \
  base/utils/SimpleAddress.h \
  base/modules/BaseLayer.h \
  inet/FWMath.h \
  ./modules/mobility/traci/TraCICommandInterface.h \
  ./modules/mobility/traci/TraCIConnection.h \
  inet/Coord.h \
  ./modules/obstacle/ObstacleControl.h \
  base/utils/Move.h
$O/modules/mac/ieee80211p/Mac1609_4.o: modules/mac/ieee80211p/Mac1609_4.cc \
  modules/mac/ieee80211p/Mac80211pToPhy11pInterface.h \
  base/connectionManager/NicEntry.h \
  base/phyLayer/Interpolation.h \
  base/phyLayer/MappingBase.h \
  base/modules/BaseMacLayer.h \
  base/utils/HostState.h \
  base/phyLayer/BasePhyLayer.h \
  modules/messages/Mac80211Pkt_m.h \
  base/modules/BaseBattery.h \
  base/modules/BaseWorldUtility.h \
  base/utils/PassedMessage.h \
  base/phyLayer/Decider.h \
  base/phyLayer/MappingUtils.h \
  base/utils/MiXiMDefs.h \
  base/modules/BaseModule.h \
  modules/messages/WaveShortMessage_m.h \
  ./base/connectionManager/ChannelAccess.h \
  base/utils/miximkerneldefs.h \
  base/phyLayer/ChannelInfo.h \
  base/phyLayer/AnalogueModel.h \
  base/phyLayer/BaseDecider.h \
  base/messages/MacPkt_m.h \
  base/phyLayer/Mapping.h \
  base/utils/Move.h \
  base/phyLayer/MacToPhyInterface.h \
  inet/Coord.h \
  modules/analogueModel/LogNormalShadowing.h \
  modules/analogueModel/JakesFading.h \
  ./base/messages/AirFrame_m.h \
  modules/phy/SNRThresholdDecider.h \
  modules/phy/Decider80211p.h \
  inet/FWMath.h \
  base/modules/BaseLayer.h \
  base/phyLayer/PhyUtils.h \
  base/utils/NetwToMacControlInfo.h \
  base/messages/ChannelSenseRequest_m.h \
  base/utils/SimpleAddress.h \
  modules/phy/Decider80211pToPhy80211pInterface.h \
  base/phyLayer/DeciderToPhyInterface.h \
  base/phyLayer/ChannelState.h \
  modules/phy/PhyLayer80211p.h \
  modules/mac/ieee80211p/Mac1609_4.h \
  modules/mac/ieee80211p/WaveAppToMac1609_4Interface.h \
  modules/utility/Consts80211p.h \
  base/phyLayer/MacToPhyControlInfo.h \
  base/modules/BaseMobility.h \
  base/connectionManager/BaseConnectionManager.h \
  base/phyLayer/Signal_.h \
  base/utils/FindModule.h \
  base/modules/BatteryAccess.h \
  modules/analogueModel/SimplePathlossModel.h
$O/modules/mobility/traci/TraCIBuffer.o: modules/mobility/traci/TraCIBuffer.cc \
  modules/mobility/traci/TraCICoord.h \
  modules/mobility/traci/TraCIBuffer.h \
  modules/mobility/traci/TraCIConstants.h
$O/modules/mobility/traci/TraCIColor.o: modules/mobility/traci/TraCIColor.cc \
  modules/mobility/traci/TraCIColor.h
$O/modules/mobility/traci/TraCICommandInterface.o: modules/mobility/traci/TraCICommandInterface.cc \
  modules/mobility/traci/TraCIConnection.h \
  modules/mobility/traci/TraCIBuffer.h \
  modules/mobility/traci/TraCIConstants.h \
  ./modules/mobility/traci/TraCIColor.h \
  ./modules/mobility/traci/TraCIBuffer.h \
  modules/mobility/traci/TraCICommandInterface.h \
  ./modules/mobility/traci/TraCICoord.h
$O/modules/mobility/traci/TraCIConnection.o: modules/mobility/traci/TraCIConnection.cc \
  modules/mobility/traci/TraCIConnection.h \
  modules/mobility/traci/TraCIConstants.h \
  ./modules/mobility/traci/TraCIBuffer.h
$O/modules/mobility/traci/TraCIMobility.o: modules/mobility/traci/TraCIMobility.cc \
  inet/FWMath.h \
  ./modules/mobility/traci/TraCIConnection.h \
  ./modules/mobility/traci/TraCICommandInterface.h \
  inet/Coord.h \
  ./modules/obstacle/ObstacleControl.h \
  base/utils/Move.h \
  base/connectionManager/BaseConnectionManager.h \
  base/modules/BatteryAccess.h \
  base/utils/FindModule.h \
  base/modules/BaseMobility.h \
  base/modules/BaseBattery.h \
  base/modules/BaseWorldUtility.h \
  ./modules/mobility/traci/TraCICoord.h \
  base/utils/HostState.h \
  ./modules/world/annotations/AnnotationManager.h \
  base/connectionManager/NicEntry.h \
  ./modules/mobility/traci/TraCIScenarioManager.h \
  ./modules/mobility/traci/TraCIColor.h \
  ./modules/mobility/traci/TraCIBuffer.h \
  ./modules/obstacle/Obstacle.h \
  base/utils/miximkerneldefs.h \
  base/utils/MiXiMDefs.h \
  base/modules/BaseModule.h \
  ./modules/mobility/traci/TraCIMobility.h
$O/modules/mobility/traci/TraCIScenarioManager.o: modules/mobility/traci/TraCIScenarioManager.cc \
  base/utils/miximkerneldefs.h \
  ./modules/mobility/traci/TraCIScenarioManagerInet.h \
  ./modules/obstacle/Obstacle.h \
  ./modules/mobility/traci/TraCIMobility.h \
  base/modules/BaseModule.h \
  base/utils/MiXiMDefs.h \
  ./modules/mobility/traci/TraCICoord.h \
  base/modules/BaseWorldUtility.h \
  base/modules/BaseBattery.h \
  ./modules/mobility/traci/TraCIConstants.h \
  ./modules/mobility/traci/TraCIColor.h \
  ./modules/mobility/traci/TraCIBuffer.h \
  ./modules/mobility/traci/TraCIScenarioManager.h \
  base/connectionManager/NicEntry.h \
  ./modules/world/annotations/AnnotationManager.h \
  base/utils/HostState.h \
  base/modules/BaseMobility.h \
  base/modules/BatteryAccess.h \
  base/utils/FindModule.h \
  base/connectionManager/BaseConnectionManager.h \
  ./modules/mobility/traci/TraCICommandInterface.h \
  ./modules/mobility/traci/TraCIConnection.h \
  inet/FWMath.h \
  base/utils/Move.h \
  ./modules/obstacle/ObstacleControl.h \
  inet/Coord.h
$O/modules/mobility/traci/TraCIScenarioManagerInet.o: modules/mobility/traci/TraCIScenarioManagerInet.cc \
  inet/Coord.h \
  inet/FWMath.h \
  base/utils/MiXiMDefs.h \
  base/utils/miximkerneldefs.h
$O/modules/mobility/traci/TraCIScenarioManagerLaunchd.o: modules/mobility/traci/TraCIScenarioManagerLaunchd.cc \
  base/connectionManager/BaseConnectionManager.h \
  base/utils/FindModule.h \
  inet/FWMath.h \
  ./modules/mobility/traci/TraCIConnection.h \
  ./modules/mobility/traci/TraCICommandInterface.h \
  inet/Coord.h \
  ./modules/obstacle/ObstacleControl.h \
  ./modules/obstacle/Obstacle.h \
  ./modules/mobility/traci/TraCIScenarioManagerLaunchd.h \
  base/utils/miximkerneldefs.h \
  base/utils/MiXiMDefs.h \
  ./modules/mobility/traci/TraCIConstants.h \
  base/modules/BaseWorldUtility.h \
  ./modules/mobility/traci/TraCICoord.h \
  ./modules/world/annotations/AnnotationManager.h \
  ./modules/mobility/traci/TraCIBuffer.h \
  ./modules/mobility/traci/TraCIColor.h \
  base/connectionManager/NicEntry.h \
  ./modules/mobility/traci/TraCIScenarioManager.h
$O/modules/obstacle/Obstacle.o: modules/obstacle/Obstacle.cc \
  base/utils/MiXiMDefs.h \
  ./modules/obstacle/Obstacle.h \
  base/utils/miximkerneldefs.h \
  ./modules/world/annotations/AnnotationManager.h \
  base/utils/FindModule.h \
  inet/Coord.h \
  inet/FWMath.h
$O/modules/obstacle/ObstacleControl.o: modules/obstacle/ObstacleControl.cc \
  ./modules/obstacle/Obstacle.h \
  base/utils/miximkerneldefs.h \
  base/utils/MiXiMDefs.h \
  ./modules/world/annotations/AnnotationManager.h \
  base/utils/FindModule.h \
  inet/FWMath.h \
  inet/Coord.h \
  ./modules/obstacle/ObstacleControl.h
$O/modules/phy/Decider80211p.o: modules/phy/Decider80211p.cc \
  modules/messages/Mac80211Pkt_m.h \
  modules/mac/ieee80211p/Mac80211pToPhy11pInterface.h \
  base/phyLayer/MappingBase.h \
  base/phyLayer/Interpolation.h \
  base/utils/miximkerneldefs.h \
  base/phyLayer/AnalogueModel.h \
  base/phyLayer/BaseDecider.h \
  base/phyLayer/Decider.h \
  modules/messages/AirFrame11p_m.h \
  base/phyLayer/MappingUtils.h \
  base/utils/MiXiMDefs.h \
  ./base/messages/AirFrame_m.h \
  modules/phy/Decider80211p.h \
  base/phyLayer/PhyUtils.h \
  inet/FWMath.h \
  base/messages/ChannelSenseRequest_m.h \
  base/messages/MacPkt_m.h \
  base/phyLayer/Mapping.h \
  base/phyLayer/MacToPhyInterface.h \
  inet/Coord.h \
  modules/utility/Consts80211p.h \
  base/phyLayer/Signal_.h \
  modules/phy/Decider80211pToPhy80211pInterface.h \
  base/utils/SimpleAddress.h \
  base/phyLayer/DeciderToPhyInterface.h \
  base/phyLayer/ChannelState.h \
  modules/phy/DeciderResult80211.h
$O/modules/phy/PhyLayer80211p.o: modules/phy/PhyLayer80211p.cc \
  base/phyLayer/Decider.h \
  modules/messages/AirFrame11p_m.h \
  base/phyLayer/MappingUtils.h \
  modules/analogueModel/PERModel.h \
  base/utils/MiXiMDefs.h \
  base/modules/BaseModule.h \
  modules/analogueModel/SimpleObstacleShadowing.h \
  ./base/connectionManager/ChannelAccess.h \
  base/utils/miximkerneldefs.h \
  base/phyLayer/ChannelInfo.h \
  ./modules/obstacle/Obstacle.h \
  base/phyLayer/BaseDecider.h \
  base/phyLayer/AnalogueModel.h \
  base/connectionManager/NicEntry.h \
  modules/mac/ieee80211p/Mac80211pToPhy11pInterface.h \
  base/phyLayer/Interpolation.h \
  base/phyLayer/MappingBase.h \
  base/utils/HostState.h \
  modules/analogueModel/TwoRayInterferenceModel.h \
  ./modules/world/annotations/AnnotationManager.h \
  base/phyLayer/BasePhyLayer.h \
  base/modules/BaseWorldUtility.h \
  base/modules/BaseBattery.h \
  base/phyLayer/ChannelState.h \
  base/phyLayer/DeciderToPhyInterface.h \
  modules/phy/PhyLayer80211p.h \
  modules/phy/Decider80211pToPhy80211pInterface.h \
  base/phyLayer/Signal_.h \
  base/connectionManager/BaseConnectionManager.h \
  base/modules/BatteryAccess.h \
  modules/analogueModel/SimplePathlossModel.h \
  base/utils/FindModule.h \
  modules/utility/Consts80211p.h \
  base/modules/BaseMobility.h \
  base/phyLayer/MacToPhyControlInfo.h \
  base/phyLayer/MacToPhyInterface.h \
  ./modules/obstacle/ObstacleControl.h \
  base/utils/Move.h \
  base/phyLayer/Mapping.h \
  modules/analogueModel/JakesFading.h \
  modules/analogueModel/BreakpointPathlossModel.h \
  inet/Coord.h \
  modules/analogueModel/LogNormalShadowing.h \
  modules/phy/Decider80211p.h \
  ./base/messages/AirFrame_m.h \
  modules/phy/SNRThresholdDecider.h \
  base/messages/ChannelSenseRequest_m.h \
  inet/FWMath.h \
  base/phyLayer/PhyUtils.h
$O/modules/phy/SNRThresholdDecider.o: modules/phy/SNRThresholdDecider.cc \
  base/phyLayer/Signal_.h \
  base/phyLayer/ChannelState.h \
  base/phyLayer/DeciderToPhyInterface.h \
  base/messages/ChannelSenseRequest_m.h \
  modules/phy/SNRThresholdDecider.h \
  ./base/messages/AirFrame_m.h \
  base/phyLayer/Mapping.h \
  base/phyLayer/BaseDecider.h \
  base/utils/miximkerneldefs.h \
  base/utils/MiXiMDefs.h \
  base/phyLayer/MappingUtils.h \
  base/phyLayer/Decider.h \
  base/phyLayer/MappingBase.h \
  base/phyLayer/Interpolation.h
$O/modules/stats/FranciscoStatistics.o: modules/stats/FranciscoStatistics.cc \
  modules/stats/FranciscoStatistics.h
$O/modules/world/annotations/AnnotationDummy.o: modules/world/annotations/AnnotationDummy.cc \
  ./modules/world/annotations/AnnotationDummy.h
$O/modules/world/annotations/AnnotationManager.o: modules/world/annotations/AnnotationManager.cc \
  base/utils/MiXiMDefs.h \
  base/utils/miximkerneldefs.h \
  ./modules/obstacle/Obstacle.h \
  ./modules/mobility/traci/TraCIScenarioManager.h \
  base/connectionManager/NicEntry.h \
  ./modules/mobility/traci/TraCIColor.h \
  ./modules/mobility/traci/TraCIBuffer.h \
  ./modules/world/annotations/AnnotationManager.h \
  ./modules/mobility/traci/TraCICoord.h \
  base/modules/BaseWorldUtility.h \
  base/utils/FindModule.h \
  base/connectionManager/BaseConnectionManager.h \
  ./modules/obstacle/ObstacleControl.h \
  inet/Coord.h \
  ./modules/mobility/traci/TraCICommandInterface.h \
  ./modules/mobility/traci/TraCIConnection.h \
  inet/FWMath.h
