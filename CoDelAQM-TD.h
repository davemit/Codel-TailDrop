/************************************************************************
* File:  CoDelAQM-TD.h
*
* Purpose:
*  This module implements  a tail drop variant of CoDel AQM
*
* Revisions:
* 3/31/2014: This code is ported from the CableLab's ns2 CoDel-DT contribution.
*      The ns2 CoDel-DT code is available in this patch: http://sourceforge.net/p/nsnam/patches/24/
*
***********************************************************************/
#ifndef CoDelAQMTD_h
#define CoDelAQMTD_h

#include "ListObj.h"
#include "packetListElement.h"



struct DequeResultStruct { Packet *pktPtr; int ok_to_drop; };

class CoDelAQM : public ListObj {

public:
  virtual ~CoDelAQM();
  CoDelAQM();
  virtual void initList(int maxAQMSize);
  virtual void setAQMParams(int maxAQMSize, int priority, int minth, int maxth, int adaptiveMode, double maxp, double weightQ);
  virtual int addElement(ListElement& element);
  virtual ListElement *removeElement();    //dequeue and return
  virtual void adaptAQMParams();
  virtual void newArrivalUpdate(Packet *p);
  virtual void updateStats();
  virtual void printStatsSummary();
  virtual double getAvgQ();
  virtual double getAvgAvgQ();
  virtual double getAvgAvgQLatency();
  virtual double getAvgAvgQLoss();
  virtual double getDropP();
  virtual double getAvgDropP();
  virtual double getAvgMaxP();

  virtual void channelIdleEvent();
  virtual void channelStatus(double accessDelayMeasure, double channelUtilizationMeasure, double consumptionMeasure);

  int maxth;
  int minth;
  double maxp;
  double weightQ;
  //define the base time constant to use for averaging
  double filterTimeConstant;
  int flowPriority;


  double lastUpdateTime;

  double numberIncrements;
  double numberDecrements;
  double numberTotalChannelIdleEvents;
  double numberValidChannelIdleEvents;

  double queueLatencyTotalDelay;
  double queueLatencySamplesCount;
  double queueLatencyMonitorTotalArrivalCount;
  double avgPacketLatency;
  double avgAvgQLatency;
  double avgAvgQLatencySampleCount;
  double lastQLatencySampleTime;

  double avgQLoss;
  double avgQLossCounter;
  double avgQLossSampleCounter;
  double lastQLossSampleTime;

  double avgAvgQLoss;
  double avgAvgQLossSampleCounter;

  double avgQ;
  double dropP;
  double q_time;
  double avgDropP;
  double avgDropPSampleCount;
  double avgAvgQ;
  double avgAvgQPSampleCount;
  double avgMaxP;
  double avgMaxPSampleCount;

//Real codel stuff
    // Static state (user supplied parameters)
    double target_;         // target queue size (in time, same units as clock)
    double interval_;       // width of moving time window over which to compute min

    // Dynamic state used by algorithm
    double first_above_time_; // when we went (or will go) continuously above
                              // target for interval
    double drop_next_;      // time to drop next packet (or when we dropped last)
    int count_;             // how many drops we've done since the last time
                            // we entered dropping state.
    int dropping_;          // = 1 if in dropping state.
    int maxpacket_;         // largest packet we've seen so far (this should be
                            // the link's MTU but that's not available in NS)

    int curq_;        // current qlen seen by arrivals
    double d_exp_;    // delay seen by most recently dequeued packet
    int drop_now_;    //flag indicates next packet enqueued should be dropped (unless queue is almost empty)

    double numberDrops;
    double numberPackets;


protected:


private:

  int targetQueueLevel;
  double targetDelayLevel;
  double upperLatencyLimit;
  double upperLatencyThresh;
  double lowerLatencyThresh;
  double alpha;
  double beta;
  
  //0 is FCFS, 1 is CoDelAQM 
  int adaptiveMode;
  double avgTxTime;
  double accessDelayMeasure;
  double channelUtilizationMeasure;
   double consumptionMeasure;


  //states:  0 : NOT initialized
  //states:  1 : REGION 1
  //states:  2 : REGION 2
  //states:  3 : REGION 3
  int stateFlag;
  double updateFrequency;
  int bytesQueued;
  double avgServiceRate;
  double byteArrivals;
  double byteDepartures;
  double lastRateSampleTime;
  double lastMonitorSampleTime;
  double rateWeight;
  double avgArrivalRate;

  double CoDelAQMupdateFrequency;
  int    CoDelAQMUpdateCountThreshold;
  int    CoDelAQMUpdateCount;
  double CoDelAQMLastSampleTime;
  double CoDelAQMByteArrivals;
  double CoDelAQMByteDepartures;
  double avgCoDelAQMArrivalRate;
  double avgCoDelAQMServiceRate;
  double CoDelAQMRateWeight;

  double CoDelAQMQueueDelaySamples;
  double CoDelAQMQueueLevelSamples;
  double CoDelAQMAccessDelaySamples;
  double CoDelAQMConsumptionSamples;
  double CoDelAQMChannelUtilizationSamples;


  double updateAvgQLoss();
  double updateAvgQ();
  void updateRates();
  double updateAvgQLatency();
  void  updateMonitors();
 
//Real codel stuff
  void reset();
  int  doDropIn(Packet *pktPtr);
  int  doDropIn2(Packet *pktPtr);
  int  doDropOut(Packet *pktPtr);
  void switchToNotDropping();
  void switchToDropping();
  double control_law(double);


};


#endif
