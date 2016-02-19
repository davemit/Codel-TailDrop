/************************************************************************
* File:  CoDelAQM-TD.cc
*
* Purpose:
*  This module implements a Tail Drop variant of CoDel AQM. 
*  
* Revisions:
* 3/31/2014: This code is ported from the CableLab's ns2 CoDel-DT contribution.
*      The ns2 CoDel-DT code is available in this patch: http://sourceforge.net/p/nsnam/patches/24/
*
************************************************************************/
#include "CoDelAQM-TD.h"
#include <iostream>
#include <math.h>
#include <random.h>

#include "object.h"

#include "globalDefines.h"
#include "docsisDebug.h"

//#define TRACEME 0
//#define FILES_OK 0


CoDelAQM::CoDelAQM() : ListObj()
{
  stateFlag = 0;
  accessDelayMeasure =0;
  channelUtilizationMeasure=0;
  consumptionMeasure=0;
  //Just set to this for curTime
  targetDelayLevel = CoDel_TARGET_LATENCY;
  upperLatencyLimit = 0;
  upperLatencyThresh = 0;
  lowerLatencyThresh = 0;
#ifdef TRACEME
  printf("CoDelAQM: constructed list \n");
  fflush(stdout);
#endif
  reset();
}

CoDelAQM::~CoDelAQM()
{
#ifdef TRACEME
  printf("CoDelAQM: destructed list \n");
#endif
}

void CoDelAQM::initList(int maxListSize)
{

  ListObj::initList(maxListSize);

#ifdef TRACEME
  printf("CoDelAQM:initList: size %d \n",maxListSize);
#endif
  flowPriority = 0;
  minth = 0;
  maxth = maxListSize;
  maxp = 0.10;
  weightQ = 1.0;
  filterTimeConstant = 0.002;
  adaptiveMode=0; // 0:FCFS; 1: CoDel

//For Adaptive CoDel
  targetQueueLevel = 0;
  alpha = 0.0;
  beta = 0.0;
//For delayBasedCoDel
//Just set to this for curTime
  targetDelayLevel = CoDel_TARGET_LATENCY;
  lowerLatencyThresh = targetDelayLevel/2;
  upperLatencyThresh =  targetDelayLevel;
  upperLatencyLimit =  2 * upperLatencyThresh;

 
  
  avgPacketLatency = 0.0;
  queueLatencyTotalDelay = 0.0;
  queueLatencySamplesCount = 0;
  queueLatencyMonitorTotalArrivalCount = 0;
  avgAvgQLatency= 0;
  avgAvgQLatencySampleCount= 0;
  lastQLatencySampleTime = 0;


  avgQLoss = 0;
  avgQLossCounter=0;
  avgQLossSampleCounter=0;
  lastQLossSampleTime = 0;

  avgAvgQLoss = 0;
  avgAvgQLossSampleCounter=0;

  lastRateSampleTime = 0;
  lastMonitorSampleTime = 0;


  avgQ = 0.0;
  dropP = 0.0;
  avgMaxP=0;
  avgMaxPSampleCount=0;
  avgDropP=0;
  avgDropPSampleCount=0;
  avgAvgQ = 0;
  avgAvgQPSampleCount= 0;
  q_time = 0.0;
  avgTxTime = (double)1500 * 8 / (double) 5000000;


 lastUpdateTime =0.0;
 stateFlag = 1;
 updateFrequency = .050;
 bytesQueued = 0;
 avgServiceRate=0;
 avgArrivalRate=0;
 byteArrivals = 0;
 byteDepartures = 0;
 rateWeight = .02;
 lastQLatencySampleTime =  0;

 CoDelAQMupdateFrequency = AQM_ADAPTATION_TIME;
 CoDelAQMUpdateCountThreshold = 0;
 CoDelAQMUpdateCount = 0;
 CoDelAQMLastSampleTime = 0;
 CoDelAQMByteArrivals =0;
 CoDelAQMByteDepartures = 0;
 avgCoDelAQMArrivalRate = 0;
 avgCoDelAQMServiceRate = 0;
 CoDelAQMRateWeight = .02;

 CoDelAQMQueueDelaySamples = 0;
 CoDelAQMQueueLevelSamples = 0;
 CoDelAQMAccessDelaySamples =0;
 CoDelAQMConsumptionSamples = 0;
 CoDelAQMChannelUtilizationSamples = 0;

//Real CoDel Stuff
traceFlag = 1;

//#ifdef TRACEME
//  if (traceFlag == 1) {
      printf("CoDelAQM::init: ListID:%d,  adaptiveMode:%d, Priority:%d, maxListSize:%d, targetDelayLevel:%3.6f, lowerLatencyThresh: %3.6f, upperLatencyThresh: %3.6f, upperLatencyLimit: %3.6f\n",
            listID, adaptiveMode, flowPriority,maxListSize, targetDelayLevel, lowerLatencyThresh, upperLatencyThresh, upperLatencyLimit);
//  }
//#endif


}

void CoDelAQM::reset()
{
#ifdef TRACEME
  printf("CoDelAQM:reset  \n");
#endif
    d_exp_ = 0.0;
    dropping_ = 0;
    first_above_time_ = -1;
    drop_next_ = 0.0;
    maxpacket_ = 256;
    count_ = 0;

    numberDrops=0;
    numberPackets=0;

    interval_ = 0.100;
    target_ = 0.020;
}

/*************************************************************
* routine:
*   int  CoDelAQM::addElement(ListElement& element)
*
* Function: this routine inserts the ListElement to the tail of 
* the list.
*           
* inputs: 
*    ListElement& element : the element to be inserted
*
* outputs:
*  Returns a SUCCESS or FAILURE.
*        Possible failures:
*             -malloc fails
*             -list already has  > MAXLISTSIZE elements
*
***************************************************************/
int  CoDelAQM::addElement(ListElement& element)
{
  int rc = SUCCESS;
  double curTime =  Scheduler::instance().clock();


#ifdef TRACEME
  if (traceFlag == 1) {
   printf("CoDelAQM:addElement(%lf)(listID:%d) (adaptiveMode:%d) listsize:%d, avgQ:%3.1f, minth:%d, maxth:%d,dropP:%3.3f, avgPacketLatency:%3.6f \n ",
      curTime,listID,adaptiveMode,curListSize,avgQ,minth,maxth,dropP,avgPacketLatency);
   }
#endif

  Packet *p = ((packetListElement &)element).getPacket();
  struct hdr_cmn *ch = HDR_CMN(p);
  byteArrivals+=ch->size();
  numberPackets++;
  CoDelAQMByteArrivals += ch->size();
  ch->ts_ = curTime;

  if (maxpacket_ < ch->size_)
    // keep track of the max packet size.
    maxpacket_ = ch->size_;

//This needs to be every packet
  updateAvgQ();
  updateMonitors();

#ifdef TRACEME
  if (traceFlag == 1) {
   printf("CoDelAQM:addElement(%lf) UPDATE AvgQ listsize:%d, avgQ:%3.1f, minth:%d, maxth:%d (dropP:%3.4f, count_:%d, avgArrivalRate:%f, avgServiceRate:%f, avgPacketLatency:%f \n",
      curTime,curListSize,avgQ,minth,maxth,dropP,count_,avgArrivalRate,avgServiceRate, avgPacketLatency);
   }
#endif


//If 1, we are doing CoDel, else just insert
  if (adaptiveMode == 1) {

    rc = doDropIn(p);

#ifdef TRACEME
  if (traceFlag == 1) {
    printf("CoDelAQM:addElement(%lf) (ListID:%d) after doDropIn, rc:%d  (dropping:%d, drop_next_:%lf) \n",
         curTime,listID,rc,dropping_,drop_next_);
  }
#endif

  }
  //rc is SUCCESS  indicting the pkt can be queued, else the caller should drop it
  if (rc == SUCCESS) {
    rc = ListObj::addElement(element);
    if (rc == SUCCESS) {
      Packet *p = ((packetListElement &)element).getPacket();
      struct hdr_cmn *ch = HDR_CMN(p);
      bytesQueued+=ch->size();
    }
    
  }

#ifdef TRACEME
  if (traceFlag == 1) {
    printf("CoDelAQM:addElement(%lf)(listID:%d) exit rc=%d, listsize:%d, avgQ:%3.1f, min-t:%d, max-t:%d, dropP:%3.3f, bytesQueued:%d \n ",
      curTime,listID,rc,curListSize,avgQ,minth,maxth,dropP, bytesQueued);
  }
#endif


  avgQLossSampleCounter++;
  if (rc == FAILURE) {
    avgQLossCounter++;
    numberDrops++;
#ifdef TRACEME
    if (traceFlag == 1) {
      printf("CoDelAQM:addElement(%lf)(listID:%d) DROP listsize:%d, numberDropped:%f, count_:%d\n ", curTime,listID,curListSize,numberDrops,count_);
    }
#endif
  }
  return rc;
}

/*************************************************************
* routine: ListElement * ListObj::removeElement()
*
* Function:
*
* inputs: 
*
* outputs:
*   The element at the top of the list is removed and returned.
*   A NULL is returned if the list is empty
*
***************************************************************/
ListElement * CoDelAQM::removeElement()
{
  packetListElement *tmpPtr = NULL;
  double curTime =   Scheduler::instance().clock();
  double x = 0;


#ifdef TRACEME
  if (traceFlag == 1) {
    printf("CoDelAQM::removeElement:(%lf) (listID:%d) start size: %d (bytesQueued:%d)\n", curTime, listID, curListSize,bytesQueued);
  }
#endif

  tmpPtr = (packetListElement *)ListObj::removeElement();
  if (curListSize == 0) {
    q_time = curTime;
  }

  if (tmpPtr != NULL) {

    Packet *p = ((packetListElement *)tmpPtr)->getPacket();
    struct hdr_cmn *ch = HDR_CMN(p);
    x = curTime - ch->ts_;
    byteDepartures+=ch->size();
    CoDelAQMByteDepartures += ch->size();
    bytesQueued-=ch->size();
    if (bytesQueued < 0) {
      printf("CoDelAQM::removeElement:(%lf)(listID:%d)  TROUBLE : bytesQueued went negative with this pkt size: %d\n", 
        curTime,listID, ch->size());
      bytesQueued  = 0;
    }

    if (x > 0) {
      queueLatencyTotalDelay += x;
      queueLatencySamplesCount++;

    }
    int rc = doDropOut(p);
  }
  else if (dropping_)
    switchToNotDropping();


#ifdef TRACEME 
  if (traceFlag == 1) {
      printf("CoDelAQM::removePacket(%lf): ListID:%d CurListSize:%d Packet delay sample: %3.6f, queueLatencyAvg:%3.6f(#samples:%6.0f), avgPacketLatency:%3.6f \n",
            curTime,listID, curListSize, x, queueLatencyTotalDelay,queueLatencySamplesCount,avgPacketLatency);
  }
#endif
  return tmpPtr;
}

//Called only when a new packet arrives  (which might be dropped....)
void CoDelAQM::newArrivalUpdate(Packet *p)
{

 queueLatencyMonitorTotalArrivalCount++;

// We want to monitor the true arrival rate to the queue...
// so ignore when a new packet arrives ....we only
// caree when it gets queued.
  return;

  double curTime =  Scheduler::instance().clock();
  struct hdr_cmn *ch = HDR_CMN(p);
  byteArrivals+=ch->size();
  CoDelAQMByteArrivals += ch->size();
}


void CoDelAQM::updateStats()
{
  double curTime =   Scheduler::instance().clock();

}

void CoDelAQM::printStatsSummary()
{
  double curTime =   Scheduler::instance().clock();

#ifdef TRACEME
  printf("CoDelAQM:ListObj:printStatsSummary:(%lf), avgAvgQ: %3.1f, avgDropP: %3.3f, avgQLatency:%3.6f\n",curTime,getAvgAvgQ(), getAvgDropP(),getAvgAvgQLatency());
#endif
}


/*************************************************************
* routine: void CoDelAQM::setAQMParams(int maxAQMSizeP, int priorityP, int minthP, int maxthP, int adaptiveModeP, double maxpP, double filterTimeConstantP)
*
* Function: this routine adapts the AQM parameters.
*   The algorithm sets CoDel specific parameters.
*           
* inputs: 
*   int maxAQMSizeP
*   int priorityP 
*   int minthP 
*   int maxthP 
*   int adaptiveModeP
*         0: FIFO
*         1: CoDel
*   double maxpP
*   double filterTimeConstantP)
* outputs:
*
***************************************************************/
void CoDelAQM::setAQMParams(int maxAQMSizeP, int priorityP, int minthP, int maxthP, int adaptiveModeP, double targetP, double intervalP)
{

  flowPriority = priorityP;
  MAXLISTSIZE = maxAQMSizeP;
  minth = minthP;
  maxth = maxthP;
  double curTime =   Scheduler::instance().clock();

  adaptiveMode= adaptiveModeP;

  interval_ = intervalP;
  target_ =  targetP;

  if (!((adaptiveMode == 0) || (adaptiveMode ==1))) {
    printf("CoDelAQM:setAQMParams: (listID:%d) WARNING BAD adaptiveMode:%d, set to FCFS \n", listID, adaptiveMode);
    adaptiveMode = 0;
  }

  printf("CoDelAQM:setAQMParams(%lf) (listID:%d) adaptiveMode:%d,  maxCapacity:%d, target:%3.3f,   interval:%3.3f\n", curTime,listID,adaptiveMode,MAXLISTSIZE, target_, interval_);

  traceFlag = 1;
}

/*************************************************************
* routine: void CoDelAQM::adaptAQMParams()
*
* Function: this routine adapts the AQM parameters.
*   The algorithm is TBD.
*           
* inputs: 
*
* outputs:
*
***************************************************************/
void CoDelAQM::adaptAQMParams()
{

#ifdef TRACEME
  printf("CoDelAQM:adaptAQMParams: updated minth:%d  maxth:%d  maxp:%2.2f \n",minth,maxth,maxp);
#endif
}


double CoDelAQM::getAvgQ()
{
  return(avgQ);
}

double CoDelAQM::getAvgAvgQ()
{
  if (avgAvgQPSampleCount > 0)
    return(avgAvgQ/(double)avgAvgQPSampleCount);
  else
    return(0);
}

double CoDelAQM::getAvgAvgQLatency()
{
  if (avgAvgQLatencySampleCount > 0)
    return(avgAvgQLatency/(double)avgAvgQLatencySampleCount);
  else
    return(0);
}

double CoDelAQM::getAvgAvgQLoss()
{
  if (avgAvgQLossSampleCounter > 0)
    return(avgAvgQLoss/(double)avgAvgQLossSampleCounter);
  else
    return(0);
}


double CoDelAQM::getDropP()
{
  return(dropP);
}

double CoDelAQM::getAvgDropP()
{
  double tmpAvgDropP = 0.0;

  if (avgDropPSampleCount > 0)
    tmpAvgDropP = avgDropP / avgDropPSampleCount;
  
  return(tmpAvgDropP);
}

double CoDelAQM::getAvgMaxP()
{
  double tmpAvgMaxP = 0.0;

  if (avgMaxPSampleCount > 0)
    tmpAvgMaxP = avgMaxP / avgMaxPSampleCount;
  
  return(tmpAvgMaxP);
}



/*************************************************************
* routine: double CoDelAQM::updateAvgQLoss()
*
* Function: this routine computes and updates the loss monitor
*           
* inputs: 
*
* outputs:
*    returns the updated value
*
***************************************************************/
double CoDelAQM::updateAvgQLoss()
{
  double curTime =   Scheduler::instance().clock();
  double sampleAvg1= 0;
  double returnAvg = 0.0;
  double sampleTime = curTime - lastQLatencySampleTime;


 
 if (avgQLossSampleCounter == 0)
   return avgQLoss;

 sampleAvg1 = avgQLossCounter / avgQLossSampleCounter;


  returnAvg  = (1-filterTimeConstant) * avgQLoss + filterTimeConstant * sampleAvg1;

#ifdef TRACEME
  if (traceFlag == 1) {
  printf("CoDelAQM:updateAvgQLoss:(%lf) listID:%d  curLen:%d, avgQ:%3.3f, lossSample%f, numberSamples:%f, MovingAvg:%f\n",
       curTime,listID,curListSize, avgQ,sampleAvg1,avgQLossSampleCounter,returnAvg);
   }
#endif


  avgAvgQLoss += returnAvg;
  avgAvgQLossSampleCounter++;

  lastQLossSampleTime = curTime;
  avgQLoss = returnAvg;
  avgQLossCounter=0;
  avgQLossSampleCounter=0;
  lastQLossSampleTime = curTime;

  return returnAvg;

}


/*************************************************************
* routine: double CoDelAQM::updateAvgQLatency()
*
* Function: this routine computes and updates the global
*      AvgQ variable.
*           
* inputs: 
*
* outputs:
*    returns the updated value
*
***************************************************************/
double CoDelAQM::updateAvgQLatency()
{
  double curTime =   Scheduler::instance().clock();
  double sampleAvg1= 0;
  double returnAvg = 0.0;
  double sampleTime = curTime - lastQLatencySampleTime;



 
 if (queueLatencySamplesCount == 0)
   return avgPacketLatency;

 sampleAvg1 = queueLatencyTotalDelay / queueLatencySamplesCount;


  returnAvg  = (1-filterTimeConstant) * avgPacketLatency + filterTimeConstant * sampleAvg1;


  avgAvgQLatency+=returnAvg;
  avgAvgQLatencySampleCount++;
  lastQLatencySampleTime = curTime;

  avgPacketLatency = returnAvg;

#ifdef TRACEME
  if (traceFlag == 1) {
  printf("CoDelAQM:updateAvgQLatency:(%lf) listID:%d  curLen:%d, avgQ:%3.3f, sampleAvg:%3.6f, avgPacketLatency:%3.6f based on %f samples\n",
       curTime,listID,curListSize, avgQ,sampleAvg1,avgPacketLatency,queueLatencySamplesCount);
   }
#endif

  queueLatencyTotalDelay = 0.0;
  queueLatencySamplesCount = 0;
  queueLatencyMonitorTotalArrivalCount = 0;

  return returnAvg;

}



/*************************************************************
* routine: double CoDelAQM::updateAvgQ
*
* Function: this routine computes the global avgQ variable
*           
* inputs: 
*
* outputs:
*    returns the updated value.
*
***************************************************************/
double CoDelAQM::updateAvgQ()
{
  double curTime =   Scheduler::instance().clock();
  double returnAvg = 0.0;


#ifdef TRACEME
  if (traceFlag == 1) {
  printf("CoDelAQM:updateAvgQ:(%lf)  curListSize:%d, current avgQ:%3.3f, filterTimeConstant:%2.4f, q_time:%3.6f \n",
       curTime,curListSize,avgQ,filterTimeConstant,q_time);
  fflush(stdout);
   }
#endif


  if (curListSize == 0) {
///$A808
    double quietTime = 0;
    double power = 0;
    double x=0;

    returnAvg = 0;
#ifdef TRACEME
  if (traceFlag == 1) {
    printf("CoDelAQM:updateAvgQ:(%lf) case of empty queue quietTime: %6.6f, power:%3.6f, x:%3.6f, returnAvg:%3.6f \n",
       curTime,quietTime,power,x,returnAvg);
   }
#endif
  }
  else {
    returnAvg = (1-filterTimeConstant)*avgQ + filterTimeConstant*curListSize;
#ifdef TRACEME
  if (traceFlag == 1) {
    printf("CoDelAQM:updateAvgQ:(%lf) case of queue len:%d, current avgQ:%3.3f, updated avg:%3.3f \n",
       curTime,curListSize, avgQ,returnAvg);
   }
#endif
  }
  avgAvgQ+=returnAvg;
  avgAvgQPSampleCount++;

#ifdef TRACEME
  if (traceFlag == 1) {
  printf("CoDelAQM:updateAvgQ:(%lf) curLen:%d, avgQ:%3.3f, returnAvg:%3.3f avgAvgQ:%3.3f (filterTimeConstant:%3.3f)\n",
       curTime,curListSize, avgQ, returnAvg,avgAvgQ/avgAvgQPSampleCount,filterTimeConstant);
   }
#endif
  avgQ = returnAvg;
  return returnAvg;

}

void CoDelAQM::updateRates()
{
  double curTime =   Scheduler::instance().clock();
  double sampleTime = curTime - lastRateSampleTime;
  double ArrivalRateSample=0;
  double DepartureRateSample=0;

#ifdef TRACEME
  if (traceFlag == 1) {
  printf("CoDelAQM:updateRates:(%lf)  byteArrivals:%f, byteDepartures:%f, time since last rate sample:%f  \n", curTime,byteArrivals,byteDepartures,sampleTime);
   }
#endif

//  if (sampleTime < updateFrequency)
//    return;

  if (sampleTime > 0 ) {
    ArrivalRateSample = byteArrivals*8/sampleTime;
    DepartureRateSample = byteDepartures*8/sampleTime;
  }

  avgArrivalRate = (1-rateWeight)*avgArrivalRate + rateWeight*ArrivalRateSample;


  avgServiceRate = (1-rateWeight)*avgServiceRate + rateWeight*DepartureRateSample;

#ifdef TRACEME
  if (traceFlag == 1) {
    printf("CoDelAQM:updateRates:(%lf)  avgArrivalRate:%f (sample:%f),  avgServiceRate:%f(sample:%f) \n",
       curTime,avgArrivalRate,ArrivalRateSample,avgServiceRate,DepartureRateSample);
   }
#endif


  byteArrivals =0;
  byteDepartures =0;
  lastRateSampleTime = curTime;
}

/*************************************************************
* routine:
*  void CoDelAQM::channelStatus(double accessDelayMeasure, double channelUtilizationMeasure, double consumptionMeasure)
*
* Function: this method is perioodicaly called to inform the AQM of current channel conditions. 
*        Based on this information, the routine might adapt its AQM operating settings.  It
*        can only do this if a  CoDelAQMupdateFrequency amount of time has passed since the last
*        update.
*
* Inputs: 
*  double accessDelayMeasure
*  double channelUtilizationMeasure
*  double consumptionMeasure
*
* outputs:
*
*  This method will possible update the following:
*      -The minth or maxth range
*      - maxp
*      -
*****************************************************************/
void CoDelAQM::channelStatus(double accessDelayMeasure, double channelUtilizationMeasure, double consumptionMeasure)
{
  double curTime =   Scheduler::instance().clock();
  double sampleTime = curTime - CoDelAQMLastSampleTime;
  double ArrivalRateSample=0;
  double DepartureRateSample=0;


//  printf("CoDelAQM:channelStatus(%lf) WARNING: Why am I getting this ???\n", curTime);
  return;

}


void CoDelAQM::channelIdleEvent()
{
  double curTime =   Scheduler::instance().clock();
  double updateTime = curTime - lastUpdateTime;


  return;

}

/*************************************************************
* routine: void CoDelAQM::updateMonitors()
*
* Function: this routine is called periodically (set 
*   by updateFrequency) to update the following stats:
*        -rates :
*        -Avg Q Loss
*        -Avg Q Level
*        -Avg Q Latency
*
*  After the update if then makes an entry in the traceAQM trace file.
*           
* inputs: 
*
* outputs:
*
***************************************************************/
void CoDelAQM::updateMonitors()
{
  double curTime =   Scheduler::instance().clock();
  double sampleTime = curTime - lastMonitorSampleTime;
  double sampleAvgQ = 0.0;
  double sampleAvgLatency = 0.0;


  if (sampleTime < updateFrequency)
    return;

#ifdef TRACEME
  if (traceFlag == 1) {
    printf("CoDelAQM:updateMonitors:(%lf) listID:%d  byteArrivals:%f, byteDepartures:%f, time since last rate sample:%f  \n", curTime,listID,byteArrivals,byteDepartures,sampleTime);
  }
#endif

// this is done with each insert and remove
  sampleAvgQ = updateAvgQ();
  sampleAvgLatency =  updateAvgQLatency();
  updateRates();

  lastMonitorSampleTime = curTime;

#ifdef  FILES_OK
  if (traceFlag == 1) {
    FILE* fp = NULL;
    char traceString[32];
    char *tptr = traceString;
    sprintf(tptr,"traceAQM%d.out",listID);
    fp = fopen(tptr, "a+");
    fprintf(fp,"1 %6.6f\t%d\t%d\t%6.6f\t%6.6f\t%d\t%3.3f\t%d\t%d\t%6.6f\t%3.3f\t%12.0f\t\t%12.0f\t%3.6f\t%3.6f\n",
     Scheduler::instance().clock(),listID,dropping_,drop_next_,first_above_time_,count_, numberDrops/numberPackets,curListSize, bytesQueued,d_exp_,sampleAvgQ, avgArrivalRate, avgServiceRate, sampleAvgLatency,getAvgAvgQLatency());
    fclose(fp);
  }
#endif
}

/*************************************************************
* Routine: int CoDelAQM:doDropIn(Packet *pktPtr)
*
* Function:
*   Internal routine that determines if the packet that has arrived should be dropped.
*
* Inputs: 
*    pktPtr: reference to a pkt that just arrived
*
* Outputs:
*     rc :  SUCCESS (and packet should NOT be dropped)
*           FAILURE (and packet should be dropped)
*
*  Pseudo code:
*    if in dropping_ state
*      if drop_now_ set, return FAILURE      
*    else if queue full, return FAILURE
*    else return SUCCESS
*
*    Might move to dropping
*    Might move out of dropping
***************************************************************/
int  CoDelAQM::doDropIn(Packet *pktPtr)
{
  int rc = SUCCESS;
  double curTime =   Scheduler::instance().clock();
  double sampleAvgQ = 0.0;
  double sampleAvgLatency = 0.0;

#ifdef TRACEME
  if (traceFlag == 1) {
    printf("CoDelAQM::doDropIn:(%lf) (listID:%d), curListSize:%d, bytesQueued:%d (dropping_:%d, drop_now_:%d, drop_next_:%6.6f\n", curTime, listID,curListSize,bytesQueued,dropping_,drop_now_,drop_next_);
  }
#endif
  if (curListSize + 1 >= MAXLISTSIZE) {
    rc = FAILURE;
  } else {

    if (bytesQueued <= maxpacket_) {
       switchToNotDropping();
    } else {
      if (dropping_) {
        if (drop_now_ == 1) {
          rc = FAILURE;
          drop_now_ = 0;
          drop_next_ = control_law(curTime);
        } else if (drop_next_ == 0.0) {
          drop_next_ = control_law(curTime);
        } else if (curTime > drop_next_) {
          rc = FAILURE;
          //Schedule next drop
          drop_next_ = control_law(drop_next_);
        }
      }
    }
  }
  if (rc == FAILURE) {
    count_++;
  }
  sampleAvgQ = updateAvgQ();
  sampleAvgLatency =  updateAvgQLatency();
#ifdef  FILES2_OK
  if (traceFlag == 1) {
    FILE* fp = NULL;
    char traceString[32];
    char *tptr = traceString;
    sprintf(tptr,"traceAQM%d.out",listID);
    fp = fopen(tptr, "a+");
    fprintf(fp,"2 %6.6f\t%d\t%d\t%6.6f\t%6.6f\t%d\t%3.3f\t%d\t%d\t%6.6f\t%3.3f\t%12.0f\t\t%12.0f\t%3.6f\t%3.6f\n",
     Scheduler::instance().clock(),listID,dropping_,drop_next_,first_above_time_,count_, numberDrops/numberPackets,curListSize, bytesQueued,d_exp_,sampleAvgQ, avgArrivalRate, avgServiceRate, sampleAvgLatency,getAvgAvgQLatency());
    fclose(fp);
 }
#endif
  return rc;
}

/*************************************************************
* Routine: int CoDelAQM:doDropIn(Packet *pktPtr)
*
* Function:
*   Internal routine that determines if the packet that has arrived should be dropped.
*
* Inputs: 
*    pktPtr: reference to a pkt that just arrived
*
* Outputs:
*     rc :  SUCCESS (and packet should NOT be dropped)
*           FAILURE (and packet should be dropped)
*
*  Pseudo code:
*    if in dropping_ state
*      if drop_now_ set, return FAILURE      
*    else if queue full, return FAILURE
*    else return SUCCESS
*
***************************************************************/
int  CoDelAQM::doDropIn2(Packet *pktPtr)
{
  int rc = SUCCESS;
  double curTime =   Scheduler::instance().clock();
  double sampleAvgQ = 0.0;
  double sampleAvgLatency = 0.0;

#ifdef TRACEME
  if (traceFlag == 1) {
    printf("CoDelAQM::doDropIn:(%lf) (listID:%d), curListSize:%d, bytesQueued:%d (dropping_:%d, drop_now_:%d, drop_next_:%6.6f\n", curTime, listID,curListSize,bytesQueued,dropping_,drop_now_,drop_next_);
  }
#endif
  if (dropping_) {
    if (drop_now_ == 1) {
      rc = FAILURE;
      drop_now_ = 0;
      drop_next_ = control_law(curTime);
    }
    else if (curTime > drop_next_) {
      rc = FAILURE;
      //Schedule next drop
      drop_next_ = control_law(curTime);
    }
    //double check
    if (rc == FAILURE) {
       if (bytesQueued <= maxpacket_) {
         switchToNotDropping();
       }
       //No need to check the d_exp_ monitor - it will be done soon
    }
  }
  else { 
    //if this pkt fills the LIST, drop it
    if (curListSize + 1 >= MAXLISTSIZE)
      rc = FAILURE;
  }
  if (rc == FAILURE) {
    count_++;
  }
  sampleAvgQ = updateAvgQ();
  sampleAvgLatency =  updateAvgQLatency();
#ifdef  FILES2_OK
  if (traceFlag == 1) {
    FILE* fp = NULL;
    char traceString[32];
    char *tptr = traceString;
    sprintf(tptr,"traceAQM%d.out",listID);
    fp = fopen(tptr, "a+");
    fprintf(fp,"2 %6.6f\t%d\t%d\t%6.6f\t%6.6f\t%d\t%3.3f\t%d\t%d\t%6.6f\t%3.3f\t%12.0f\t\t%12.0f\t%3.6f\t%3.6f\n",
     Scheduler::instance().clock(),listID,dropping_,drop_next_,first_above_time_,count_, numberDrops/numberPackets,curListSize, bytesQueued,d_exp_,sampleAvgQ, avgArrivalRate, avgServiceRate, sampleAvgLatency,getAvgAvgQLatency());
    fclose(fp);
  }
#endif
  return rc;
}

/*************************************************************
* Routine: int CoDelAQM:doDropOut(Packet *pktPtr)
*
* Function:
*   Internal routine called when the channel(s) ready for next packet in the queue
*   This routine maintains the state of the CoDeL
*
* Inputs: 
*
* Outputs:
*   returns rc of SUCCESS or FAILURE. 
*   The routine indirectly updates CoDeL state including:
*        dropping_ :  might switch states
*
* Pseudo code:
*
*   Variables/state:
*    ------------------------
*   Set on entry
*     curTime : set to current simulation time
*
*   State:
*     dropping_  : if set, we are in the dropping state
*     drop_next_ : time to drop the next packet
*     first_above_time_ : remembers the time when we first see d_exp_ cross the threshold (exceed the tolerated queue delay)
*     count_ : the number of packets dropped since entering the drop state
*     d_exp_ : the delay experienced (was off pkt at the head)
*
*   Learned dynamically
*     maxpacket_ 
*
*   Globally defined:
*     target_ : the highest sustained queue delay tolerated
*     interval_ : how frequently the algorithm reacts
*
*   Logic
*    ------------------------
*
*   
***************************************************************/
int  CoDelAQM::doDropOut(Packet *pktPtr)
{
  int rc = SUCCESS;
  double curTime = Scheduler::instance().clock();
  struct hdr_cmn *ch = HDR_CMN(pktPtr);
  double sampleAvgQ = 0.0;
  double sampleAvgLatency = 0.0;

  //Update sojourn time 
  //more genericallly this can be 'update the estimated average packet queue delay'
  d_exp_ = curTime - ch->ts_;

#ifdef TRACEME
  if (traceFlag == 1) {
    printf("CoDelAQM::doDropOut:(%lf):ENTRY (listID:%d) dropping_:%d d_exp_:%6.6f, target_:%6.6f, first_above_time_:%6.6f, drop_now_:%d\n", curTime, listID,dropping_,d_exp_,target_,first_above_time_,drop_now_);
  }
#endif

  if (dropping_) {
    if (d_exp_ < target_ || bytesQueued <= maxpacket_) {
      // went below - stay below for at least interval
      switchToNotDropping();
    } 
  }
  else {
    if (d_exp_ >= target_) {
      if (first_above_time_ == 0) {
         first_above_time_ = curTime + interval_;
      } else if (curTime >= first_above_time_) {
        switchToDropping();
        drop_now_ = 1;
      }
    }
    else {
      first_above_time_ = 0;
    }

  }
    
#ifdef TRACEME
  if (traceFlag == 1) {
    printf("CoDelAQM::doDropOut:(%lf):EXIT (listID:%d) dropping_:%d d_exp_:%6.6f, target_:%6.6f, first_above_time_:%6.6f, drop_now_:%d\n", curTime, listID,dropping_,d_exp_,target_,first_above_time_,drop_now_);
  }
#endif

  sampleAvgQ = updateAvgQ();
  sampleAvgLatency =  updateAvgQLatency();
#ifdef  FILES2_OK
  if (traceFlag == 1) {
    FILE* fp = NULL;
    char traceString[32];
    char *tptr = traceString;
    sprintf(tptr,"traceAQM%d.out",listID);
    fp = fopen(tptr, "a+");
    fprintf(fp,"3 %6.6f\t%d\t%d\t%6.6f\t%6.6f\t%d\t%3.3f\t%d\t%d\t%6.6f\t%3.3f\t%12.0f\t\t%12.0f\t%3.6f\t%3.6f\n",
     Scheduler::instance().clock(),listID,dropping_,drop_next_,first_above_time_,count_, numberDrops/numberPackets,curListSize, bytesQueued,d_exp_,sampleAvgQ, avgArrivalRate, avgServiceRate, sampleAvgLatency,getAvgAvgQLatency());
    fclose(fp);
  }
#endif

  return rc;
}

/*************************************************************
* Routine: void CoDelAQM::switchToNOTDropping()
*
* Function:
*   Switches state from NOT DROPPING to DROPPING
*
* Inputs: 
*
*************************************************************/
void CoDelAQM::switchToNotDropping()
{
  double curTime = Scheduler::instance().clock();

  dropping_ = 0;
  count_ = 0;
  first_above_time_ = 0;
  drop_next_ = 0.0;
}

/*************************************************************
* Routine: void CoDelAQM::switchToDropping()
*
* Function:
*   Switches state from NOT DROPPING to DROPPING
*
* Inputs: 
*
*************************************************************/
void CoDelAQM::switchToDropping()
{
  double curTime = Scheduler::instance().clock();

  dropping_ = 1;
  count_=0;
  drop_next_ = control_law(curTime);
}

/*************************************************************
* routine: double CoDelAQM::control_law(double t)
*
* Function:
*    return the time of the next drop relative to 't'
*
* inputs: 
*    t:  reference time
*
* outputs:
*    returns the time for the next drop.
***************************************************************/
double CoDelAQM::control_law(double t)
{
  
  if (count_ > 0)
    return (t + interval_  / sqrt(count_));
  else
    return t + interval_;
}



