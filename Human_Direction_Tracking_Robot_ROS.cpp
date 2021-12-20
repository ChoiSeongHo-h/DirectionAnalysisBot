#include <ros/ros.h>
#include <ros/master.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <kanu_msgs/BoundingBoxIXYWH.h>
#include <kanu_msgs/BoundingBoxIXYWHArray.h>
#include <kanu_msgs/TargetData.h>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <numeric>

typedef uint16_t idx_t;
typedef uint16_t px_t;
typedef uint16_t dist_t; //mm
typedef float ratio_t;
typedef float rad_t;
typedef int32_t mm_t;
typedef uint32_t len_t;
typedef uint8_t target_t;
typedef double second_t;

//sudo sh -c 'echo 255 > /sys/devices/pwm-fan/target_pwm'

const px_t STANDARD_W = 848;
const px_t STANDARD_H = 480;
const ratio_t DETECT_BOX_RATIO_W = 0.15;
const ratio_t DETECT_BOX_RATIO_H = 0.1;
const ratio_t MAX_DELTA_H_RATIO = 0.85;
const ratio_t DETECT_BOX_RATIO = 10;
const ratio_t TOP_LINE_ELEMENT_RATIO = 0.35;
const std::string THIS_NODE_NAME("image_listener1");
const std::string DEPTH_NODE_NAME("/camera/aligned_depth_to_color/image_raw");
const std::string ML_NODE_NAME("/detect_results");
const uint8_t DEPTH_MSG_BUFFER_SZ = 1;
const uint8_t DETECT_MSG_BUFFER_SZ = 1;
const double ANGLE_PX_RATIO = 0.00162094565;
const second_t MAX_POS_TIME = 2;
const second_t NEAR_COMMUNICATION_TIME = 5;
const second_t OUT_COMMUNICATION_TIME = 5;
const second_t IN_COMMUNICATION_TIME = 5;
const second_t COOLTIME = 3;
const second_t MAX_TARGEING_TIME = 5;
const uint8_t MIN_POS_NUM_4_REGRESSION = 10;
const uint8_t MAX_POS_NUM_4_REGRESSION = 13;
const idx_t NO_IDX = -1;
const uint8_t MM_ADJUSTING_CONST = 16;
const dist_t MIN_STDV = 300;
const float MIN_R2 = 0.7;
ratio_t VERTICAL_STANDARD = pow(0.2, 2);
ratio_t HORIZONTAL_STANDARD = pow(0.2, 2);
const ratio_t LPF_COEFFICENT = 0.5;
const uint8_t MIN_NEAR_COUNT = 30;
const uint8_t MAX_NEAR_COUNT = 50;
const target_t NEAR = 0;
const target_t GO_OUT = 1;
const target_t GO_IN = 2;
const ratio_t MM_TO_M_RATIO = 0.001;

struct BotPos
{
  rad_t angle = -0.2;
  mm_t x = 1*1000;
  mm_t y = 3*1000;
};
BotPos botPos;

struct DestData
{
  mm_t x0 = 6*1000;
  mm_t y0 = -5*1000;
  mm_t x1 = -3*1000;
  mm_t y1 = 6*1000;
};
DestData destData;

const double CAM_FOV_W_HALF = 0.602138592;

const len_t MAP_H_MM = 13*1000;
const len_t MAP_W_MM = 18*1000;
const ratio_t TOPVIEW_RATIO = 0.03;
px_t TOPVIEW_H = MAP_H_MM*TOPVIEW_RATIO;
px_t TOPVIEW_W = MAP_W_MM*TOPVIEW_RATIO;
mm_t MAX_SCAN_RANGE = 6*1000;
mm_t NEAR_RANGE = 2*1000;
uint8_t DETECT_POS_VEC_CAP = MAX_POS_TIME*10;
//uint32_t MAP_ORIGIN_X_rat = 
//y
//angle north
//x on map mm
//y on map mm
//--temp--
const uint32_t mapOriginXmm = 6*1000;
const uint32_t mapOriginYmm = 7*1000;
px_t originXPx = mapOriginXmm*TOPVIEW_RATIO;
px_t originYPx = mapOriginYmm*TOPVIEW_RATIO;

//------process
px_t botXPx = originXPx+botPos.x*TOPVIEW_RATIO;
px_t botYPx = originYPx-botPos.y*TOPVIEW_RATIO;

px_t DestX0Px = originXPx+destData.x0*TOPVIEW_RATIO;
px_t DestY0Px = originYPx-destData.y0*TOPVIEW_RATIO;
px_t DestX1Px = originXPx+destData.x1*TOPVIEW_RATIO;
px_t DestY1Px = originYPx-destData.y1*TOPVIEW_RATIO;


//---------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------



class DepthProcessor
{
private:
  struct DetectPos
  {
    ros::Time initTime;
    mm_t x;
    mm_t y;
    DetectPos(const mm_t x, const mm_t y)
    {
      this->initTime = ros::Time::now();
      this->x = x;
      this->y = y;
    }
  };
  struct DetectPosVecTableElement
  {
    idx_t id;
    std::vector<DepthProcessor::DetectPos> detectPosVec;
    int16_t nearCount;
    bool isNearRecently;
		mm_t recentDist;
    
    DetectPosVecTableElement(const idx_t id, std::vector<DepthProcessor::DetectPos> detectPosVec)
    {
      this->id = id;
      this->detectPosVec = detectPosVec;
      this->nearCount = 0;
      this->isNearRecently = false;
      this->recentDist = 0;
    }
  };
  struct TargetData
  {
    ros::Time targetingTime;
    bool isTargeting = false;
    bool isVertical = false;
    bool isHorizontal = false;
    idx_t idx = 0;
    mm_t alpha = 0;
    float beta = 0;
    mm_t meanX = 0;
    mm_t meanY = 0;
    mm_t intersectionX = 0;
    mm_t intersectionY = 0;
    target_t type = NEAR;
    bool isNear = false;
    rad_t angle = 0;
    void SetTarget(void);
    void ResetTarget(void);
  };
  struct StatisticsData
  {
    mm_t meanX;
    mm_t meanY;
    float sxx;
    float syy;
    float sxy;
  };
  struct CommunicationTimer
  {
    ros::Time endTime;
    ros::Time communicationTime;
    void SetCooltime(void);
    bool IsCooltime(void);
    void Start(void);
    bool IsOver(const target_t type);
  };
  
  DepthProcessor::TargetData targetData;
  std::vector<DepthProcessor::DetectPosVecTableElement> detectPosVecTable;
  cv::Mat depthImg;
  cv::Mat topView;
  ros::Publisher targetDataPub;
  std::vector<idx_t> sortedByDistIndices;
  DepthProcessor::CommunicationTimer communicationTimer;

  std::vector<DepthProcessor::DetectPos>& NewDetectPosVec(const idx_t id);
  void UpdateDetectPos(void);
  idx_t GetDetectPosVecTableElementIdx(const idx_t id);
  void PushDetectPos(const kanu_msgs::BoundingBoxIXYWHArray::ConstPtr& detectResultMsg);
  dist_t CalcPoseDist(DepthProcessor::DetectPos& detectPos, const mm_t x, const mm_t y);
  void FindPedestrain(void);
  void CallbackDetect(const kanu_msgs::BoundingBoxIXYWHArray::ConstPtr& detectResultMsg);
  void DrawTopView(void);
  void InitTopView(void);
  void CallbackDepth(const sensor_msgs::ImageConstPtr& imgDataMsg);
  rad_t GetAngle(const px_t x);
  dist_t GetDist(const cv::Rect& detectResult);
  cv::Scalar Get8UColor(const idx_t id);
  void LowPassFilt(std::vector<DepthProcessor::DetectPos>& detectPosVecPtr);
  bool IsIntersect(const DepthProcessor::TargetData& tmpTargetData);
  void SetStatisticsData(std::vector<DepthProcessor::DetectPos>& detectPosVec, DepthProcessor::StatisticsData& statisticsData);
  void GetDirection(void);
  void GetIntersectionPoint(void);
  void CheckNear(void);
  void SendTargetData(void);
	void SetSortedByDistIndices(void);
  void ReleaseNotNearTarget(void);
  void ClearNearCount(void);

public:
  DepthProcessor(int argc, char **argv);
  ~DepthProcessor(void);
};


//---------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------


void DepthProcessor::CommunicationTimer::Start(void)
{
  printf("start communication\n");
  this->communicationTime = ros::Time::now();
}

bool DepthProcessor::CommunicationTimer::IsOver(const target_t type)
{
  if(type == NEAR)
    return (ros::Time::now()-this->communicationTime).toSec() > NEAR_COMMUNICATION_TIME;
  else if(type == GO_OUT)
    return (ros::Time::now()-this->communicationTime).toSec() > OUT_COMMUNICATION_TIME;
  else if(type == GO_IN)
    return (ros::Time::now()-this->communicationTime).toSec() > IN_COMMUNICATION_TIME;
}

void DepthProcessor::ClearNearCount(void)
{
  idx_t detectPosVecTableSz = this->detectPosVecTable.size();
  for(idx_t i = 0; i<detectPosVecTableSz; i++)
    this->detectPosVecTable[i].nearCount = 0;
}

void DepthProcessor::CommunicationTimer::SetCooltime(void)
{
  this->endTime = ros::Time::now();
}

bool DepthProcessor::CommunicationTimer::IsCooltime(void)
{
  return (ros::Time::now()-this->endTime).toSec() < COOLTIME;
}

void DepthProcessor::ReleaseNotNearTarget(void)
{
  if(this->targetData.isTargeting == false)
    return;

  if(this->targetData.isNear == false && (ros::Time::now()-this->targetData.targetingTime).toSec() > MAX_TARGEING_TIME)
  {
    this->targetData.ResetTarget();
    this->communicationTimer.SetCooltime();
    this->ClearNearCount();
  }  
  else if(this->targetData.isNear)
    this->targetData.SetTarget();
}

void DepthProcessor::TargetData::SetTarget(void)
{
  this->targetingTime = ros::Time::now();
  this->isTargeting = true;
}

void DepthProcessor::TargetData::ResetTarget(void)
{
  this->isTargeting = false;
}

void DepthProcessor::SetSortedByDistIndices(void)
{
	std::vector<dist_t> detectPoseDistVec;
  this->sortedByDistIndices.clear();
  idx_t detectPosVecTableSz = this->detectPosVecTable.size();
  for(idx_t i = 0; i<detectPosVecTableSz; i++)
  {
    detectPoseDistVec.emplace_back(this->detectPosVecTable[i].recentDist);
    this->sortedByDistIndices.emplace_back(i);
	}
  std::sort(this->sortedByDistIndices.begin(), this->sortedByDistIndices.end(),
		[&](const idx_t i, const idx_t j){return detectPoseDistVec[i]<detectPoseDistVec[j];});
}

void DepthProcessor::GetIntersectionPoint(void)
{
  //target's
  float m2 = this->targetData.beta;
  float b2 = float(this->targetData.alpha);
  
  if(destData.x1-destData.x0 == 0)  //dest is vertical
  {
    this->targetData.intersectionX = destData.x0;
    this->targetData.intersectionY = m2*float(destData.x0)+b2;
    return;
  }

  //dest's
  float m1 =  float(destData.y1-destData.y0)/float(destData.x1-destData.x0);
  float b1 = -float(destData.x0*(destData.y1-destData.y0))/float(destData.x1-destData.x0)+float(destData.y0);

  if(this->targetData.isHorizontal)
  {
    this->targetData.intersectionX = (b2-b1)/(m1-m2);
    this->targetData.intersectionY = this->targetData.meanY;
    return;
  }

  if(this->targetData.isVertical)
  {
    this->targetData.intersectionX = this->targetData.meanX;
    this->targetData.intersectionY = m1*float(this->targetData.meanX)+b1;
    return;
  }

  this->targetData.intersectionX = (b2-b1)/(m1-m2);
  this->targetData.intersectionY = m1*this->targetData.intersectionX+b1;
}

void DepthProcessor::GetDirection(void)
{
  if(this->targetData.isTargeting == false)
    return;

  mm_t directionSample; 
  float signedSlopeDx;
  float signedSlopeDy;

  if(this->targetData.isVertical)
  {
    directionSample = this->targetData.meanY - this->targetData.intersectionY;
    signedSlopeDx = 0;
    signedSlopeDy = directionSample>0? 1 : -1;
  }
  else if(this->targetData.isHorizontal)
  {
    directionSample = this->targetData.meanX - this->targetData.intersectionX;
    signedSlopeDx = directionSample>0? 1 : -1;
    signedSlopeDy = 0;
  }
  else  //common
  {
    directionSample = this->targetData.meanY - this->targetData.intersectionY; 
    int8_t outDirectionSign = directionSample>0? 1 : -1;
    signedSlopeDx = 1;
    signedSlopeDy = outDirectionSign*abs(this->targetData.beta);
    float signedSlopeLen = sqrt(pow(signedSlopeDx, 2)+pow(signedSlopeDy, 2));
    signedSlopeDx /= signedSlopeLen;
    signedSlopeDy /= signedSlopeLen;
  }

  //path
  std::vector<DepthProcessor::DetectPos>& detectPosVec = this->detectPosVecTable[this->targetData.idx].detectPosVec;
  idx_t detectPosVecSz = detectPosVec.size();

  float dx = 0;
  float dy = 0;
  for(idx_t i = 0; i<detectPosVecSz-1; i++)
  {
    float partialDx = (float)(-detectPosVec[i].x+detectPosVec[i+1].x);
    float partialDy = float(-detectPosVec[i].y+detectPosVec[i+1].y);
    float partialLen = sqrt(pow(partialDx, 2)+pow(partialDy, 2));
    if(partialLen == 0)
      continue;
    dx += partialDx/partialLen;
    dy += partialDy/partialLen;
  }
  float len = sqrt(pow(dx, 2)+pow(dy, 2));
  dx /= len;
  dy /= len;

  float dot = signedSlopeDx*dx+signedSlopeDy*dy;

  if(dot>0)
    this->targetData.type = GO_OUT;
  else
    this->targetData.type = GO_IN;
   
}

bool DepthProcessor::IsIntersect(const DepthProcessor::TargetData& tmpTargetData)
{
  mm_t destX0, destX1, destY0, destY1;
  bool isDestVertical = false;
  bool isDestHorizontal = false;

  //destX0(small) destX1(large)
  if(destData.x0<destData.x1)
  {
    destX0 = destData.x0;
    destY0 = destData.y0;
    destX1 = destData.x1;
    destY1 = destData.y1;
  }
  else if(destData.x0>destData.x1)
  {
    destX0 = destData.x1;
    destX1 = destData.x0;
    destY0 = destData.y1;
    destY1 = destData.y0;
  }
  else  //destData.x0 == destData.x1
  {
    isDestVertical = true;
    //destY0(large)
    //destY1(small)
    if(destData.y0>destData.y1)
    {  
      destX0 = destData.x0;
      destY0 = destData.y0;
      destY1 = destData.y1;
    }
    else if(destData.y0<destData.y1)
    {
      destX0 = destData.x0;
      destY0 = destData.y1;
      destY1 = destData.y0;
    }
    else  //same point
      return false;
  }

  if(destData.y0 == destData.y1)
    isDestHorizontal = true;
  
  mm_t testY0, testY1;

  //Vertical dest case
  if(isDestVertical && tmpTargetData.isHorizontal)
  {
    if(destY1<tmpTargetData.meanY && tmpTargetData.meanY<destY0)
      return true;
    else
      return false;
  }
  if(isDestVertical && tmpTargetData.isVertical)
    return false;
  if(isDestVertical && !tmpTargetData.isHorizontal && !tmpTargetData.isVertical)
  {
    testY0 = (float)tmpTargetData.alpha+tmpTargetData.beta*(float)destX0;
    if(destY1<testY0 && testY0<destY0)
      return true;
    else
      return false;
  }

  //Horizontal dest & special case
  if(isDestHorizontal && tmpTargetData.isHorizontal)
    return false;

  //common dest case
  if(tmpTargetData.isHorizontal)
  {
    if((destY0<tmpTargetData.meanY)^(tmpTargetData.meanY<destY1) == 0)  //if A xnor B
      return true;
    else
      return false;
  }
  if(tmpTargetData.isVertical)
  {
    if(destX0<tmpTargetData.meanX && tmpTargetData.meanX<destX1)
      return true;
    else
      return false;
  }
  if(!tmpTargetData.isHorizontal && !tmpTargetData.isVertical)  //common dest, common target
  {
    testY0 = (float)tmpTargetData.alpha+tmpTargetData.beta*(float)destX0;
    testY1 = (float)tmpTargetData.alpha+tmpTargetData.beta*(float)destX1;
    if((destY0<testY0)^(destY1>testY1) == 0)  //if A xnor B
      return true;
    else
      return false;
  }
}

void DepthProcessor::LowPassFilt(std::vector<DepthProcessor::DetectPos>& detectPosVec)
{
  idx_t detectPosVecSz = detectPosVec.size();
  if (detectPosVecSz <= 1)
    return;

  detectPosVec[detectPosVecSz-1].x = LPF_COEFFICENT*detectPosVec[detectPosVecSz-2].x+
    (1-LPF_COEFFICENT)*detectPosVec[detectPosVecSz-1].x;
  detectPosVec[detectPosVecSz-1].y = LPF_COEFFICENT*detectPosVec[detectPosVecSz-2].y+
    (1-LPF_COEFFICENT)*detectPosVec[detectPosVecSz-1].y;
}

DepthProcessor::DepthProcessor(int argc, char **argv)
{
  this->depthImg = cv::Mat(STANDARD_H, STANDARD_W, CV_16UC1);
  this->InitTopView();
  ros::init(argc, argv, THIS_NODE_NAME);
  ros::NodeHandle nh;
  this->communicationTimer.SetCooltime();
  
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber depthSub = it.subscribe(DEPTH_NODE_NAME, DEPTH_MSG_BUFFER_SZ, &DepthProcessor::CallbackDepth, this);
  
  ros::Subscriber detectSub = nh.subscribe(ML_NODE_NAME, DETECT_MSG_BUFFER_SZ, &DepthProcessor::CallbackDetect, this);
  this->targetDataPub = nh.advertise<kanu_msgs::TargetData>("targetData", 1);

  ros::spin();
}

DepthProcessor::~DepthProcessor(void)
{
  cv::destroyWindow("view");
}

std::vector<DepthProcessor::DetectPos>& DepthProcessor::NewDetectPosVec(const idx_t id)
{
  std::vector<DepthProcessor::DetectPos> detectPosVec;
  detectPosVec.reserve(DETECT_POS_VEC_CAP);
  this->detectPosVecTable.emplace_back(DepthProcessor::DetectPosVecTableElement(id, detectPosVec));
  return this->detectPosVecTable.back().detectPosVec;
}

void DepthProcessor::CheckNear(void)
{
  for(const idx_t& i:this->sortedByDistIndices)
  {
    if(this->detectPosVecTable[i].isNearRecently && this->detectPosVecTable[i].nearCount<MAX_NEAR_COUNT)
      this->detectPosVecTable[i].nearCount++;
    else if(detectPosVecTable[i].nearCount>0)
      this->detectPosVecTable[i].nearCount--;
    this->detectPosVecTable[i].isNearRecently = false;

    bool condition = this->detectPosVecTable[i].nearCount > MIN_NEAR_COUNT;
    //for targeting
    if(this->targetData.isTargeting == false)
    {
      if(!condition)
        continue;
      else
      {
        this->targetData.idx = i;
        this->targetData.type = NEAR;
        this->targetData.isNear = true;
        this->targetData.SetTarget();
        this->communicationTimer.Start();
        continue;
      }
    }
    //isTargeting, i is not target
    else if(this->targetData.idx != i)
    {
      if(!condition)
        continue;
      else if(this->targetData.isNear)
        continue;
      else
      {
        this->targetData.idx = i;
        this->targetData.type = NEAR;
        this->targetData.isNear = true;
        this->targetData.SetTarget();
        this->communicationTimer.Start();
        continue;
      }
    }
    //i is target
    else
    {
      if(condition)
        if(this->targetData.isNear)
          continue;
        else
        {
          this->targetData.isNear = true;
          this->communicationTimer.Start();
          continue;
        }
      else
      {
        this->targetData.isNear = false;
        if(this->targetData.type == NEAR)
        {
          this->targetData.ResetTarget();      
          this->communicationTimer.SetCooltime();      
          this->ClearNearCount();
          continue;
        }
        continue;
      }
    }
  }
}

void DepthProcessor::UpdateDetectPos(void)
{
  ros::Time nowTime = ros::Time::now();
  idx_t detectPosVecTableSz = detectPosVecTable.size();
  for(idx_t i = 0; i<detectPosVecTableSz; i++)
  {
    std::vector<DepthProcessor::DetectPos>& detectPosVec = this->detectPosVecTable[i].detectPosVec;
    if(detectPosVec.empty())
    {
      if(&(this->detectPosVecTable[i]) < &(this->detectPosVecTable[this->targetData.idx]))  //begin - ... - zero size - ... - target
        this->targetData.idx--;
      else if(&(this->detectPosVecTable[i]) == &(this->detectPosVecTable[this->targetData.idx])) //begin - zero size(= target))
        this->targetData.ResetTarget();
      this->detectPosVecTable.erase(this->detectPosVecTable.begin()+i);
      i--;
      detectPosVecTableSz--;
    }
    else if((nowTime-detectPosVec.front().initTime).toSec() > MAX_POS_TIME || detectPosVec.size() > MAX_POS_NUM_4_REGRESSION)
    {
      detectPosVec.erase(detectPosVec.begin());
      i--;
    }
  }
}

idx_t DepthProcessor::GetDetectPosVecTableElementIdx(const idx_t id)
{
  idx_t detectPosVecTableSz = this->detectPosVecTable.size();
  
  for(idx_t i = 0; i<detectPosVecTableSz; i++)
  {
    if(this->detectPosVecTable[i].id == id)
      return i;
  }
  return NO_IDX;
}


void DepthProcessor::PushDetectPos(const kanu_msgs::BoundingBoxIXYWHArray::ConstPtr& detectResultMsg)
{
  auto boxesVec = detectResultMsg->boxes;
  uint8_t boxesSz = boxesVec.size();

  for(uint8_t detectIter = 0; detectIter<boxesSz; detectIter++)
  {
    idx_t id = boxesVec[detectIter].i;
    cv::Rect detectResult(boxesVec[detectIter].x, boxesVec[detectIter].y, boxesVec[detectIter].w, boxesVec[detectIter].h);
    dist_t dist = this->GetDist(detectResult);
    if(dist>MAX_SCAN_RANGE)
      continue;
    rad_t angle = this->GetAngle(detectResult.x+detectResult.width/2);
    mm_t x = botPos.x+dist*cos(angle-botPos.angle);
    mm_t y = botPos.y-dist*sin(angle-botPos.angle);
    DepthProcessor::DetectPos detectPos(x, y);
    
    idx_t detectPosVecTableElementIdx = this->GetDetectPosVecTableElementIdx(id);
    std::vector<DepthProcessor::DetectPos>* detectPosVecPtr;
    if(detectPosVecTableElementIdx == NO_IDX)
    {
      detectPosVecPtr = &(this->NewDetectPosVec(id));
    	detectPosVecTableElementIdx = this->detectPosVecTable.size();
    }
    else
      detectPosVecPtr = &(this->detectPosVecTable[detectPosVecTableElementIdx].detectPosVec);

    (*detectPosVecPtr).emplace_back(detectPos);

    this->LowPassFilt(*detectPosVecPtr);
    
    mm_t filtedDx = (*detectPosVecPtr).back().x-botPos.x;
    mm_t filtedDy = (*detectPosVecPtr).back().y-botPos.y;
		this->detectPosVecTable[detectPosVecTableElementIdx].recentDist = 
			sqrt(pow((float)filtedDx, 2)+pow((float)filtedDy, 2));
    if(this->detectPosVecTable[detectPosVecTableElementIdx].recentDist<NEAR_RANGE)
      this->detectPosVecTable[detectPosVecTableElementIdx].isNearRecently = true;

    if(this->targetData.isTargeting && this->targetData.idx == detectPosVecTableElementIdx)
      this->targetData.angle = -atan2((float)filtedDy, (float)filtedDx)+botPos.angle;
  }    
}

dist_t DepthProcessor::CalcPoseDist(DepthProcessor::DetectPos& detectPos, const mm_t x, const mm_t y)
{
  return (dist_t)sqrt(pow(detectPos.x-x, 2)+pow(detectPos.y-y, 2));
}

void DepthProcessor::SetStatisticsData(std::vector<DepthProcessor::DetectPos>& detectPosVec, DepthProcessor::StatisticsData& statisticsData)
{
  std::vector<mm_t> detectXVec, detectYVec;
  detectXVec.reserve(DETECT_POS_VEC_CAP); 
  detectYVec.reserve(DETECT_POS_VEC_CAP);     
  idx_t detectPosVecSz = detectPosVec.size();
  for(idx_t i = 0; i<detectPosVecSz; i++)
  {
    DepthProcessor::DetectPos& detectPos = detectPosVec[i];
    detectXVec.emplace_back(detectPos.x/MM_ADJUSTING_CONST);
    detectYVec.emplace_back(detectPos.y/MM_ADJUSTING_CONST);
  }
  statisticsData.meanX = std::accumulate(detectXVec.begin(), detectXVec.end(), 0)/detectXVec.size();
  statisticsData.meanY = std::accumulate(detectYVec.begin(), detectYVec.end(), 0)/detectXVec.size();
  std::vector<int64_t> squaredDiffXVec, squaredDiffYVec, crossDiffXYVec;
  squaredDiffXVec.reserve(DETECT_POS_VEC_CAP);
  squaredDiffYVec.reserve(DETECT_POS_VEC_CAP);
  for(idx_t i = 0; i<detectPosVecSz; i++)
  {
    squaredDiffXVec.emplace_back(detectXVec[i]-statisticsData.meanX);
    squaredDiffYVec.emplace_back(detectYVec[i]-statisticsData.meanY);
    crossDiffXYVec.emplace_back(squaredDiffXVec[i]*squaredDiffYVec[i]);
    squaredDiffXVec[i] = pow(squaredDiffXVec[i], 2);
    squaredDiffYVec[i] = pow(squaredDiffYVec[i], 2);
  }
  
  float POW_MM_ADJUSTING_CONST = pow((float)MM_ADJUSTING_CONST, 2);
  statisticsData.meanX *= MM_ADJUSTING_CONST;
  statisticsData.meanY *= MM_ADJUSTING_CONST;
  statisticsData.sxx = std::accumulate(squaredDiffXVec.begin(), squaredDiffXVec.end(), 0)/detectPosVecSz*POW_MM_ADJUSTING_CONST;
  statisticsData.syy = std::accumulate(squaredDiffYVec.begin(), squaredDiffYVec.end(), 0)/detectPosVecSz*POW_MM_ADJUSTING_CONST;
  statisticsData.sxy = std::accumulate(crossDiffXYVec.begin(), crossDiffXYVec.end(), 0)/detectPosVecSz*POW_MM_ADJUSTING_CONST; 
}

void DepthProcessor::FindPedestrain(void)
{
  if(this->detectPosVecTable.empty())
    return;

  //sort by dist -> num -> stdv -> r2 -> linear regression
  //table iteration
  for(const idx_t& i:this->sortedByDistIndices)
  {
  	//num
  	if(detectPosVecTable[i].detectPosVec.size()<MIN_POS_NUM_4_REGRESSION)
			continue;

    std::vector<DepthProcessor::DetectPos>& detectPosVec = detectPosVecTable[i].detectPosVec;
    DepthProcessor::StatisticsData statisticsData;
    
    this->SetStatisticsData(detectPosVec, statisticsData);

    //stdv
    float stdv = sqrt(statisticsData.sxx+statisticsData.syy);
    if(stdv<MIN_STDV)
      continue;

    //r2 & linear regression
    float r2 = pow(statisticsData.sxy,2)/(statisticsData.sxx*statisticsData.syy);
    DepthProcessor::TargetData tmpTargetData;
    bool isTargeting = false;
    if(r2>MIN_R2)
    {
      isTargeting = true;
      tmpTargetData.idx = i;
      tmpTargetData.beta = statisticsData.sxy/statisticsData.sxx;
      tmpTargetData.alpha = float(statisticsData.meanY)-tmpTargetData.beta*float(statisticsData.meanX);
      tmpTargetData.meanX = statisticsData.meanX;
      tmpTargetData.meanY = statisticsData.meanY;
    }
    else if(statisticsData.sxx/statisticsData.syy < VERTICAL_STANDARD)
    {
      isTargeting = true;
      tmpTargetData.isVertical = true;
      tmpTargetData.idx = i;
      tmpTargetData.meanX = statisticsData.meanX;
      tmpTargetData.meanY = statisticsData.meanY;
    }
    else if(statisticsData.syy/statisticsData.sxx < HORIZONTAL_STANDARD)
    {
      isTargeting = true;
      tmpTargetData.isHorizontal = true;
      tmpTargetData.idx = i;
      tmpTargetData.meanX = statisticsData.meanX;
      tmpTargetData.meanY = statisticsData.meanY;
    }
    
    if(isTargeting && this->IsIntersect(tmpTargetData))
    {
      this->targetData = tmpTargetData;
      this->targetData.SetTarget();
      this->GetIntersectionPoint();
      this->GetDirection();
      return;
    }
  }
}

void DepthProcessor::SendTargetData(void)
{
  kanu_msgs::TargetData targetMsg;
  targetMsg.isNear = this->targetData.isNear; 
  targetMsg.type = this->targetData.type;      
  targetMsg.dist = float(this->detectPosVecTable[this->targetData.idx].recentDist)*MM_TO_M_RATIO;
  targetMsg.angle = this->targetData.angle;
  if(ros::ok())
    targetDataPub.publish(targetMsg);
}

void DepthProcessor::CallbackDetect(const kanu_msgs::BoundingBoxIXYWHArray::ConstPtr& detectResultMsg)
{
  if(this->depthImg.empty() || this->depthImg.data == nullptr)
    return;

  this->PushDetectPos(detectResultMsg);
  this->UpdateDetectPos();

  if(this->communicationTimer.IsCooltime() == false)
  {
    printf("no cooltime\n");
    if(this->targetData.isNear && this->communicationTimer.IsOver(this->targetData.type))
    {
      printf("end communication\n");
      this->targetData.isNear = false;     
      this->ClearNearCount();
      this->targetData.ResetTarget();  
      this->communicationTimer.SetCooltime();     
    }
	  this->SetSortedByDistIndices();
    this->CheckNear();  //priority : near > in&out
    if(this->targetData.isTargeting)
    {
      this->SendTargetData();
      this->ReleaseNotNearTarget();
    }
    else
      this->FindPedestrain();
  }
  else
    printf("cooltime\n");

  //visualize
  this->InitTopView();
  this->DrawTopView();
  cv::imshow("view", this->topView);
  cv::waitKey(10);
}

cv::Scalar DepthProcessor::Get8UColor(const idx_t id)
{
  cv::Scalar color;

  if(id%7 == 0)    
    color = cv::Scalar(255,0,0);
  else if(id%7 == 1)    
    color = cv::Scalar(0,255,0);
  else if(id%7 == 2)    
    color = cv::Scalar(0,0,255);
  else if(id%7 == 3)    
    color = cv::Scalar(255,255,0);
  else if(id%7 == 4)    
    color = cv::Scalar(0,255,255);
  else if(id%7 == 5)    
    color = cv::Scalar(255,0,255);
  else    
    color = cv::Scalar(255,255,255);

  return color;
}

void DepthProcessor::DrawTopView(void)
{
  idx_t detectPosVecTableSz = this->detectPosVecTable.size();
  cv::Scalar color;

  for(idx_t i = 0; i<detectPosVecTableSz; i++)
  {
    idx_t id = detectPosVecTable[i].id;
    std::vector<DepthProcessor::DetectPos>& detectPosVec = detectPosVecTable[i].detectPosVec;

    idx_t detectPosVecSz = detectPosVec.size();
    for(idx_t j = 0; j<detectPosVecSz; j++)
    {
      DepthProcessor::DetectPos& detectPos = detectPosVec[j];

      px_t detectPointX = originXPx+TOPVIEW_RATIO*(detectPos.x);
      px_t detectPointY = originYPx-TOPVIEW_RATIO*(detectPos.y);

      color = this->Get8UColor(id);
      cv::circle(this->topView, cv::Point(detectPointX, detectPointY), 3, color);
    }
  }
  
  if(this->targetData.isTargeting == false)
    return;

  color = this->Get8UColor(detectPosVecTable[this->targetData.idx].id);
  std::string txt;

  if(this->targetData.type == NEAR)
  {
    txt = std::string("near");
    cv::putText(this->topView, txt, cv::Point(botXPx, botYPx), 2, 1.2, color);
    return;
  }

  //regression, not near
  if(this->targetData.isVertical)
  {
    cv::line(this->topView, 
      cv::Point(TOPVIEW_RATIO*(mapOriginXmm+this->targetData.meanX), 0), 
      cv::Point(TOPVIEW_RATIO*(mapOriginXmm+this->targetData.meanX), STANDARD_H-1),  
      color);
  }
  else if(this->targetData.isHorizontal)
    cv::line(this->topView, 
      cv::Point(0, TOPVIEW_RATIO*(mapOriginYmm-this->targetData.meanY)), 
      cv::Point(STANDARD_W-1, TOPVIEW_RATIO*(mapOriginYmm-this->targetData.meanY)),  
      color);
  else    //linear
    cv::line(this->topView, 
      cv::Point(0, TOPVIEW_RATIO*(float(mapOriginYmm)+this->targetData.beta*float(mapOriginXmm)-(float)this->targetData.alpha)), 
      cv::Point(STANDARD_W-1, TOPVIEW_RATIO*(float(mapOriginYmm)+this->targetData.beta*float(mapOriginXmm)-(float)this->targetData.alpha)-this->targetData.beta*(STANDARD_W-1)), 
      color);
  
  //intersection point
  cv::circle(this->topView, cv::Point(originXPx+this->targetData.intersectionX*TOPVIEW_RATIO, originYPx-this->targetData.intersectionY*TOPVIEW_RATIO), 5, color);

  //in out  
  if(this->targetData.type == GO_OUT)
    txt = std::string("out");
  else if(this->targetData.type == GO_IN)
    txt = std::string("in");
  cv::putText(this->topView, txt, cv::Point(originXPx+this->targetData.intersectionX*TOPVIEW_RATIO, originYPx-this->targetData.intersectionY*TOPVIEW_RATIO), 2, 1.2, color);
}

void DepthProcessor::InitTopView(void)
{
  this->topView = cv::Mat::zeros(TOPVIEW_H, TOPVIEW_W, CV_8UC3);

  //bot
  cv::circle(this->topView, cv::Point(originXPx, originYPx), 5, cv::Scalar(0,0,255));
  cv::circle(this->topView, cv::Point(botXPx, botYPx), 5, cv::Scalar(255,0,0));

  //max range
  px_t scanPoint0X = botXPx+MAX_SCAN_RANGE*TOPVIEW_RATIO*cos(botPos.angle-CAM_FOV_W_HALF);
  px_t scanPoint0Y = botYPx-MAX_SCAN_RANGE*TOPVIEW_RATIO*sin(botPos.angle-CAM_FOV_W_HALF);
  px_t scanPoint1X = botXPx+MAX_SCAN_RANGE*TOPVIEW_RATIO*cos(botPos.angle+CAM_FOV_W_HALF);
  px_t scanPoint1Y = botYPx-MAX_SCAN_RANGE*TOPVIEW_RATIO*sin(botPos.angle+CAM_FOV_W_HALF);
  cv::line(this->topView, cv::Point(botXPx, botYPx), cv::Point(scanPoint0X, scanPoint0Y), cv::Scalar(255,0,0));
  cv::line(this->topView, cv::Point(botXPx, botYPx), cv::Point(scanPoint1X, scanPoint1Y), cv::Scalar(255,0,0));

  //near
  scanPoint0X = botXPx+NEAR_RANGE*TOPVIEW_RATIO*cos(botPos.angle-CAM_FOV_W_HALF);
  scanPoint0Y = botYPx-NEAR_RANGE*TOPVIEW_RATIO*sin(botPos.angle-CAM_FOV_W_HALF);
  scanPoint1X = botXPx+NEAR_RANGE*TOPVIEW_RATIO*cos(botPos.angle+CAM_FOV_W_HALF);
  scanPoint1Y = botYPx-NEAR_RANGE*TOPVIEW_RATIO*sin(botPos.angle+CAM_FOV_W_HALF);
  cv::line(this->topView, cv::Point(botXPx, botYPx), cv::Point(scanPoint0X, scanPoint0Y), cv::Scalar(255,255,0));
  cv::line(this->topView, cv::Point(botXPx, botYPx), cv::Point(scanPoint1X, scanPoint1Y), cv::Scalar(255,255,0));

  //dest
  cv::line(this->topView, cv::Point(DestX0Px, DestY0Px), cv::Point(DestX1Px, DestY1Px), cv::Scalar(0,255,0));
}

void DepthProcessor::CallbackDepth(const sensor_msgs::ImageConstPtr& imgDataMsg)
{
  this->depthImg.data = (uint8_t*)(&(*imgDataMsg).data[0]);
}

rad_t DepthProcessor::GetAngle(const px_t x)
{
  return atan2(ANGLE_PX_RATIO*((int16_t)x-(int16_t)STANDARD_W/2), 1);
}

dist_t DepthProcessor::GetDist(const cv::Rect& detectResult)
{
  px_t gapW = detectResult.width*DETECT_BOX_RATIO_W;
  px_t gapH = detectResult.height*DETECT_BOX_RATIO_H;
  
  std::vector<dist_t> calCol;
  for(px_t deltaH = 0; deltaH<detectResult.height*MAX_DELTA_H_RATIO; deltaH+=gapH)
  {
    px_t x, y, w;
    x = detectResult.x+gapW;
    y = detectResult.y+gapH+deltaH;
    w = detectResult.width-2*gapW;

    dist_t* depPtr = &(this->depthImg.at<px_t>(y, x));
    std::vector<dist_t> calLine;
    calLine.reserve(w);
    calLine.resize(w);
    std::copy(&depPtr[0], &depPtr[w], calLine.begin());
    std::sort(calLine.begin(), calLine.end());

    calCol.emplace_back(calLine[w*TOP_LINE_ELEMENT_RATIO]);
  }
  std::sort(calCol.begin(), calCol.end());
  idx_t calColSz = calCol.size();
  for(idx_t i = 0; i<calColSz; i++)
  {
    if(calCol[i]>0)
      return calCol[i];    
  }
}



//---------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------


int main(int argc, char **argv)
{
  DepthProcessor depthProcessor(argc, argv);
}
  
