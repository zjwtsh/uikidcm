#ifdef __cplusplus
extern "C" {
#endif

#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"

#ifdef __cplusplus
}
#endif

#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <vector>

#include "lua_accumulate_ball.h"
#include "RadonTransform.h"

#define POW(x)  x*x
#define VISION_ANGLE_WIDTH 60/180*M_PI
#define FOCUS_LENGTH 480/2/tan(VISION_ANGLE_WIDTH/2)
#define CAMERA_TILT 80/180*M_PI

#define ORIGINAL_BALL_RADIUS 140

//static std::vector<Array2D> fourConn(4) = {{-1, 0}, {0, 1}, {1, 0}, {0, -1}};

std::vector<Cluster>  infoOfCluster;
std::vector<std::vector<uint8_t> > relationMap;

int Max(int a, int b) {return a<b?b:a;};
int Min(int a, int b) {return a<b?a:b;};

int lua_accumulate_ball(std::vector <Candidate> &ballCandidates, uint8_t *label, int width, int height)
{
  static std::vector<Array2D> fourConn;//{{-1, 0}, {0, 1}, {1, 0}, {0, -1}};
  fourConn.resize(4);
  fourConn.push_back({-1, 0});
  fourConn.push_back({0, 1});
  fourConn.push_back({1, 0});
  fourConn.push_back({0, -1});

  int m = width;
  int n = height;

  std::vector<uint8_t> pointProcFlag(m*n, 0); // need fix
  std::vector<Array2D> growQueue;
  std::vector<Array2D> corners(2);

  Array2D currPt, newPt, clusterCenter;
  Cluster singleCluster, currCluster, tryingCluster;

  float lineAngle, maxRadius, maxNoisy, currRadius;

  for (int i = 0; i < m; i ++)
  {
    for (int j = 0; j < n; j++)
    {
      if (label[i*m + j] != 0 && pointProcFlag[i*m + j] == 0)
      {
        int nStart = 0;
        int nEnd = 1;
        int tag = label[i*m + j];
        pointProcFlag[i*m + j] = 1;
        growQueue.push_back({i, j});
        corners[0] = {i, j};
        corners[1] = {i, j};
        while (nStart < nEnd)
        {
          currPt = growQueue[nStart];
          for (int k = 0; k < 4; k++)
          {
            newPt.x = currPt.x + fourConn[i].x;
            newPt.y = currPt.y + fourConn[i].y;
            if (newPt.x <= m && newPt.x >= 0 
                && newPt.y <= n && newPt.y >=0)
            {
              if (pointProcFlag[newPt.x*m + newPt.y] == 0
                  && label[newPt.x*m + newPt.y] == tag)
              {
                growQueue.push_back(newPt);
                nEnd++;
                pointProcFlag[newPt.x*m + newPt.y] = 1;
              }
              else
              {
                switch (k)
                {
                  case 1:
                    if (currPt.x < corners[0].x)
                      corners[0].x = currPt.x;
                    break;
                  case 2:
                    if (currPt.y > corners[1].y)
                      corners[1].y = currPt.y;
                    break;
                  case 3:
                    if (currPt.x > corners[1].x)
                      corners[1].x = currPt.x;
                    break;
                  case 4:
                    if (currPt.y < corners[0].y)
                      corners[0].y = currPt.y;
                    break;
                  default:
                    break;
                }
              }
            }
            else
            {
              switch (k)
              {
                case 1:
                  if (currPt.x < corners[0].x)
                    corners[0].x = currPt.x;
                  break;
                case 2:
                  if (currPt.y > corners[1].y)
                    corners[1].y = currPt.y;
                  break;
                case 3:
                  if (currPt.x > corners[1].x)
                    corners[1].x = currPt.x;
                  break;
                case 4:
                  if (currPt.y < corners[0].y)
                    corners[0].y = currPt.y;
                  break;
                default:
                  break;
              }
            }
          } // for fourConner end
          nStart++;
        } // while end
        clusterCenter.y = (corners[0].y + corners[1].y)/2;
        lineAngle = atan((-clusterCenter.y - n/2)/FOCUS_LENGTH) + CAMERA_TILT; //need fix
        if (lineAngle < 5/180*M_PI)
          continue;
        
        maxRadius = ORIGINAL_BALL_RADIUS * sin(lineAngle);  // need fix
        maxNoisy = 0.3 * maxRadius; 
        currRadius = Max(abs(corners[0].x - corners[1].x), abs(corners[0].y - corners[1].y));

        if (nEnd > maxNoisy && currRadius < 1.2 * maxRadius)
        {
          singleCluster.colorTag = tag;
          singleCluster.colorCount = nEnd;
          singleCluster.bBox.push_back(corners[0]);
          singleCluster.bBox.push_back(corners[1]);
          infoOfCluster.push_back(singleCluster);
        }
      } // if label != 0 end
    } // for j = height end
  } // for i = width end

  relationMap.resize(infoOfCluster.size());
  std::vector<Array2D> enlargedBBox(2);

  for (int i = 0; i < infoOfCluster.size(); i++)
  {
    relationMap[i].resize(infoOfCluster.size());
    currCluster = infoOfCluster[i];
    for (int j = i+1; j < infoOfCluster.size(); j++)
    {
      relationMap[i][j] = 0;
      relationMap[j][i] = 0;

      tryingCluster = infoOfCluster[j];

      enlargedBBox[0].x = Min(currCluster.bBox[0].x, tryingCluster.bBox[0].x);
      enlargedBBox[0].y = Min(currCluster.bBox[0].y, tryingCluster.bBox[0].y);
      enlargedBBox[1].x = Max(currCluster.bBox[0].x, tryingCluster.bBox[0].x);
      enlargedBBox[1].y = Max(currCluster.bBox[0].y, tryingCluster.bBox[0].y);

      clusterCenter.y = (enlargedBBox[0].y + enlargedBBox[1].y)/2;
      lineAngle = atan(clusterCenter.y - n/2)/FOCUS_LENGTH + CAMERA_TILT;    // need fix

      if (lineAngle < 5/180*M_PI)
      {
        relationMap[i][j] = 2;
        relationMap[j][i] = 2;
        continue;
      }

      maxRadius = ORIGINAL_BALL_RADIUS * sin(lineAngle);  // need fix
      currRadius = Max(abs(enlargedBBox[0].x - enlargedBBox[1].x), abs(enlargedBBox[0].y - enlargedBBox[1].y));

      if (currRadius > 1.2*maxRadius)
      {
        relationMap[i][j] = 2;
        relationMap[j][i] = 2;
        continue;
      }

      int totalLengthX = currCluster.bBox[1].x - currCluster.bBox[0].x + tryingCluster.bBox[1].x - tryingCluster.bBox[0].x;
      int totalLengthY = currCluster.bBox[1].y - currCluster.bBox[0].y + tryingCluster.bBox[1].y - tryingCluster.bBox[0].y;
      if ((totalLengthX + 2) > (enlargedBBox[1].x - enlargedBBox[0].x) && 
          (totalLengthY + 2) > (enlargedBBox[1].y - enlargedBBox[0].y))
      {
        relationMap[i][j] = 1;
        relationMap[j][i] = 1;
      }
    }
  }

  std::vector<uint8_t> candidateRegionCenter(infoOfCluster.size(), 0);
  std::vector<uint8_t> connectRegion, nextConnectRegion, targetRegion;

  for (int i = 0; i < infoOfCluster.size(); i++)
  {
    if (candidateRegionCenter[i] = 3)
      continue;

    bool isRunning = true;

    connectRegion.assign(relationMap[i].begin(), relationMap[i].end());
    connectRegion[i] = 3;
    
    nextConnectRegion.assign(connectRegion.begin(), connectRegion.end());
    
    while (isRunning)
    {
      isRunning = false;
      for (int j = 0; j < infoOfCluster.size(); j++)
      {
        if (connectRegion[j] == 1)
        {
          targetRegion.assign(relationMap[j].begin(), relationMap[j].end());
          for (int k = 0; k < infoOfCluster.size(); k++)
          {
            if (targetRegion[k] != 0)
            {
              if (targetRegion[k] == 2)
              {
                connectRegion[k] = targetRegion[k];
                nextConnectRegion[k] = targetRegion[k];
              }
              else if (connectRegion[k] == 0)
              {
                nextConnectRegion[k] = targetRegion[k];
                isRunning = true;
              }
            }
          }
          connectRegion[j] = 3;
          nextConnectRegion[j] = 3;
        }
      }
      connectRegion.assign(nextConnectRegion.begin(), nextConnectRegion.end());
    }// while end
    
    std::vector<Cluster>  connectResult;
    for (int j = 0; j < infoOfCluster.size(); j++)
    {
      if (connectRegion[j] != 0)
      {
        if (connectRegion[j] == 3)
        {
          connectResult.push_back(infoOfCluster[j]);
          candidateRegionCenter[j] = connectRegion[j];
        }
        else if (candidateRegionCenter[j] != 3)
        {
          candidateRegionCenter[j] = connectRegion[j];
        }
      }
    }
    
    int bkCntr = 0;
    int wtCntr = 0;
    int blCntr = 0;
    std::vector<Array2D> ballBBox(2);
    
    for (int i = 0; i < connectResult.size(); i++)
    {
      ballBBox[0].x = Min(connectResult[i].bBox[0].x, ballBBox[0].x);
      ballBBox[0].y = Min(connectResult[i].bBox[0].y, ballBBox[0].y);
      ballBBox[1].x = Min(connectResult[i].bBox[1].x, ballBBox[1].x);
      ballBBox[1].x = Min(connectResult[i].bBox[1].y, ballBBox[1].y);
      switch (connectResult[i].colorTag)
      {
        case 1:
          blCntr += connectResult[i].colorCount;
      	  break;
        case 2:
          bkCntr += connectResult[i].colorCount;
      	  break;
        case 4:
          wtCntr += connectResult[i].colorCount;
      	  break;
      }
    }
    int totalCntr = bkCntr + wtCntr + blCntr;
    float rate = blCntr/totalCntr;
    if (rate < 0.9 && rate > 0.1)
    {
      rate = wtCntr / totalCntr;
      if (rate < 0.9 || rate > 0.1)
     	  continue;
    }
    else
      continue;

    clusterCenter.y = (ballBBox[0].y + ballBBox[1].y)/2;
    lineAngle = atan(-clusterCenter.y - n/2)/FOCUS_LENGTH + CAMERA_TILT; //need fix
    maxRadius = ORIGINAL_BALL_RADIUS * sin(lineAngle);  // need fix
    maxNoisy = 0.3 * maxRadius; 
    currRadius = Max(abs(ballBBox[0].x - ballBBox[1].x), abs(ballBBox[0].y - ballBBox[1].y));
    
    if (currRadius < 0.3 * maxRadius)
      continue;

    float Kcolor = 0.33;
    float Kground = 2.0;  // need fix
    float Kradius = 0.33;

    float evaluation = Kcolor * (POW(float(blCntr)/totalCntr - 0.4) + POW(float(wtCntr)/totalCntr - 0.4) + POW(float(bkCntr)/totalCntr - 0.2)) + Kradius * POW(float(currRadius)/maxRadius - 1);  //  need fix

    ballCandidates.push_back({ballBBox, blCntr, wtCntr, bkCntr, evaluation});
  }
  return ballCandidates.size();

}
