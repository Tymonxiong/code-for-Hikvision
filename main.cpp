#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "OSSocket.h"
#include "JsonParse.h"
#include "CmdParse.h"
#include "AStar.h"
#include <time.h>


#define MAX_SOCKET_BUFFER       (1024 * 1024 * 4)       ///4M



/** @fn     int RecvJuderData(OS_SOCKET hSocket, char *pBuffer)
 *  @brief	��������
 *	@param  -I   - OS_SOCKET hSocket
 *	@param  -I   - char * pBuffer
 *	@return int
 */
int RecvJuderData(OS_SOCKET hSocket, char *pBuffer)          //
{
    int         nRecvLen = 0;
    int         nLen = 0;

    while (1)
    {
        // ����ͷ������
        nLen = OSRecv(hSocket, pBuffer + nRecvLen, MAX_SOCKET_BUFFER);
        if (nLen <= 0)
        {
            printf("recv error\n");
            return nLen;
        }

        nRecvLen += nLen;

        if (nRecvLen >= SOCKET_HEAD_LEN)
        {
            break;
        }
    }

    int         nJsonLen = 0;
    char        szLen[10] = { 0 };

    memcpy(szLen, pBuffer, SOCKET_HEAD_LEN);

    nJsonLen = atoi(szLen);

    while (nRecvLen < (SOCKET_HEAD_LEN + nJsonLen))
    {
        // ˵�����ݻ�û������
        nLen = OSRecv(hSocket, pBuffer + nRecvLen, MAX_SOCKET_BUFFER);
        if (nLen <= 0)
        {
            printf("recv error\n");
            return nLen;
        }

        nRecvLen += nLen;
    }

    return 0;
}

/** @fn     int SendJuderData(OS_SOCKET hSocket, char *pBuffer, int nLen)
 *  @brief	��������
 *	@param  -I   - OS_SOCKET hSocket
 *	@param  -I   - char * pBuffer
 *	@param  -I   - int nLen
 *	@return int
 */
int SendJuderData(OS_SOCKET hSocket, char *pBuffer, int nLen)
{
    int     nSendLen = 0;
    int     nLenTmp = 0;

    while (nSendLen < nLen)
    {
        nLenTmp = OSSend(hSocket, pBuffer + nSendLen, nLen - nSendLen);
        if (nLenTmp < 0)
        {
            return -1;
        }

        nSendLen += nLenTmp;
    }

    return 0;
}



//在此处声明一个全局变量方便二维数组在函数间的传递
#define dimLength  210

int threeDMap[dimLength][dimLength][dimLength];

int goodsDetecMap[dimLength][dimLength];
int mapWidth = 0;
int mapLength = 0;

int serachPathOfZ = 0;                    //该变量决定在不同的高度搜索路径 范围在:mapHLow和buildingMaxHigh之间
int buildingMaxHigh = 0;                  //该值所在高度可以飞行

typedef struct {
    int x;
    int y;
} PathNode; 
PathNode pathFrom = {0,16};
PathNode pathTo = {17,8};

int mapHLow=0;
int mapHHigh=0;
PathNode parkingAapron;
int crashUAVNum = 0;
int totalObtainedGoodValue = 0;                    //该价格不包含无人机价格

//无人机价格表
UAV_PRICE UAVPriceList[MAX_UAV_PRICE_NUM];
int uavPriceNum = 0;

//家和敌人家的位置
GOAL_POINT homeLocation;
GOAL_POINT enemyHomeLocation;

int idleStatusUAVNum=0;                     //空闲状态的无人机数量  
int acceptedStatusUAVNum=0;                 //接受到准备执行任务的无人机数量
int executingStatusUAVNum=0;                //载货正在执行的无人机数量

int deatSquadsUAVNum = 0;


//分别用于存放三种工作状态下的无人机
UAVS  idleUAV[MAX_UAV_NUM];                  //一般只需要对空闲状态下的无人机排序,空闲状态下包括已经匹配到目标和未匹配到目标的
UAVS  acceptedUAV[MAX_UAV_NUM];
UAVS  executingUAV[MAX_UAV_NUM];

UAVS  deatSquadsUAV[MAX_UAV_NUM];                  


int unmatchGoodsNum=0;
int matchedGoodsNum=0;
int pickedGoodsNum=0;
//货物状态
GOODS_STATE unmatchGoods[MAX_GOODS_NUM];     //等待匹配货物
GOODS_STATE matchedGoods[MAX_GOODS_NUM];     //已被匹配待拾起货物  
GOODS_STATE pickedGoods[MAX_GOODS_NUM];      //已被拾起货物


UAV initEnemyUav[MAX_UAV_NUM];               //一开始初始化时敌我双方无人机
int initEnemyUavNum = 0;

UAVS trackerUav[MAX_UAV_NUM];
int trackerUavNum = 0;

typedef struct _ENEMYUAV_
{
    int enemyUavnNO;
    int nEndX;                               //货物终点
    int nEndY;
    int nTotalValue;                         //总的价值
    int nRemainStepNum;                      //到达货物终点剩余步子数
}ENEMYUAV;

ENEMYUAV enemyCarryGoodsUav[MAX_UAV_NUM];         //敌人载货无人机
int enemyCarryGoodsUavNum = 0;



int DebugFlag = 0;

int sendUavFromHomeFlag = 0;                 //从家中调度一架无人机执行任务标致  0:未调度  1:调度了一架


static void PathNodeNeighbors(ASNeighborList neighbors, void *node, void *context)          //设置邻接节点列表
{
    PathNode *pathNode = (PathNode *)node;   
    PathNode tmpNode;
    PathNode *ptmpNode;  

    //水平和垂直邻接节点
    if (threeDMap[serachPathOfZ][pathNode->x+1][pathNode->y] == 0) {

        tmpNode.x = pathNode->x+1;
        tmpNode.y = pathNode->y;
        ptmpNode = &tmpNode;
        ASNeighborListAdd(neighbors, ptmpNode, 1);
    }
    if (threeDMap[serachPathOfZ][pathNode->x-1][pathNode->y] == 0) {

        tmpNode.x = pathNode->x-1;
        tmpNode.y = pathNode->y;
        ptmpNode = &tmpNode;
        ASNeighborListAdd(neighbors, ptmpNode, 1);
    }
    if (threeDMap[serachPathOfZ][pathNode->x][pathNode->y+1] == 0) {
        tmpNode.x = pathNode->x;
        tmpNode.y = pathNode->y+1;
        ptmpNode = &tmpNode;
        ASNeighborListAdd(neighbors, ptmpNode, 1);
    }
    if (threeDMap[serachPathOfZ][pathNode->x][pathNode->y-1] == 0) {
        tmpNode.x = pathNode->x;
        tmpNode.y = pathNode->y-1;
        ptmpNode = &tmpNode;
        ASNeighborListAdd(neighbors, ptmpNode, 1);
    }
    //斜对角邻接节点 ,其代价均一样为一个单位
    if (threeDMap[serachPathOfZ][pathNode->x+1][pathNode->y+1] == 0) {
        tmpNode.x = pathNode->x+1;
        tmpNode.y = pathNode->y+1;
        ptmpNode = &tmpNode;
        ASNeighborListAdd(neighbors, ptmpNode, 1);
    }
    if (threeDMap[serachPathOfZ][pathNode->x-1][pathNode->y-1] == 0) {
        tmpNode.x = pathNode->x-1;
        tmpNode.y = pathNode->y-1;
        ptmpNode = &tmpNode;
        ASNeighborListAdd(neighbors, ptmpNode, 1);
    }
    if (threeDMap[serachPathOfZ][pathNode->x-1][pathNode->y+1] == 0) {
        tmpNode.x = pathNode->x-1;
        tmpNode.y = pathNode->y+1;
        ptmpNode = &tmpNode;
        ASNeighborListAdd(neighbors, ptmpNode, 1);
    }
    if (threeDMap[serachPathOfZ][pathNode->x+1][pathNode->y-1] == 0) {
        tmpNode.x = pathNode->x+1;
        tmpNode.y = pathNode->y-1;
        ptmpNode = &tmpNode;
        ASNeighborListAdd(neighbors, ptmpNode, 1);
    }
}

static float PathNodeHeuristic(void *fromNode, void *toNode, void *context)   //采用曼哈顿距离 distance作为启发式搜索方式
{
    PathNode *from = (PathNode *)fromNode;
    PathNode *to = (PathNode *)toNode;

    return (fabs(from->x - to->x) + fabs(from->y - to->y));
}

static const ASPathNodeSource PathNodeSource =
{
    sizeof(PathNode),
    &PathNodeNeighbors,
    &PathNodeHeuristic,
    NULL,
    NULL
};

//无人机从停机坪初始上升的路径
PATH initPath[MAX_UAV_NUM][200];

void InitPath(int initTotalNum)
{
    int count =0;
    for(int i = 0;i<initTotalNum;i++)
    {
        for(int j=0;j<mapHHigh;j++)
        {
            initPath[i][j].x = 0;
            initPath[i][j].y = 0;
            initPath[i][j].z = 0;
        }
    }
    for(int i = 0;i<initTotalNum;i++)
    {
        for(int j=i;j<mapHHigh;j++)
        {
            initPath[i][j].z = count;
            count ++;
        }
    }
}

int UAVPriceListContent(const void *data1, const void *data2)
{
    UAV_PRICE *p1 = (UAV_PRICE *)data1;
    UAV_PRICE *p2 = (UAV_PRICE *)data2;
    return (p1->nValue - p2->nValue);    
}

//功能:敌人载货无人机的总价值排序
int EnemyCarryGoodsUAVCompareContent(const void *data1, const void *data2)
{
    ENEMYUAV *p1 = (ENEMYUAV *)data1;
    ENEMYUAV *p2 = (ENEMYUAV *)data2;
    return (p2->nTotalValue - p1->nTotalValue);                                //由大到小排序       
}

//敌人无人机排序函数  按载重排序
int EnemyUAVCompareContent(const void *data1, const void *data2)
{
    UAV *p1 = (UAV *)data1;
    UAV *p2 = (UAV *)data2;
    return (p1->nLoadWeight - p2->nLoadWeight);  //由小到大达排序
}

//无人机排序函数
int UAVCompareContent(const void *data1, const void *data2)
{
    UAVS *p1 = (UAVS *)data1;
    UAVS *p2 = (UAVS *)data2;
    return (p1->baseInfo.nLoadWeight - p2->baseInfo.nLoadWeight);  //由小到达排序
}
//货物排序函数
int GOODSCompareContent(const void *data1, const void *data2)  
{
    GOODS_STATE *p1 = (GOODS_STATE *)data1;
    GOODS_STATE *p2 = (GOODS_STATE *)data2;
    return (p2->goods.nValue - p1->goods.nValue);                //由大到小排序
} 
//
int SearchGoodsNo(const GOODS_STATE *goodsState,int curstatenum,int goodNos)       //需要被执行三次,因为有三种状态
{
    for(int i=0;i<curstatenum;i++)
    {
        if(goodNos == goodsState[i].nNO)                                           //
        {
            return i;
        }
    }
    return -1;
}

void InsertUnmatchGoods(GOODS_STATE *goodsState)
{
    int tmpIndex=0;
    int goodNo = goodsState->goods.nNO;
    tmpIndex = SearchGoodsNo(unmatchGoods,unmatchGoodsNum,goodNo); 
    if(tmpIndex !=-1)      
    {
        return ;           //说明该货物本身存在其中,直接退出
    }   
    goodsState->nNO = goodsState->goods.nNO;
    goodsState->UAVNO = -1;
    goodsState->goodMatchState=UNMATCHED;
    unmatchGoods[unmatchGoodsNum] = *goodsState;
    unmatchGoodsNum++;
    qsort(unmatchGoods,unmatchGoodsNum,sizeof(unmatchGoods[0]),GOODSCompareContent);   

}

void DeleteUnmatchGoods(GOODS_STATE *goodsState)
{
    int tmpIndex=0;
    int goodNo = goodsState->nNO;
    tmpIndex = SearchGoodsNo(unmatchGoods,unmatchGoodsNum,goodNo); 
    if(tmpIndex ==-1)      
    {
        return ;                                //未搜寻到该货物,直接退出
    }   
    for(int i=tmpIndex;i<unmatchGoodsNum;i++)
    {
        unmatchGoods[i] = unmatchGoods[i+1];
    }
    unmatchGoodsNum--;
}

void InsertMatchedGoods(GOODS_STATE *goodsState)
{
    int tmpIndex=0;
    int goodNo = goodsState->goods.nNO;
    tmpIndex = SearchGoodsNo(matchedGoods,matchedGoodsNum,goodNo); 
    if(tmpIndex !=-1)      
    {
        return ;           //说明该货物本身存在其中,直接退出
    }  
    //printf("insert=%d",goodsState->goods.nNO);
    goodsState->nNO = goodsState->goods.nNO;
    goodsState->goodMatchState=MATCHED;
    matchedGoods[matchedGoodsNum] = *goodsState;
    //printf("insert:%d %d",matchedGoods[matchedGoodsNum].goods.nNO,goodsState->nNO);
    matchedGoodsNum++;    
}

void DeleteMatchedGoods(GOODS_STATE *goodsState)
{
    int tmpIndex=0;
    int goodNo = goodsState->nNO;
    tmpIndex = SearchGoodsNo(matchedGoods,matchedGoodsNum,goodNo);   
    if(tmpIndex ==-1)      
    {
        return ;           //未搜寻到该货物,直接退出
    }    
    for(int i=tmpIndex;i<matchedGoodsNum;i++)
    {
        matchedGoods[i] = matchedGoods[i+1];
    }
    matchedGoodsNum--;
}

void InsertPickedGoods(GOODS_STATE *goodsState)
{
    int tmpIndex=0;
    int goodNo = goodsState->goods.nNO;
    tmpIndex = SearchGoodsNo(pickedGoods,pickedGoodsNum,goodNo); 
    if(tmpIndex !=-1)      
    {
        return ;           //未搜寻到该货物,直接退出
    }   
    goodsState->nNO = goodsState->goods.nNO;
    goodsState->goodMatchState = PICKED;         //
    pickedGoods[pickedGoodsNum] = *goodsState;
    pickedGoodsNum++;  
}

void DeletePickedGoods(GOODS_STATE *goodsState)
{
    int tmpIndex=0;
    int goodNo = goodsState->nNO;
    tmpIndex = SearchGoodsNo(pickedGoods,pickedGoodsNum,goodNo);  
    if(tmpIndex ==-1)      
    {
        return ;           //未搜寻到该货物,直接退出
    }     
    for(int i=tmpIndex;i<pickedGoodsNum;i++)
    {
        pickedGoods[i] = pickedGoods[i+1];
    }
    pickedGoodsNum--;
}

//在数组中查找是否有该无人机标号,如果有,则返回该标号无人机索引,如果没有则返回-1,说明该无人机是刚刚购买的,则将其添加到空闲状态下
int SearchUAVNos(const UAVS *uavs,int curstatenum,int uavNos)       //需要被执行三次,因为有三种状态
{
    for(int i=0;i<curstatenum;i++)
    {
        if(uavNos == uavs[i].nNO)        //
        {
            return i;
        }
    }
    return -1;
}


int SearchEnemyUAVNos(const UAV *uav,int curstatenum,int uavNos)       //需要被执行三次,因为有三种状态
{
    for(int i=0;i<curstatenum;i++)
    {
        if(uavNos == uav[i].nNO)        //
        {
            return i;
        }
    }
    return -1;
}



void InsertIdleUAV(UAVS *uavs)
{
    int tmpIndex=0;
    int UAVNo = uavs->baseInfo.nNO;
    tmpIndex = SearchUAVNos(idleUAV,idleStatusUAVNum,UAVNo);        //是否已经存在该列表中
    if(tmpIndex !=-1)      
    {
        return ;           //未搜寻到该无人机,直接退出
    }   
    uavs->nNO = uavs->baseInfo.nNO;
    uavs->taskState = UAV_TASK_IDLE;
    uavs->goodMatchState = UNMATCHED;
    uavs->goodsNo = -1;                              //此为无人机匹配到的货物编号
    uavs->baseInfo.nGoodsNo = -1;                    //告诉服务器,当前处于空闲中
    idleUAV[idleStatusUAVNum] = *uavs;   
    idleStatusUAVNum++;
    qsort(idleUAV,idleStatusUAVNum,sizeof(idleUAV[0]),UAVCompareContent);      //其实也花费不了多长时间
}

void DeleteIdleUAV(UAVS *uavs)
{
    int tmpIndex=0;
    int UAVNo = uavs->nNO;
    tmpIndex = SearchUAVNos(idleUAV,idleStatusUAVNum,UAVNo);    //找到需要删除无人机的索引
    if(tmpIndex ==-1)      
    {
        return ;           //未搜寻到该无人机,直接退出
    }   
    for(int i=tmpIndex;i<idleStatusUAVNum;i++)
    {
        idleUAV[i] = idleUAV[i+1];
    }
    idleStatusUAVNum--;
}

void InsertAcceptedUAV(UAVS *uavs)
{
    int tmpIndex=0;
    int UAVNo = uavs->baseInfo.nNO;
    tmpIndex = SearchUAVNos(acceptedUAV,acceptedStatusUAVNum,UAVNo);    //找到需要删除无人机的索引
    if(tmpIndex !=-1)      
    {
        return ;           //未搜寻到该无人机,直接退出
    }   
    uavs->nNO = uavs->baseInfo.nNO;
    uavs->taskState = UAV_TASK_ACCEPTED;
    uavs->goodMatchState = MATCHED;
    acceptedUAV[acceptedStatusUAVNum] = *uavs;    
    acceptedStatusUAVNum++;
    qsort(acceptedUAV,acceptedStatusUAVNum,sizeof(acceptedUAV[0]),UAVCompareContent);    //
}

void DeleteAcceptedUAV(UAVS *uavs)
{
    int tmpIndex=0;
    int UAVNo = uavs->nNO;
    tmpIndex = SearchUAVNos(acceptedUAV,acceptedStatusUAVNum,UAVNo);    //找到需要删除无人机的索引
    if(tmpIndex ==-1)      
    {
        return ;           //未搜寻到该无人机,直接退出
    }   
    for(int i=tmpIndex;i<acceptedStatusUAVNum;i++)
    {
        acceptedUAV[i] = acceptedUAV[i+1];
    }
    acceptedStatusUAVNum--;
}

void InsertExecutingUAV(UAVS *uavs)
{
    int tmpIndex=0;
    int UAVNo = uavs->baseInfo.nNO;
    tmpIndex = SearchUAVNos(executingUAV,executingStatusUAVNum,UAVNo);    //找到需要删除无人机的索引
    if(tmpIndex !=-1)      
    {
        return ;           //未搜寻到该无人机,直接退出
    }   
    uavs->nNO = uavs->baseInfo.nNO;
    uavs->taskState = UAV_TASK_EXECUTING;
    uavs->goodMatchState = MATCHED;
    executingUAV[executingStatusUAVNum] = *uavs;
    executingStatusUAVNum++;
}

void DeleteExecutingUAV(UAVS *uavs)
{
    int tmpIndex=0;
    int UAVNo = uavs->nNO;
    tmpIndex = SearchUAVNos(executingUAV,executingStatusUAVNum,UAVNo);    //找到需要删除无人机的索引
    if(tmpIndex ==-1)      
    {
        return ;           //未搜寻到该无人机,直接退出
    }   
    for(int i=tmpIndex;i<executingStatusUAVNum;i++)
    {
        executingUAV[i] = executingUAV[i+1];
    }
    executingStatusUAVNum--;
}
//敢死队
void InsertDeatSquadsUAV(UAVS *uavs)
{
    int tmpIndex=0;
    int UAVNo = uavs->baseInfo.nNO;
    tmpIndex = SearchUAVNos(deatSquadsUAV,deatSquadsUAVNum,UAVNo);    //找到需要删除无人机的索引
    if(tmpIndex !=-1)      
    {
        return ;                   //说明已经有了该飞机,不需要再次插入,直接退出 
    }   
    uavs->nNO = uavs->baseInfo.nNO;
    uavs->taskState = UAV_TASK_DEATH;
    uavs->goodMatchState = UNMATCHED;
    uavs->baseInfo.nGoodsNo = -1;             //告诉服务器处于空闲状态下
    deatSquadsUAV[deatSquadsUAVNum] = *uavs;
    deatSquadsUAVNum++;
}

void DeleteDeatSquadsUAV(UAVS *uavs)
{
    int tmpIndex=0;
    int UAVNo = uavs->nNO;
    tmpIndex = SearchUAVNos(deatSquadsUAV,deatSquadsUAVNum,UAVNo);    //找到需要删除无人机的索引
    if(tmpIndex ==-1)      
    {
        return ;           //未搜寻到该无人机,直接退出
    }   
    for(int i=tmpIndex;i<deatSquadsUAVNum;i++)
    {
        deatSquadsUAV[i] = deatSquadsUAV[i+1];
    }
    deatSquadsUAVNum--;
}

//追踪者
void InsertTrackerUAV(UAVS *uavs)
{
    int tmpIndex=0;
    int UAVNo = uavs->baseInfo.nNO;
    tmpIndex = SearchUAVNos(trackerUav,trackerUavNum,UAVNo);    //找到需要删除无人机的索引
    if(tmpIndex !=-1)      
    {
        return ;                   //说明已经有了该飞机,不需要再次插入,直接退出 
    }   
    
    uavs->baseInfo.nGoodsNo = -1;             //告诉服务器处于空闲状态下
    trackerUav[trackerUavNum] = *uavs;
    trackerUavNum++;
}

void DeleteTrackerUAV(UAVS *uavs)
{
    int tmpIndex=0;
    int UAVNo = uavs->nNO;
    tmpIndex = SearchUAVNos(trackerUav,trackerUavNum,UAVNo);    //找到需要删除无人机的索引
    if(tmpIndex ==-1)      
    {
        return ;           //未搜寻到该无人机,直接退出
    }   
    for(int i=tmpIndex;i<trackerUavNum;i++)
    {
        trackerUav[i] = trackerUav[i+1];
    }
    trackerUavNum--;
}


//敌人无人机的插入和删除
void InsertEnemyUAV(UAV *uav)
{    
    initEnemyUav[initEnemyUavNum] = *uav;
    initEnemyUavNum++;
    qsort(initEnemyUav,initEnemyUavNum,sizeof(initEnemyUav[0]),EnemyUAVCompareContent);      //其实也花费不了多长时间
}

void DeleteEnemyUAV(UAV *uav)
{
    int tmpIndex=0;
    int UAVNo = uav->nNO;
    tmpIndex = SearchEnemyUAVNos(initEnemyUav,initEnemyUavNum,UAVNo);    //找到需要删除无人机的索引
    if(tmpIndex ==-1)      
    {
        return ;           //未搜寻到该无人机,直接退出
    }   
    for(int i=tmpIndex;i<initEnemyUavNum;i++)
    {
        initEnemyUav[i] = initEnemyUav[i+1];
    }
    initEnemyUavNum--;
}


void GoodsInfoStorge(GOODS *good_dens,GOODS good_sour)
{
    good_dens->nNO = good_sour.nNO;
    good_dens->nStartX = good_sour.nStartX;
    good_dens->nStartY = good_sour.nStartY;
    good_dens->nEndX = good_sour.nEndX;
    good_dens->nEndY = good_sour.nEndY;
    good_dens->nWeight = good_sour.nWeight;
    good_dens->nValue = good_sour.nValue;
    good_dens->nStartTime = good_sour.nStartTime;
    good_dens->nRemainTime = good_sour.nRemainTime;
    good_dens->nLeftTime = good_sour.nLeftTime;
    good_dens->nState = good_sour.nState;
}

void UAVInfoStorgUpdate(UAV *uav_dens,UAV good_sour)
{
    uav_dens->nNO = good_sour.nNO;
    strcpy(uav_dens->szType,good_sour.szType);                
    uav_dens->nX = good_sour.nX;
    uav_dens->nY = good_sour.nY;
    uav_dens->nZ = good_sour.nZ;
    uav_dens->nLoadWeight = good_sour.nLoadWeight;
    uav_dens->nRemainElectricity = good_sour.nRemainElectricity;
    uav_dens->nStatus = good_sour.nStatus;
    uav_dens->nGoodsNo = good_sour.nGoodsNo;
}


//功能:已知起点和终点,在三维地图中寻找一条最短路,并返回路径和步数
void SearchShortPath(PathNode pathfrom,PathNode pathto,ASPath *path , int *stepnum,int *pathhigh)
{
    ASPath tmpPath = NULL;  
    int tmpTotalStepNum = -1;
    int tmpPathHigh = -1;
    serachPathOfZ = mapHLow;                                            //每次在使用前都需要将其初始化为最低飞行高度
    for(int i=serachPathOfZ;i<=buildingMaxHigh;i++,serachPathOfZ++)
    {
        ASPath Path = ASPathCreate(&PathNodeSource, NULL, &pathfrom, &pathto);         //执行A*
        int levelStepCount =  ASPathGetCount(Path)-1;                           //水平移动步数
        if(levelStepCount < 0)                                                  //说明此路不通
        {
            continue ;
        }
        int totalStepNum = levelStepCount + serachPathOfZ * 2;                  //水平步数加垂直步数,后面的耗电就需要根据这个来算
        if(tmpTotalStepNum == -1)
        {
            tmpTotalStepNum = totalStepNum;
            tmpPathHigh = serachPathOfZ;                                       //记录该路所在高度
            tmpPath = Path;
        }
        else
        {
            if(totalStepNum < tmpTotalStepNum)                //成立说明有距离更短的路
            {
                tmpTotalStepNum = totalStepNum;
                tmpPathHigh = serachPathOfZ;                                       //记录该路所在高度
                tmpPath = Path;
            }
        }
    }
    serachPathOfZ = mapHLow;      
    *path = tmpPath;
    *stepnum = tmpTotalStepNum;
    *pathhigh = tmpPathHigh;
}


//所有的无人机都会被分配到空闲状态数组下,所有的货物均会被分配到未匹配状态数组下
void FirstStateProcess(MATCH_STATUS * pstMatch)
{
    UAVS initUavs;
    GOODS_STATE initGoodsState;
    //我方无人机分类   
    int totalUAVNum = pstMatch->nUavWeNum;
    for(int i=0;i<totalUAVNum;i++)
    {
               
        printf("NO:%d type=%s w=%d r=%d \n",pstMatch->astWeUav[i].nNO,pstMatch->astWeUav[i].szType,pstMatch->astWeUav[i].nLoadWeight,pstMatch->astWeUav[i].nRemainElectricity);
        
        int uavPriceIndex = -1;
        for(int j=0;j<uavPriceNum;j++)
        {
            if(strcmp(UAVPriceList[j].szType,pstMatch->astWeUav[i].szType)==0)                //查找对应的价格表
            {
                uavPriceIndex = j;
                break;
            }
        }
        if(uavPriceIndex == -1)                    //说明价格表里面没有这号无人机,基本上不可能
        {
            printf("this type uav is not in priceList\n");
            continue;
        }
        initUavs.nCharge = UAVPriceList[uavPriceIndex].nCharge;
        initUavs.nCapacity = UAVPriceList[uavPriceIndex].nCapacity;

        initUavs.stepNum = mapHLow; 
        for(int j=0;j<mapHLow;j++)                           //新增加的无人机保持在原地不动
        {
            initUavs.pPath[j].x = pstMatch->astWeUav[i].nX;               
            initUavs.pPath[j].y = pstMatch->astWeUav[i].nY;               
            initUavs.pPath[j].z = pstMatch->astWeUav[i].nZ;
           
        }                           
        initUavs.pathIndex = 0;
        initUavs.goodsNo = -1;
        UAVInfoStorgUpdate(&initUavs.baseInfo,pstMatch->astWeUav[i]);

        InsertIdleUAV(&initUavs);        
    }

    //敌方无人机分类
    for(int i=0;i<pstMatch->nUavEnemyNum;i++)
    {
        InsertEnemyUAV(&pstMatch->astEnemyUav[i]);               //插入敌人初始时的无人机
    }
   
    //货物分类
    int totalGoodNum = pstMatch->nGoodsNum;
    for(int i=0;i<totalGoodNum;i++)
    {
       // initGoodsState.goods = pstMatch->astGoods[i];
       if(goodsDetecMap[pstMatch->astGoods[i].nStartX][pstMatch->astGoods[i].nStartY] != 0)
        {
            //printf("the goal point 0f goods is in the wall\n");
            continue;
        }
       if(goodsDetecMap[pstMatch->astGoods[i].nEndX][pstMatch->astGoods[i].nEndY] != 0)
        {
            //printf("the goal point 0f goods is in the wall\n");
            continue;
        }
        initGoodsState.distanceStartToEnd = -1;                         //表示还未开始存储
        GoodsInfoStorge(&initGoodsState.goods,pstMatch->astGoods[i]);

        pathFrom.x = initGoodsState.goods.nStartX;
        pathFrom.y = initGoodsState.goods.nStartY;  

        pathTo.x = initGoodsState.goods.nEndX;
        pathTo.y = initGoodsState.goods.nEndY;   
        
        ASPath path;    
        int distance = 0;
        int pathHigh = 0;
        SearchShortPath(pathFrom,pathTo,&path,&distance,&pathHigh);
        initGoodsState.pathStartToEnd = path;
        initGoodsState.distanceStartToEnd = distance;
        initGoodsState.pathHigh = pathHigh;                              //记录路径所在高度
        initGoodsState.levelStepNum = distance - (pathHigh * 2);         //水平步子数
        initGoodsState.needCapacity = distance * initGoodsState.goods.nWeight;             //所需要的电量       
        InsertUnmatchGoods(&initGoodsState);         
    }
}

//根据货物编号搜寻敌人战机
//返回战机类型 -1:没有查找到敌机 否则为敌人无人机相应编号
int SearchEnemyUAVType(const MATCH_STATUS * pstMatch, int uavNos)       
{
    int enemyUAVType = -1;
    for(int i=0;i<pstMatch->nUavEnemyNum ;i++)
    {
        if(uavNos == pstMatch->astEnemyUav[i].nGoodsNo)        //
        {
            sscanf(pstMatch->astEnemyUav[i].szType,"F%d",&enemyUAVType);
            return enemyUAVType;
        }
    }
    return -1;
}

//功能:根据被敌机器载起货物编号计算出敌机和货物总的价值
//返回值: -1.计算失败     其他.价值实际值
//该模块没问题
int CaculateEnemyTotalValue(const MATCH_STATUS * pstMatch,const GOODS_STATE *goodsState,int curstatenum,int enemyGoodNO,int *enemyUavValue)
{
    int enemyUavType = SearchEnemyUAVType(pstMatch,enemyGoodNO);              //查询敌人无人机型号 
    
    if(enemyUavType == -1)               //没找到敌机
    {
        return -1;
    }
    char uavType[3] = "\0";
    char tmp[2] = "\0";
    int tmpEnemyUavValue = 0;
    sprintf(tmp,"%d",enemyUavType);
    strcat(uavType,"F");
    strcat(uavType,tmp);
   
    for(int i=0;i<uavPriceNum;i++)
    {
        if(strcmp(UAVPriceList[i].szType,uavType)==0)
        {
            
            tmpEnemyUavValue = UAVPriceList[i].nValue;                 //获取敌机价格
            break ;
        }
    }
    
    if(tmpEnemyUavValue == 0)                                    
    {
        return -1;
    } 
    *enemyUavValue = tmpEnemyUavValue;
    int goodsValue = 0;                                                             //这个时候货物还处于匹配状态下 
    int tmpGoodIndex = SearchGoodsNo(goodsState,curstatenum,enemyGoodNO);           //在已经匹配的货物中搜寻索引          
    if(tmpGoodIndex == -1)
    {
        return -1;
    }      
    goodsValue = goodsState[tmpGoodIndex].goods.nValue;                             //获取货物价值
    
    return (tmpEnemyUavValue + goodsValue);    
}
//功能:如果匹配的货物被敌人抢先拾取应作出怎样的选择
//1.如果我方该无人机价值大于敌人拾取货物和无人机总价值,则改变我方无人机相应的状态
//2.如果低于敌人总价值,则要根据我方战机所在位置是否需要修改状态
//(1).我方战机也准备下去拾取该货物,但被敌人先拾取,这个时候不要修改我方战机状态,直接让其下降撞毁敌机
//(2).我方战机到货物点最正上方最低飞行高度剩余步数是否小于敌机上升的高度-1,如果是,则不需要改变状态,直接飞过去和其相撞
//返回值:-1 改变状态(正常执行) 1 不改变状态(可以撞毁敌机)
//该模块Ok
int WhetherChangeAcceptedToIdle(const MATCH_STATUS * pstMatch, const UAVS *uavs, int enemyGoodNO)                   //这个函数有bug明天调一下,调完明天就准备复习了
{ 
    int enemyUavValue = 0;
    int enemyTotalValue = CaculateEnemyTotalValue(pstMatch,matchedGoods,matchedGoodsNum,enemyGoodNO,&enemyUavValue);
   
    if(enemyTotalValue == -1)
    {
        return -1;                                 //计算失败
    }
    int ourUavValue = 0;
    for(int i=0;i<uavPriceNum;i++)
    {
        if(strcmp(UAVPriceList[i].szType,uavs->baseInfo.szType)==0)
        {
            ourUavValue = UAVPriceList[i].nValue;                 
            break ;
        }
    }
    if(ourUavValue > enemyUavValue)                                             //这里只需要判断是否大于敌人无人机价值,如果是,则这里不进行碰撞,在后面会有蹲点判断
    {
        return -1;
    }
    int restLevelStepNum = uavs->stepNum - mapHLow - uavs->pathIndex;              //为安全起见,加个1就不用太纠结了,最后面那个2是考虑万一途中出现和自己撞击的情况需要上升一个高度而增加2步(此时发生和自己撞击的概率就特别小)
    if(restLevelStepNum > mapHLow)                                                    //是否来得及赶到敌机上方,成立说明来不及
    {                                                               //restLevelStepNum 这个值可能为负数,此时说明 飞机也正在下降捡货,只是被敌人提前几步
        return -1;
    }
    return 1;
    //找出货物价值        
}

//拦截敌机的路径规划,这个路径就不需要下降到最低点了,只要到达最低飞行高度即可,如果最后低于最低飞行高度的话,会出现很多自己相矛盾的问题,处理起来很棘手
//第三个参数意义: 0:表示最低飞行高度 ,downStep表示下降到最低飞行高度多少步
//在捡货和放货的过程中,后面两个参数是相等的
void RoutePlan(UAVS *uavs,ASPath *path,int levelStepCount,int downStep,int pathHigh)
{
    int UAVInitHigh = uavs->baseInfo.nZ; 
    uavs->pPath[0].x = uavs->baseInfo.nX;
    uavs->pPath[0].y = uavs->baseInfo.nY;
    uavs->pPath[0].z = uavs->baseInfo.nZ;
    uavs->pPath[1].z = uavs->baseInfo.nZ + 1;
    uavs->pathIndex = 1;                                     //0为飞机当前位置

    if(UAVInitHigh < pathHigh)
    {   
        int firstClimbHigh = pathHigh-UAVInitHigh;
        uavs->stepNum = levelStepCount  + firstClimbHigh + downStep + 1;                  //到达目的地需要执行的步数,因为货物是在地面上,所以总的需要步数还需要加上无人机本身的高度   
        for(int i=1;i <= firstClimbHigh;i++)
        {
            uavs->pPath[i].x =  uavs->baseInfo.nX;                                    //水平位置不变,只做垂直上升运动  
            uavs->pPath[i].y =  uavs->baseInfo.nY;         
            uavs->pPath[i+1].z = uavs->pPath[i].z+1; 
        }
        for (int j=1,i=firstClimbHigh + 1; i<=firstClimbHigh + levelStepCount; j++,i++)        //将路经保存到对应无人机数据结构中
        {           
            PathNode *pathNode = (PathNode *)ASPathGetNode(*path, j);
            uavs->pPath[i].x = pathNode->x;
            uavs->pPath[i].y = pathNode->y;
            uavs->pPath[i].z = uavs->pPath[i-1].z;        //在二维平面移动时,高度就保持不变           
        }
        /*敢死队不需要下降到地面,下降到最低飞行高度的一半,别人的清道夫就很难处理了*/
        for(int i=firstClimbHigh + levelStepCount +1 ;i<uavs->stepNum;i++)   //只做垂直移动,水平坐标保持不变
        {
            uavs->pPath[i].x =  uavs->pPath[i-1].x;      
            uavs->pPath[i].y =  uavs->pPath[i-1].y;         
            uavs->pPath[i].z = uavs->pPath[i-1].z - 1;                    //垂直方向下降一个高度
        }
    }
    else          //要么处于最低飞行高度,要么高于最低飞行高度
    {        
        uavs->stepNum = levelStepCount + (UAVInitHigh - pathHigh) + downStep + 1;
        for (int j=1,i=1; i<=levelStepCount; j++,i++)                             //将路经保存到对应无人机数据结构中
        {   
            PathNode *pathNode = (PathNode *)ASPathGetNode(*path, i);
            uavs->pPath[j].x = pathNode->x;
            uavs->pPath[j].y = pathNode->y;
            uavs->pPath[j].z = UAVInitHigh;                         //在二维平面移动时,高度就保持不变
        }
        
        for(int i=levelStepCount + 1;i<uavs->stepNum;i++)                    //只做垂直移动,水平坐标保持不变
        {
            uavs->pPath[i].x =  uavs->pPath[i-1].x;      
            uavs->pPath[i].y =  uavs->pPath[i-1].y;         
            uavs->pPath[i].z = uavs->pPath[i-1].z - 1;                                  //垂直方向下降一个高度
        }
    } 
}

//功能:敌人捡货后判断是否是被追踪目标
//返回值:0:表示是被追踪的 -1:不是被追踪的
//该模块没有问题
int enemyGoodsUavWhethTracked(MATCH_STATUS * pstMatch,int goodsNo,int *enemyUavnNO)
{
    int tmpEnemyUavnNO = -1;
    for(int i=0;i<pstMatch->nUavEnemyNum ;i++)
    {
        if(goodsNo == pstMatch->astEnemyUav[i].nGoodsNo)        //成立说明找到了载有该货物标号的敌人无人机
        {
            tmpEnemyUavnNO = pstMatch->astEnemyUav[i].nNO;
            for(int j=0;j<trackerUavNum;j++)
            {
                if(tmpEnemyUavnNO == trackerUav[j].nEnemyUavNO)   //
                {
                    *enemyUavnNO = tmpEnemyUavnNO;
                    return 0;
                }
            }
            break;
        }
    }
    return -1;
}


//当敌机拾取我方未匹配货物时,是否需要派遣我方战机守在货物目标点等待撞击
//撞击条件:1.我方存在空闲无人机 2.我方空闲无人机价值比敌方无人机及价值总额要少,3.我方该空闲无人机比敌方无人机更早到货物目标点
//需要买几架单价最小的无人机,只有最小的无人机才有可能会是空闲状态
//返回值:-1:未找到敢死队无人机 0:找到了敢死队无人机
//该模块没有问题
int OccupyGoodGoalPoint(const MATCH_STATUS * pstMatch, int enemyGoodNO)                           
{
   
    //计算敌方总价值               
    int enemyUavValue = 0;
    int enemyTotalValue = CaculateEnemyTotalValue(pstMatch,unmatchGoods,unmatchGoodsNum,enemyGoodNO,&enemyUavValue);        //该模块Ok
    if(enemyTotalValue != -1)                                       //成立说明找到了,不需要进入else中
    {
        
    }
    else
    {
        enemyTotalValue = CaculateEnemyTotalValue(pstMatch,matchedGoods,matchedGoodsNum,enemyGoodNO,&enemyUavValue);        //该模块Ok
        if(enemyTotalValue == -1)
        {
            return 0;               //未能搜索到
        } 
    }
    
    int enemyUavTotalStepNum = 0;
    int ourUavTotalStepNum = 0;

    //计算敌我双方到达目的地的路径
    int tmpGoodIndex = SearchGoodsNo(unmatchGoods,unmatchGoodsNum,enemyGoodNO);              //在未匹配的货物中搜寻索引          
    if(tmpGoodIndex != -1)              
    {
        pathTo.x = unmatchGoods[tmpGoodIndex].goods.nEndX;
        pathTo.y = unmatchGoods[tmpGoodIndex].goods.nEndY;
        enemyUavTotalStepNum = unmatchGoods[tmpGoodIndex].distanceStartToEnd;                //敌人无人机完成放货需要行走的步子数
    }   
    else
    {
        int tmpGoodIndex = SearchGoodsNo(matchedGoods,matchedGoodsNum,enemyGoodNO);           //在匹配的货物中搜寻索引 
        if(tmpGoodIndex == -1)
        {
            return 0;               //未能搜索到
        } 
        pathTo.x = matchedGoods[tmpGoodIndex].goods.nEndX;
        pathTo.y = matchedGoods[tmpGoodIndex].goods.nEndY;
        enemyUavTotalStepNum = matchedGoods[tmpGoodIndex].distanceStartToEnd;                //敌人无人机完成放货需要行走的步子数
    } 

    int findDeatSquadsUAVIndex = 0;
    if(idleStatusUAVNum>0)                  //有空闲的就从空闲的里面选,否则就从接受任务中的选
    {
        for(int i=0;i<idleStatusUAVNum;i++)                                    //遍历满足要求的无人机              
        {
            int ourUavValue = 0;
            for(int j=0;j<uavPriceNum;j++)           //计算我方无人机价格
            {
                if(strcmp(UAVPriceList[j].szType,idleUAV[i].baseInfo.szType)==0)
                {
                    ourUavValue = UAVPriceList[i].nValue;                 
                    break ;
                }
            }
            if(ourUavValue > enemyTotalValue)            //我方无人机大于敌机和货物总价值,则不撞
            {
                continue ;
            }       
            //我方无人机起点

            ASPath  ourPath;
            int distance = 0;
            int pathHigh = 0;
            int levelStepCount = 0;

            pathFrom.x = idleUAV[i].baseInfo.nX;
            pathFrom.y = idleUAV[i].baseInfo.nY;

            SearchShortPath(pathFrom,pathTo,&ourPath,&distance,&pathHigh); 
            levelStepCount = distance - pathHigh * 2;
            ourUavTotalStepNum = levelStepCount + pathHigh + abs(pathHigh - idleUAV[i].baseInfo.nZ);

            //计算以上两个变量值
            //还有要判断我方战机是否处于水平位置
            //if(ourUavTotalStepNum > (enemyUavTotalStepNum + 2*mapHLow))           //这个条件成立说明要撞敌人的空飞机
            if(ourUavTotalStepNum > enemyUavTotalStepNum)                           //敌人比我方抢先到达目的地,因为我不想只撞空飞机,则匹配下一个无人机
            {
                continue ;
            }           
                
            RoutePlan(&idleUAV[i],&ourPath,levelStepCount,pathHigh,pathHigh);         //直接下降到地面

            //从空闲状态变为敢死队状态
            printf("uavno=%d\n",idleUAV[i].baseInfo.nNO);
            InsertDeatSquadsUAV(&idleUAV[i]);    
            DeleteIdleUAV(&idleUAV[i]);  
            //printf("egx=%d egy=%d\n",pathTo.x,pathTo.y);        
            findDeatSquadsUAVIndex = 1;                      //说明在空闲中找到了敢死队
            //找到了就直接退出该函数
            return 0;                                       
        }
    }
    if(findDeatSquadsUAVIndex == 0)                  //说明未从空闲中找到敢死队
    {
        for(int i=0;i<acceptedStatusUAVNum-1;i++)                                    //遍历满足要求的无人机              
        {
            int ourUavValue = 0;
            for(int j=0;j<uavPriceNum;j++)           //计算我方无人机价格
            {
                if(strcmp(UAVPriceList[j].szType,acceptedUAV[i].baseInfo.szType)==0)
                {
                    ourUavValue = UAVPriceList[i].nValue;                 
                    break ;
                }
            }
            if(ourUavValue > enemyTotalValue)            //我方无人机大于敌机和货物总价值,则不撞
            {
                continue ;
            }       
            if(acceptedUAV[i].baseInfo.nZ < (mapHLow * 0.5))      //我方无人机正处于下降捡货状态,则跳过
            {
                continue ;
            }

            //我方无人机起点

            ASPath  ourPath;
            int distance = 0;
            int pathHigh = 0;
            int levelStepCount = 0;

            pathFrom.x = acceptedUAV[i].baseInfo.nX;
            pathFrom.y = acceptedUAV[i].baseInfo.nY;

            SearchShortPath(pathFrom,pathTo,&ourPath,&distance,&pathHigh); 
            levelStepCount = distance - pathHigh * 2;
            ourUavTotalStepNum = levelStepCount + pathHigh + abs(pathHigh - acceptedUAV[i].baseInfo.nZ);

            //计算以上两个变量值
            //还有要判断我方战机是否处于水平位置

            if(ourUavTotalStepNum > enemyUavTotalStepNum)                           //敌人比我方抢先到达目的地,因为我不想只撞空飞机,则匹配下一个无人机
            {
                continue ;
            }           
            
            RoutePlan(&acceptedUAV[i],&ourPath,levelStepCount,pathHigh,pathHigh);         //直接下降到地面          

            //从空闲状态变为敢死队状态
            printf("uavno=%d\n",acceptedUAV[i].baseInfo.nNO);

            //这里需要释放与其匹配的货物
            //释放与其匹配好了的货物
            int matchedNo = acceptedUAV[i].goodsNo;
            int tmpGoodIndex = SearchGoodsNo(matchedGoods,matchedGoodsNum,matchedNo);   //在已经匹配的货物中搜寻索引 
            if(tmpGoodIndex != -1)                                                      //一定要增加该条件,因为可能会出现搜索不到该货物的情形,比如我方无人机匹配该货物时,敌方拾取,而我方为了撞击却没有改变该无人机状态,而该货物状态却已经从匹配中跳到拾取状态下,此时将无法查找到该货物
            {
                InsertUnmatchGoods(&matchedGoods[tmpGoodIndex]);                            //保存到未匹配状态下
                DeleteMatchedGoods(&matchedGoods[tmpGoodIndex]);                            //从已匹配状态下释放出来 
            }  

            InsertDeatSquadsUAV(&acceptedUAV[i]);    
            DeleteAcceptedUAV(&acceptedUAV[i]);  
            //printf("egx=%d egy=%d\n",pathTo.x,pathTo.y);        
            //找到了就直接退出该函数
            return 0;                                       
        }
    }
    return -1;                      //未找到敢死队无人机
}


//当敌人无人机在刚捡货时我方没有可以拦截的无人机,将其保存记录,时刻检测是否可以拦截
//只需要用空闲的拦截即可,因为接受任务的要能够拦截在一开始就拦截了
//这里需要注意一点是,如果拦截者来自家中,则一次只能派出一架,如果能拦截,就不充电
//该模块没有问题
void InterceptionEnemyCarryGoodsUav(void)
{
    int despatchUavFromHomeFlag = 0;                    //从家中派出一台无人机标志
    for(int k=0;k<enemyCarryGoodsUavNum;k++)
    {
        int enemyUavTotalStepNum = 0;
        int ourUavTotalStepNum = 0;

        enemyUavTotalStepNum = enemyCarryGoodsUav[k].nRemainStepNum;       //敌人剩余总步子数

        pathTo.x = enemyCarryGoodsUav[k].nEndX;
        pathTo.y = enemyCarryGoodsUav[k].nEndY;
        int enemyTotalValue = enemyCarryGoodsUav[k].nTotalValue;        //敌人的总价值
        if(idleStatusUAVNum>0)                                          //有空闲的就从空闲的里面选
        {
            for(int i=0;i<idleStatusUAVNum;i++)                                    //遍历满足要求的无人机              
            {
                if(despatchUavFromHomeFlag == 1)
                {
                    if((idleUAV[i].baseInfo.nX == homeLocation.x)&&(idleUAV[i].baseInfo.nY == homeLocation.y)&&(idleUAV[i].baseInfo.nZ == homeLocation.z))
                    {
                        continue;
                    }
                }
                int ourUavValue = 0;
                for(int j=0;j<uavPriceNum;j++)           //计算我方无人机价格
                {
                    if(strcmp(UAVPriceList[j].szType,idleUAV[i].baseInfo.szType)==0)
                    {
                        ourUavValue = UAVPriceList[i].nValue;                 
                        break ;
                    }
                }
                if(ourUavValue > enemyTotalValue)            //我方无人机大于敌机和货物总价值,则不撞
                {
                    continue ;
                }       
                //我方无人机起点

                ASPath  ourPath;
                int distance = 0;
                int pathHigh = 0;
                int levelStepCount = 0;

                pathFrom.x = idleUAV[i].baseInfo.nX;
                pathFrom.y = idleUAV[i].baseInfo.nY;

                SearchShortPath(pathFrom,pathTo,&ourPath,&distance,&pathHigh); 
                levelStepCount = distance - pathHigh * 2;
                ourUavTotalStepNum = levelStepCount + pathHigh + abs(pathHigh - idleUAV[i].baseInfo.nZ);
                if(ourUavTotalStepNum > enemyUavTotalStepNum)                           //敌人比我方抢先到达目的地,因为我不想只撞空飞机,则匹配下一个无人机
                {
                    continue ;
                }           
                if((idleUAV[i].baseInfo.nX == homeLocation.x)&&(idleUAV[i].baseInfo.nY == homeLocation.y)&&(idleUAV[i].baseInfo.nZ == homeLocation.z))
                {
                    //从家中派出一台无人机
                    despatchUavFromHomeFlag = 1;
                }     
                printf("weno=%d enemyno=%d\n",idleUAV[i].baseInfo.nNO,enemyCarryGoodsUav[k].enemyUavnNO);           
                RoutePlan(&idleUAV[i],&ourPath,levelStepCount,pathHigh,pathHigh);         //直接下降到地面

                //从空闲状态变为敢死队状态
                printf("uavno=%d\n",idleUAV[i].baseInfo.nNO);
                InsertDeatSquadsUAV(&idleUAV[i]);    
                DeleteIdleUAV(&idleUAV[i]); 
                //该敌人无人机已被拦截,可将其从数组中删除
                for(int m=k;m<enemyCarryGoodsUavNum;m++)
                {
                    enemyCarryGoodsUav[m] = enemyCarryGoodsUav[m + 1];
                }
                enemyCarryGoodsUavNum --;
                k -= 1;                                    //因为上面for循环删除了一个元素,导致索引往前移动一个单位
                break;
            }
        }
    }
}

//功能:(处理来自非自己更改的状态变化)
//1.无人机是否已被撞毁   
//2.是否有新的无人机出现
//3.是否有新的货物出现 
//4.是否有货物被敌方拾起(则需要根据这个修改我方无人机是状态信息)
void LastStateProcess(MATCH_STATUS * pstMatch)                              //这个函数很复杂,写完这个函数今晚就可以休息了
{
     //无人机分类
    int totalUAVNum = pstMatch->nUavWeNum;                                  //包括撞毁了的无人机
    int totalGoodsNum = pstMatch->nGoodsNum;      
    int tmpIndex=0;
    //无人机是否有损坏
    for(int i=0;i<totalUAVNum;i++)                                           //用连个for循环结构更加清晰
    {
        if(pstMatch->astWeUav[i].nStatus == UAV_CRASH)                      //条件成立说明该无人机已经撞坏
        {
            
            int UAVNo = pstMatch->astWeUav[i].nNO;
            
            tmpIndex = SearchUAVNos(idleUAV,idleStatusUAVNum,UAVNo);
            if(tmpIndex !=-1)                                               //撞毁的是空闲状态下的无人机,则直接将其删除即可
            {    
                crashUAVNum += 1;            
                DeleteIdleUAV(&idleUAV[tmpIndex]);                         
            }
            tmpIndex = SearchUAVNos(acceptedUAV,acceptedStatusUAVNum,UAVNo);
            if(tmpIndex !=-1)                                               //撞毁的是接收到任务状态下的无人机,则将其删除并释放与其匹配的货物
            {
                 crashUAVNum += 1;          
                //释放与其匹配好了的货物
                int matchedNo = acceptedUAV[tmpIndex].goodsNo;
                int tmpGoodIndex = SearchGoodsNo(matchedGoods,matchedGoodsNum,matchedNo);   //在已经匹配的货物中搜寻索引 
                 if(tmpGoodIndex != -1)                                                      //一定要增加该条件,因为可能会出现搜索不到该货物的情形,比如我方无人机匹配该货物时,敌方拾取,而我方为了撞击却没有改变该无人机状态,而该货物状态却已经从匹配中跳到拾取状态下,此时将无法查找到该货物
                {
                    InsertUnmatchGoods(&matchedGoods[tmpGoodIndex]);                            //保存到未匹配状态下
                    DeleteMatchedGoods(&matchedGoods[tmpGoodIndex]);                            //从已匹配状态下释放出来 
                }                  
                DeleteAcceptedUAV(&acceptedUAV[tmpIndex]);                                   //删除无人机,不知道这个地方是否需要加一个取地址符号                
            }
            tmpIndex = SearchUAVNos(executingUAV,executingStatusUAVNum,UAVNo);            //撞毁的是载货飞行的无人机
            if(tmpIndex !=-1)                                                             //撞毁的是载货飞行的无人机,则将无人机和货物都删除
            {
                 crashUAVNum += 1;           
                int matchedNo = executingUAV[tmpIndex].goodsNo;
                int tmpGoodIndex = SearchGoodsNo(pickedGoods,pickedGoodsNum,matchedNo);   //在已经匹配的货物中搜寻索引
                DeletePickedGoods(&pickedGoods[tmpGoodIndex]);                            //从已匹配状态下删除
                DeleteExecutingUAV(&executingUAV[tmpIndex]);                              //删除该正在执行无人机                       
            }
            tmpIndex = SearchUAVNos(deatSquadsUAV,deatSquadsUAVNum,UAVNo); 
            if(tmpIndex !=-1) 
            {
                crashUAVNum += 1;            
                DeleteDeatSquadsUAV(&deatSquadsUAV[tmpIndex]);
            } 
            tmpIndex = SearchUAVNos(trackerUav,trackerUavNum,UAVNo); 
            if(tmpIndex !=-1) 
            {
                crashUAVNum += 1;            
                DeleteTrackerUAV(&trackerUav[tmpIndex]);
            } 
        }
    }    

    //是否有新的无人机增加
    for(int i=0;i<totalUAVNum;i++)
    {
        int UAVNo = pstMatch->astWeUav[i].nNO;
       
        tmpIndex = SearchUAVNos(idleUAV,idleStatusUAVNum,UAVNo);
        if(tmpIndex != -1)                                                                  //如果找到了,只需要更换一下无人机基本信息
        {   
            UAVInfoStorgUpdate(&idleUAV[tmpIndex].baseInfo,pstMatch->astWeUav[i]);
        }
        else
        {
            tmpIndex = SearchUAVNos(acceptedUAV,acceptedStatusUAVNum,UAVNo);
            if(tmpIndex != -1)                                                              //如果找到了,只需要更换一下无人机基本信息
            {
                UAVInfoStorgUpdate(&acceptedUAV[tmpIndex].baseInfo,pstMatch->astWeUav[i]);
            }
            else
            {
                 tmpIndex = SearchUAVNos(executingUAV,executingStatusUAVNum,UAVNo);
                if(tmpIndex != -1)                                                              //如果找到了,只需要更换一下无人机基本信息
                {
                   UAVInfoStorgUpdate(&executingUAV[tmpIndex].baseInfo,pstMatch->astWeUav[i]);
                }
                else                                                                            //否则就是新购买的无人机 将其保存到idle状态数组中
                {
                    int tmpIndex = SearchUAVNos(deatSquadsUAV,deatSquadsUAVNum,UAVNo);
                    if(tmpIndex != -1)
                    {
                        UAVInfoStorgUpdate(&deatSquadsUAV[tmpIndex].baseInfo,pstMatch->astWeUav[i]);
                    }
                    else
                    {
                        int tmpIndex = SearchUAVNos(trackerUav,trackerUavNum,UAVNo);
                        if(tmpIndex != -1)
                        {
                            UAVInfoStorgUpdate(&trackerUav[tmpIndex].baseInfo,pstMatch->astWeUav[i]);
                        }                    
                        else
                        {
                            if(pstMatch->astWeUav[i].nStatus != UAV_CRASH)             //因为撞毁的无人机也会在回传列表中
                            {
                                UAVS newAddUavs; 
                                
                                int uavPriceIndex = -1;
                                for(int j=0;j<uavPriceNum;j++)
                                {
                                    if(strcmp(UAVPriceList[j].szType,pstMatch->astWeUav[i].szType)==0)                //查找对应的价格表
                                    {
                                        uavPriceIndex = j;
                                        break;
                                    }
                                }
                                if(uavPriceIndex == -1)                    //说明价格表里面没有这号无人机,基本上不可能
                                {
                                    printf("this type uav is not in priceList\n");
                                    continue;
                                }
                                newAddUavs.nCharge = UAVPriceList[uavPriceIndex].nCharge;
                                newAddUavs.nCapacity = UAVPriceList[uavPriceIndex].nCapacity;

                                newAddUavs.stepNum = mapHLow;
                                for(int j=0;j<mapHLow;j++)                                //新增加的无人机保持在原地不动     
                                {
                                    newAddUavs.pPath[j].x = pstMatch->astWeUav[i].nX; 
                                    newAddUavs.pPath[j].y = pstMatch->astWeUav[i].nY; 
                                    newAddUavs.pPath[j].z = pstMatch->astWeUav[i].nZ;
                                }                                                      
                                newAddUavs.goodsNo = -1;
                                newAddUavs.pathIndex = 0;
                                UAVInfoStorgUpdate(&newAddUavs.baseInfo,pstMatch->astWeUav[i]);
                                InsertIdleUAV(&newAddUavs);
                            }
                        } 
                    }                                      
                }
            }
        }  
      }
      //是否有货物被拾起   是否有新货物出现  完成这两个功能    
      for(int i=0;i<totalGoodsNum;i++)
      {        
        int GoodsNo = pstMatch->astGoods[i].nNO;
        int tmpGoodIndex=-1;
        //分别在三个列表中搜索,如果找不到说明是新出现的货物,则放到未匹配列表中去,否则根据货物的状态修改相应无人机状态
        tmpGoodIndex = SearchGoodsNo(pickedGoods,pickedGoodsNum,GoodsNo);         //在已经匹配的货物中搜寻索引
        if(tmpGoodIndex != -1)                                                    //成立说明该货物之前处于已被拾起的状态
        {            
            //不做任何处理
        }
        else 
        {
            tmpGoodIndex = SearchGoodsNo(unmatchGoods,unmatchGoodsNum,GoodsNo);          //在未匹配的货物中搜寻索引
            if(tmpGoodIndex != -1)                                                       //成立说明该货物之前就已经存在,并处于未匹配状态下
            {
                //这里要更新货物的剩余时间

                 unmatchGoods[tmpGoodIndex].goods.nLeftTime = pstMatch->astGoods[i].nLeftTime;
               
                 if(pstMatch->astGoods[i].nState == 1 )                                 //成立说明被敌方拾起
                 {
                    //待会儿还要在这里放一个大招
                     //如果被敌人拾取,就要判断是否我方有空闲且价值低于敌方的无人机先一步占领放货的入口
                    int result = -1;
                    /***********************************************************************************************/
                    
                    int enemyCarryGoodsUavNO = -1;
                    int trackedResult = enemyGoodsUavWhethTracked(pstMatch,GoodsNo,&enemyCarryGoodsUavNO);
                    if(trackedResult != -1)
                    {
                        //释放对应的追踪无人机
                        for(int k=0;k<trackerUavNum;k++)
                        {
                            if(enemyCarryGoodsUavNO == trackerUav[k].nEnemyUavNO)
                            {
                                if(trackerUav[k].pPath[trackerUav[k].stepNum -1].z < mapHLow)                       //成立说明可以下降堵
                                {
                                    goto  notOccupy; 
                                }
                                InsertIdleUAV(&trackerUav[k]);
                                DeleteTrackerUAV(&trackerUav[k]);
                            }
                        }
                    }
                    //以上区间模块没有问题
                    /***********************************************************************************************/
                    result = OccupyGoodGoalPoint(pstMatch,GoodsNo);                         //这种情况前期满足要求的概率很小,因为既然有未被匹配的货物,那么我方空闲的无人机就会很少 ,但有了它,我就可以都买小的无人机,最后不考虑捡货,而是撞击                         
                    tmpGoodIndex = SearchGoodsNo(unmatchGoods,unmatchGoodsNum,GoodsNo); 
                    if(result == -1)
                    {
                        //说明没有找到敢死队无人机,需要将敌人该无人机做一个标注
                        for(int i=0;i<pstMatch->nUavEnemyNum ;i++)
                        {
                            if(GoodsNo == pstMatch->astEnemyUav[i].nGoodsNo)                //成立说明找到了载有该货物标号的无人机
                            {
                                int enemyUavValue = 0;
                                int enemyTotalValue = CaculateEnemyTotalValue(pstMatch,unmatchGoods,unmatchGoodsNum,GoodsNo,&enemyUavValue);        //该模块Ok
                                enemyCarryGoodsUav[enemyCarryGoodsUavNum].nTotalValue = enemyTotalValue;                                            //后面会按价值做一个排序
                                enemyCarryGoodsUav[enemyCarryGoodsUavNum].enemyUavnNO = pstMatch->astEnemyUav[i].nNO;
                                enemyCarryGoodsUav[enemyCarryGoodsUavNum].nEndX = unmatchGoods[tmpGoodIndex].goods.nEndX;
                                enemyCarryGoodsUav[enemyCarryGoodsUavNum].nEndY = unmatchGoods[tmpGoodIndex].goods.nEndY;                                
                                enemyCarryGoodsUav[enemyCarryGoodsUavNum].nRemainStepNum = unmatchGoods[tmpGoodIndex].distanceStartToEnd; 
                                enemyCarryGoodsUavNum++;
                                qsort(enemyCarryGoodsUav,enemyCarryGoodsUavNum,sizeof(enemyCarryGoodsUav[0]),EnemyCarryGoodsUAVCompareContent);  
                                break;
                            }
                        }                       
                    }    
                    //以上模块没问题
                    notOccupy:
                    InsertPickedGoods(&unmatchGoods[tmpGoodIndex]);                        //保存到已被拾起状态下
                    DeleteUnmatchGoods(&unmatchGoods[tmpGoodIndex]);                    
                 }                           
            }
            else
            {             
                tmpGoodIndex = SearchGoodsNo(matchedGoods,matchedGoodsNum,GoodsNo);      //在已被匹配的货物中搜寻索引

                if(tmpGoodIndex != -1)                                                   //成立说明之前就已经匹配好了的
                {
                    matchedGoods[tmpGoodIndex].goods.nLeftTime = pstMatch->astGoods[i].nLeftTime;                        //这里也需要更新货物剩余时间,因为被匹配的货物很可能因为敢死队变成未匹配货物,导致后面空闲无人机匹配该货物时,导致剩余时间是该货物被敢死队匹配之前的值             
                    if(pstMatch->astGoods[i].nState == 0 )                              // 未被拾起        
                    {
                        //不作任何处理
                    }
                    else                                                                 //一定是被敌方拾起,如果被我方拾取就不会在匹配的货物中找到,因为一旦被我方拾取就会立刻设置该货物的状态为picked
                    {            
                        
                        //查找我方与该货物匹配到的无人机
                        int matchedUAVNo = matchedGoods[tmpGoodIndex].UAVNO;
                        int tmpUAVindex = SearchUAVNos(acceptedUAV,acceptedStatusUAVNum,matchedUAVNo);                  //按道理来说这个一定能搜索到的
                        int occupyResult = -1;

                        int enemyCarryGoodsUavNO = -1;
                        int trackedResult = enemyGoodsUavWhethTracked(pstMatch,GoodsNo,&enemyCarryGoodsUavNO);
                        if(trackedResult != -1)
                        {
                            //释放对应的追踪无人机
                            for(int k=0;k<trackerUavNum;k++)
                            {
                                if(enemyCarryGoodsUavNO == trackerUav[k].nEnemyUavNO)
                                {
                                    if(trackerUav[k].pPath[trackerUav[k].stepNum -1].z < mapHLow)                       //成立说明可以下降堵
                                    {
                                         if(tmpUAVindex !=-1)
                                         {
                                             InsertIdleUAV(&acceptedUAV[tmpUAVindex]);                              
                                             DeleteAcceptedUAV(&acceptedUAV[tmpUAVindex]);
                                         }                                         
                                         goto  jumpStatusChange; 
                                    }
                                    InsertIdleUAV(&trackerUav[k]);
                                    DeleteTrackerUAV(&trackerUav[k]);
                                    break;
                                }
                            }
                        }
                        if(tmpUAVindex != -1)                                           //修改相应无人机信息
                        {      
                            int changeStatus = WhetherChangeAcceptedToIdle(pstMatch,&acceptedUAV[tmpUAVindex],GoodsNo);            //如果不需要就直接屏蔽掉
                            if(changeStatus == 1)                                                                                  //成立说明不需要改变状态,直接撞毁敌机
                            {
                                goto  jumpStatusChange;                                               //这里
                            }                             
                            InsertIdleUAV(&acceptedUAV[tmpUAVindex]);                              
                            DeleteAcceptedUAV(&acceptedUAV[tmpUAVindex]);                                                                                   
                        }
                        occupyResult = OccupyGoodGoalPoint(pstMatch,GoodsNo);                                                       //这种情况前期满足要求的概率很小,因为既然有未被匹配的货物,那么我方空闲的无人机就会很少 ,但有了它,我就可以都买小的无人机,最后不考虑捡货,而是撞击                         
                        tmpGoodIndex = SearchGoodsNo(matchedGoods,matchedGoodsNum,GoodsNo);  
                        if(occupyResult == -1)
                        {
                            //说明没有找到敢死队无人机,需要将敌人该无人机做一个标注                           
                            for(int i=0;i<pstMatch->nUavEnemyNum ;i++)
                            {
                                if(GoodsNo == pstMatch->astEnemyUav[i].nGoodsNo)                //成立说明找到了载有该货物标号的无人机
                                {
                                    int enemyUavValue = 0;
                                    int enemyTotalValue = CaculateEnemyTotalValue(pstMatch,unmatchGoods,unmatchGoodsNum,GoodsNo,&enemyUavValue);        //该模块Ok
                                    enemyCarryGoodsUav[enemyCarryGoodsUavNum].nTotalValue = enemyTotalValue; 
                                    enemyCarryGoodsUav[enemyCarryGoodsUavNum].enemyUavnNO = pstMatch->astEnemyUav[i].nNO;
                                    enemyCarryGoodsUav[enemyCarryGoodsUavNum].nEndX = matchedGoods[tmpGoodIndex].goods.nEndX;
                                    enemyCarryGoodsUav[enemyCarryGoodsUavNum].nEndY = matchedGoods[tmpGoodIndex].goods.nEndY;
                                    enemyCarryGoodsUav[enemyCarryGoodsUavNum].nRemainStepNum = matchedGoods[tmpGoodIndex].distanceStartToEnd;
                                    enemyCarryGoodsUavNum++;                                    
                                }
                            }                       
                        }    
                        //以上模块没问题
                        jumpStatusChange:                                                        //货物状态必须更改,如果不更改该货物状态的话,后面会因为检查到我方无人机被撞毁后,该货物状态会由匹配状态下改为未匹配状态
                        InsertPickedGoods(&matchedGoods[tmpGoodIndex]);
                        DeleteMatchedGoods(&matchedGoods[tmpGoodIndex]);                                                                            
                    }
                }
                else             
                {                        
                    //该货物是新出现的货物,将其存入未匹配状态中
                    if(goodsDetecMap[pstMatch->astGoods[i].nStartX][pstMatch->astGoods[i].nStartY] != 0)
                    {
                        continue;
                    }
                    if(goodsDetecMap[pstMatch->astGoods[i].nEndX][pstMatch->astGoods[i].nEndY] != 0)
                    {
                        continue;
                    }          

                    GOODS_STATE newAddGoods;
                    
                    GoodsInfoStorge(&newAddGoods.goods,pstMatch->astGoods[i]);

                    pathFrom.x = newAddGoods.goods.nStartX;
                    pathFrom.y = newAddGoods.goods.nStartY;  

                    pathTo.x = newAddGoods.goods.nEndX;
                    pathTo.y = newAddGoods.goods.nEndY;   
                    
                    ASPath path;    
                    int distance = 0;
                    int pathHigh = 0;
                    SearchShortPath(pathFrom,pathTo,&path,&distance,&pathHigh);               
                    newAddGoods.pathStartToEnd = path;
                    newAddGoods.distanceStartToEnd = distance;
                    newAddGoods.pathHigh = pathHigh; 
                    newAddGoods.levelStepNum = distance - (pathHigh * 2); 
                    newAddGoods.needCapacity = (distance + 1) * newAddGoods.goods.nWeight;             //所需要的电量     
                    InsertUnmatchGoods(&newAddGoods);                                            //保存到未匹配状态下
                }
            }
        }
      }
      //更新敌人被追踪的初始无人机状态
      //该模块没有问题
      for(int i=0;i<trackerUavNum;i++)
      {
          int enemyUavnNO = trackerUav[i].nEnemyUavNO;                                         //被追踪的敌人无人机编号
          int tmpUAVindex = SearchEnemyUAVNos(pstMatch->astEnemyUav,pstMatch->nUavEnemyNum,enemyUavnNO);
          if(tmpUAVindex == -1)
          {
                DeleteEnemyUAV(&initEnemyUav[enemyUavnNO]);                             //删除敌人该标号无人机
                InsertIdleUAV(&trackerUav[i]);
                DeleteTrackerUAV(&trackerUav[i]); 
                i -= 1;
          }
          else 
          {
              int initTmpUavIndex = SearchEnemyUAVNos(initEnemyUav,initEnemyUavNum,enemyUavnNO);  
              if(initTmpUavIndex != -1)
              {
                    if(pstMatch->astEnemyUav[tmpUAVindex].nStatus == UAV_FOG)
                    {
                        initEnemyUav[initTmpUavIndex].nStatus = UAV_FOG;                             //保留敌人进入雾区前一时刻的坐标    
                        initEnemyUav[initTmpUavIndex].nGoodsNo = pstMatch->astEnemyUav[tmpUAVindex].nGoodsNo;  //时时更新其货物状态       
                    }
                    else
                    {
                        initEnemyUav[initTmpUavIndex] = pstMatch->astEnemyUav[tmpUAVindex];  
                    }   
              }           
          }
      }
      //敌人载货无人机状态处理
      //该模块OK
      for(int i=0;i<enemyCarryGoodsUavNum;i++)                                        //在拦截之前还要判断一下对应的敌人无人机是否已经撞毁,直接在这里处理吧
      {
          if(enemyCarryGoodsUav[i].nRemainStepNum > 0)
          {
              enemyCarryGoodsUav[i].nRemainStepNum -= 1;                              //敌人载货无人机抵达货物目的地的剩余步子数,0代表货物已投放成功
          }          
          printf("num=%d\n",enemyCarryGoodsUavNum);
          printf("no=%d totalValue=%d x=%d y=%d rstepnum=%d\n",enemyCarryGoodsUav[i].enemyUavnNO,enemyCarryGoodsUav[i].nTotalValue,enemyCarryGoodsUav[i].nEndX,enemyCarryGoodsUav[i].nEndY,enemyCarryGoodsUav[i].nRemainStepNum);
          int enemyUavnNO = enemyCarryGoodsUav[i].enemyUavnNO;
          int tmpUAVindex = SearchEnemyUAVNos(pstMatch->astEnemyUav,pstMatch->nUavEnemyNum,enemyUavnNO);          
          
          if((enemyCarryGoodsUav[i].nRemainStepNum < mapHLow)||(tmpUAVindex == -1))  //删除掉小于最低飞行高度的或者已经撞毁的
          {
              for(int k = i;k<enemyCarryGoodsUavNum;k++)
              {
                  enemyCarryGoodsUav[k] = enemyCarryGoodsUav[k + 1];
              }
              enemyCarryGoodsUavNum --;
              i -= 1;
              continue;
          }
      }
}

//功能:
//判断家中共有多少架飞机
int  countUavNumHome(void)
{
    int countNum = 0;
    for(int i=0;i<idleStatusUAVNum;i++)
    {
        if((idleUAV[i].baseInfo.nX == homeLocation.x)&&(idleUAV[i].baseInfo.nY == homeLocation.y)&&(idleUAV[i].baseInfo.nZ == homeLocation.z))
        {
            countNum += 1;
        }
    }
    return countNum;
}

//功能:一开局用小无人机撞毁敌人大无人机
//将该无人机定义为追踪状态,不考虑碰撞问题,判定状态为我方最小的无人机一定要大于或等于敌人最大的无人机
//该模块没有问题
void InitTrackerUav(void)             //该函数在初始状态调用完后调用
{   
    static int delayStepNum = 0;
    if(delayStepNum < (mapHLow - 1))
    {
        delayStepNum++;
        return ;
    }
    static int exectFlag = 0;
    if(exectFlag == 1)
    {        
        return;
    }
    exectFlag = 1;
   
    for(int j=1,i=0;i < idleStatusUAVNum;j++,i++)
    {
        if(idleUAV[i].baseInfo.nLoadWeight < initEnemyUav[initEnemyUavNum - j].nLoadWeight)
        {
            
            idleUAV[i].nEnemyUavNO = initEnemyUav[initEnemyUavNum - j].nNO;
            printf("w1=%d w2=%d\n",idleUAV[i].baseInfo.nLoadWeight,initEnemyUav[initEnemyUavNum - j].nLoadWeight);
            printf("no1=%d no2=%d\n",idleUAV[i].nNO,initEnemyUav[initEnemyUavNum - j].nNO);
            InsertTrackerUAV(&idleUAV[i]);                                                  //初始时他们还是空闲状态下设置的路径
            DeleteIdleUAV(&idleUAV[i]);
            i -= 1;
        }
    }
   
}



//功能:更新追踪者信息
//只需要根据敌人初始无人机状态来处理即可
//如果敌人无人机飞行高度不低于最低飞行高度,则不考虑敌人无人机所在高度情况
//该模块没问题
void UpdateTrakerUav(MATCH_STATUS * pstMatch)
{
    //为避免碰撞问题,首先需要将其上升至最低飞行高度上两个单位
    //当其最低飞行高度低于最低飞行高度时,让其保持上一时刻设定的路径飞行
    for(int i=0;i<trackerUavNum;i++)
    {
        if(trackerUav[i].baseInfo.nZ < mapHLow)                 //低于最低飞行高度只会有两种情况:1.从家的位置飞出时在低于最低飞行高度时就设置为了追踪者状态.2.可以堵住敌人飞机
        {
            continue;
        }
        //对其状态进行更新       
        ASPath Path;       
        int distance = 0;
        int pathHigh = 0;
        int levelStepCount = 0;

        pathFrom.x = trackerUav[i].baseInfo.nX;
        pathFrom.y = trackerUav[i].baseInfo.nY;
        int tmpUAVindex = SearchEnemyUAVNos(initEnemyUav,initEnemyUavNum,trackerUav[i].nEnemyUavNO);
        if(tmpUAVindex == -1)                           //说明该无人机已经被撞毁了
        {
            //需要将该追踪者转为空闲状态
            InsertIdleUAV(&trackerUav[i]);
            DeleteTrackerUAV(&trackerUav[i]);
            return;
        }
        pathTo.x = initEnemyUav[tmpUAVindex].nX;                                     //当该飞机进入雾区时,该坐标为其进入雾区前一时刻的坐标,在那里等着吧
        pathTo.y = initEnemyUav[tmpUAVindex].nY;

        SearchShortPath(pathFrom,pathTo,&Path,&distance,&pathHigh); 
        levelStepCount = distance - pathHigh * 2;
        int stopHigh=0;
        if(initEnemyUav[tmpUAVindex].nZ < mapHLow)                                  //敌机开始捡货,如果敌人在雾区,则保留的是其进入雾区前一时刻的位置
        {            
            //这里需要判断追踪者是否来得及在货物出口堵住,来不及就判断去往货物的终点是否来的及
            int enemyUavReachMapHlowStepNum = mapHLow + initEnemyUav[tmpUAVindex].nZ;            //敌人到达出口剩余步子数
            int weUavReachMapHlowStepNum = levelStepCount + (trackerUav[i].baseInfo.nZ - mapHLow);
            if(weUavReachMapHlowStepNum <= enemyUavReachMapHlowStepNum)                           //如果能提前到达,则下降到地面
            {
                 stopHigh = pathHigh;
            }                  
            //如果来不及堵,则继续靠近敌机,当其捡起货物时会作相应的处理的,不用担心
        }      
        RoutePlan(&trackerUav[i],&Path,levelStepCount,stopHigh,pathHigh); 
        int totalStepNum = trackerUav[i].stepNum;
        trackerUav[i].pPath[totalStepNum] = trackerUav[i].pPath[totalStepNum-1];
        trackerUav[i].stepNum += 1; 
        
        //在追踪者下降之前需要判断一下下降点是否有我方无人机,有的话则守在出口等着:不管我方无人机是在敌方无人机上面还是下面
        if(trackerUav[i].pPath[trackerUav[i].pathIndex].z < mapHLow)                                     //成立说明我方追踪者准备下降堵敌人
        {
            for(int k = 0;k<pstMatch->nUavWeNum;k++)
            {
                if((pstMatch->astWeUav[k].nX == trackerUav[i].pPath[trackerUav[i].pathIndex].x)&&(pstMatch->astWeUav[k].nY == trackerUav[i].pPath[trackerUav[i].pathIndex].y)&&(pstMatch->astWeUav[k].nZ < mapHLow))//仅凭X和Y是不行的,因为会查询到自己
                {
                    if(pstMatch->astWeUav[k].nNO == trackerUav[i].baseInfo.nNO)
                    {
                        continue;
                    }
                    if(pstMatch->astWeUav[k].nZ == mapHLow-1)                   //说明要发生碰撞
                    {
                        trackerUav[i].pathIndex -= 1;
                        trackerUav[i].pPath[trackerUav[i].pathIndex].z += 1;
                    }
                    else
                    {
                        trackerUav[i].pathIndex -= 1;
                    }
                    
                }
            }
        }
    }
}

//根据无人机和货物状态分类存到不同的数组中去           
void ProcessState(MATCH_STATUS * pstMatch)   //
{
    static int firstExectFlag = 0;
    if(!firstExectFlag)                      //条件成立说明是第一次执行
    {
        FirstStateProcess(pstMatch);
        firstExectFlag =1;
    }
    else                                     //根据返回的状态信息调整三种无人机状态数组
    {
        LastStateProcess(pstMatch);
    }    
}


typedef struct _GOODSELECT_
{
    int goodsNO;
    float rateOfValueAndDistance;
    ASPath path;
    int pathStepNum;                             //路径总步子数
    int levelStepNum;                            //水平步子数
    int pathHigh;                                //最短路径所在高度
} GOODSELECT;

GOODSELECT   saGoodsSelect[MAX_GOODS_NUM];
int saGoodsSelectIndex = 0;

//无人机匹配到目标
void setMatchedInfo(UAVS *uavs,GOODS_STATE *goodsState,GOODSELECT *goodselect)    //此函数在无人机匹配到目标后调用
{

    //直接用一个函数处理
    RoutePlan(uavs,&goodselect->path,goodselect->levelStepNum,goodselect->pathHigh,goodselect->pathHigh);     //捡货时需要下降到地面,所以后两个参数相等  
    
    //uavs->pathIndex = 0;   
    uavs->goodsNo = goodsState->goods.nNO;                    //保存货物编号
    uavs->goodsWeight = goodsState->goods.nWeight;
    uavs->starGgoalPoint.x = goodsState->goods.nStartX;      //货物起点
    uavs->starGgoalPoint.y = goodsState->goods.nStartY;
    uavs->starGgoalPoint.z = 0;                              //货物起始点始终在地上
    uavs->endGgoalPoint.x = goodsState->goods.nEndX;         //货物终点
    uavs->endGgoalPoint.y = goodsState->goods.nEndY;    
    uavs->endGgoalPoint.z = 0;                               //货物目的点始终在地上
    
    goodsState->UAVNO = uavs->nNO;                       //记录匹配到的无人机编号,货物的无人机编号只在这个地方   

    InsertMatchedGoods(goodsState); 
    
    DeleteUnmatchGoods(goodsState);        
    
    InsertAcceptedUAV(uavs);  
    
    //无人机状态设置
    DeleteIdleUAV(uavs);  

    //货物状态设置 
}




//此函数被执行前需要用if判断无人机确实拾取到货物,不然此无人机就废了
//无人机拾起到货物
//函数功能
//1.判断无人机是否已经到捡货地点
//2.如果已经到了捡货地点,则需要重新规划路径
//此处只需要对接收到任务的无人机进行判断
void TaskExecteInit(void)                                              //无人机拾起到货物后准备去往目的地前的初始化,一般为状态信息变换和路径规划
{
    if(acceptedStatusUAVNum == 0)                          //没有接收到任务的无人机,所以下一步肯定不会出现拾起到货物的情况
    {
        return ;
    }        
    for(int i=0;i<acceptedStatusUAVNum;i++)
    {
        int goodSGX = acceptedUAV[i].starGgoalPoint.x;
        int goodSGY = acceptedUAV[i].starGgoalPoint.y;        
        int UAVCurrLX = acceptedUAV[i].baseInfo.nX;
        int UAVCurrLY = acceptedUAV[i].baseInfo.nY;
        int UAVCurrLZ = acceptedUAV[i].baseInfo.nZ;          
       
        if((goodSGX==UAVCurrLX)&&(goodSGY==UAVCurrLY)&&(0 == UAVCurrLZ-1))                   // 
        {//货物被拾起,在捡货之前应先检查货物是否处于拾取状态                     

             int tmpGoodIndex = SearchGoodsNo(matchedGoods,matchedGoodsNum,acceptedUAV[i].goodsNo); 
             if(tmpGoodIndex != -1)
             {
                 if(matchedGoods[tmpGoodIndex].goods.nState == 0)                             //成立说明货物未被拾取
                 {
                     acceptedUAV[i].baseInfo.nGoodsNo = acceptedUAV[i].goodsNo;  
                     acceptedUAV[i].baseInfo.nRemainElectricity -=  acceptedUAV[i].goodsWeight;

                     InsertPickedGoods(&matchedGoods[tmpGoodIndex]);                                 //在货物的状态之前先保存一些数据
                     DeleteMatchedGoods(&matchedGoods[tmpGoodIndex]);
                 }                  
             }                     
        }                
        if((goodSGX==UAVCurrLX)&&(goodSGY==UAVCurrLY)&&(0==UAVCurrLZ))                   //成立说明已经到达取货点,则需要设置无人机相应的状态参数  
        {         
            //修改无人机状态信息            
             int tmpGoodIndex = SearchGoodsNo(pickedGoods,pickedGoodsNum,acceptedUAV[i].goodsNo);  //查找与之对应的货物
             if(tmpGoodIndex != -1)
             {                
                 //货物需要放到终点,所以后两个参数相等
                 RoutePlan(&acceptedUAV[i],&pickedGoods[tmpGoodIndex].pathStartToEnd,pickedGoods[tmpGoodIndex].levelStepNum,pickedGoods[tmpGoodIndex].pathHigh,pickedGoods[tmpGoodIndex].pathHigh);
                 acceptedUAV[i].taskState = UAV_TASK_EXECUTING;   
                
                //这里还少一个对相应货物状态修改的部分

               
                InsertExecutingUAV(&acceptedUAV[i]);            
                DeleteAcceptedUAV(&acceptedUAV[i]);                                                
                i = i -1;  
             }                                                                                      //找这个bug花了一下午时间
        }
        
    }  
}

//任务完成函数检测
//检测所有运货飞行的无人机是否已达目的地
//根据结果设置相应无人机的状态模型
void FinishTaskDetec(void)
{
    if(executingStatusUAVNum == 0)                          //没有正在载货的无人机,所以下一步肯定不会出现货物运达的情况
    {
        return ;
    }    
    for(int i=0;i<executingStatusUAVNum;i++)
    {        
        int goodEGX = executingUAV[i].endGgoalPoint.x;
        int goodEGY = executingUAV[i].endGgoalPoint.y;
        int UAVCurrLX = executingUAV[i].baseInfo.nX;
        int UAVCurrLY = executingUAV[i].baseInfo.nY;
        int UAVCurrLZ = executingUAV[i].baseInfo.nZ;

        if((goodEGX==UAVCurrLX)&&(goodEGY==UAVCurrLY)&&(0==UAVCurrLZ))     //成立说明已经到达货物存放点,则需要设置无人机相应的状态参数 
        {            
            //修改无人机状态信息            
            int tmpGoodsNO = executingUAV[i].goodsNo;                             //记录下无人机匹配到的货物编号
            int initHigh = executingUAV[i].baseInfo.nZ;  

            executingUAV[i].pPath[0].x = executingUAV[i].baseInfo.nX;               
            executingUAV[i].pPath[0].y = executingUAV[i].baseInfo.nY;
            executingUAV[i].pPath[0].z = executingUAV[i].baseInfo.nZ;
            executingUAV[i].pathIndex = 1;  
            executingUAV[i].stepNum = mapHLow + 1; 
            for(int j=1;j<=mapHLow;j++)
            {
                executingUAV[i].pPath[j].x = executingUAV[i].baseInfo.nX;               
                executingUAV[i].pPath[j].y = executingUAV[i].baseInfo.nY; 
                initHigh += 1;               
                executingUAV[i].pPath[j].z = initHigh; 
            }                            
            executingUAV[i].pPath[executingUAV[i].stepNum].x = executingUAV[i].pPath[executingUAV[i].stepNum - 1].x;
            executingUAV[i].pPath[executingUAV[i].stepNum].y = executingUAV[i].pPath[executingUAV[i].stepNum - 1].y;
            executingUAV[i].pPath[executingUAV[i].stepNum].z = executingUAV[i].pPath[executingUAV[i].stepNum - 1].z;
            executingUAV[i].stepNum += 1;
            InsertIdleUAV(&executingUAV[i]);
            DeleteExecutingUAV(&executingUAV[i]);  
            //货物已被拾起,则需要修改货物信息
            int tmpGoodsIndex = SearchGoodsNo(pickedGoods,pickedGoodsNum,tmpGoodsNO);            
            DeletePickedGoods(&pickedGoods[tmpGoodsIndex]);
            i = i - 1;
        }
    }
}

//随机函数生成的目标点
PathNode randTargtePoint[MAX_UAV_NUM];
int randTargteIndex = 0;
//功能:查询是否有相同目标点
// -1 :没有相同的,0:有相同的
int SerachRandTargetPoint(int x ,int y)
{
    for(int i=0;i<randTargteIndex;i++)
    {
        if((randTargtePoint[i].x == x)&&(randTargtePoint[i].y == y))
        {
            return 0;
        }
    }
    return -1;
}


//功能:将家中空闲的无人机分布到地图的各个点中
//该函数一次循环只能调度一架无人机移动
//首先的目标点为货物终点(起点好还是终点好呢?)
//选哪一类货物的终点的(匹配的货物终点吧:相当于代替了清夫的功能和敢死队两者的功能)
//新加更改还未调试
void ScatterHomeIdleUav(void)
{
    if(sendUavFromHomeFlag == 1)                          //成立说明重家中派遣了一架无人机执行任务,那么就不能再次移动家中无人机了,否则会和任务无人机撞击
    {
        sendUavFromHomeFlag = 0;
        return ;
    }
    int homeUavNum = countUavNumHome();                   //计算家中是否有空闲无人机
    
    if(homeUavNum == 0)                                   //家中没有空闲无人机,直接退出
    {
        return ;
    }
   
    //采用随机函数生成目标点                                 //目标点有几个非法点 1.不能在障碍物中,2.不能是家的位置,3.不能产生重复的点
    int targetPointX = 0;
    int targetPointY = 0;
    int createTargetPointFlag = 0;
    for(int i=0;i<100;i++)                               //100次如果都无法产生一个合法的目标点的话,就直接退出
    {
        if(i < matchedGoodsNum)                             //首先从匹配的货物中找寻目标点
        {
            //待调试
            targetPointX = matchedGoods[i].goods.nEndX;
            targetPointY = matchedGoods[i].goods.nEndY;
        }
        else
        {
            targetPointX = (int)(rand()% mapWidth-1);               //产生0到mapWidth之间的整数
            targetPointY = (int)(rand()% mapLength-1); 
        }
        

        int appearBefore = SerachRandTargetPoint(targetPointX,targetPointY);
        if(appearBefore == 0)                                   //说明之前有产生相同的目标点,跳过该点
        {
            continue ;
        }

        if((targetPointX == homeLocation.x)&&(targetPointY == homeLocation.y))    //目标点为家中的位置,则跳过重新找
        {
            continue ;
        }
        if(threeDMap[mapHLow][targetPointX][targetPointY] != 0)                   //目标点在建筑物中
        {
            continue ;
        }
        
        createTargetPointFlag = 1;                           //到达这里说明生成了目标点
        break ;
    }

    if(createTargetPointFlag == 0)                          //说明前面循环了100次都没有生产合法的目标点
    {
        return ;
    }
    //目标点已经生成
    
    pathTo.x = targetPointX;
    pathTo.y = targetPointY;
    
    int startPointX = 0;
    int startPointY = 0; 
    //找到在家中空闲无人机
    int IdleUavHomeIndex = -1;
    for(int i = 0;i<idleStatusUAVNum;i++)
    {          
        if((idleUAV[i].baseInfo.nX == homeLocation.x)&&(idleUAV[i].baseInfo.nY == homeLocation.y)&&(idleUAV[i].baseInfo.nZ == homeLocation.z))  //成立说明该无人机在家中,可以设置起飞
        {
            if(idleUAV[i].baseInfo.nRemainElectricity == idleUAV[i].nCapacity)                  //成立表示充电完成
            {                
                startPointX = idleUAV[i].baseInfo.nX;
                startPointY = idleUAV[i].baseInfo.nY;
                IdleUavHomeIndex = i;                     //找到了在家中空闲无人机
                break;
            }  
        }          
        
    }
    if(IdleUavHomeIndex == -1)                           //未找到在家中的空闲无人机
    {
        return ;
    }        

    pathFrom.x = startPointX;
    pathFrom.y = startPointY;

    ASPath Path;       
    int distance = 0;
    int pathHigh = 0;
    int levelStepCount = 0;

    pathFrom.x = idleUAV[IdleUavHomeIndex].baseInfo.nX;
    pathFrom.y = idleUAV[IdleUavHomeIndex].baseInfo.nY;

    SearchShortPath(pathFrom,pathTo,&Path,&distance,&pathHigh); 
    levelStepCount = distance - pathHigh * 2;

    int stopHigh = pathHigh - mapHLow;                                              //下降到最低飞行高度

    RoutePlan(&idleUAV[IdleUavHomeIndex],&Path,levelStepCount,stopHigh,pathHigh);  

    int totalStepNum =  idleUAV[IdleUavHomeIndex].stepNum;
    idleUAV[IdleUavHomeIndex].pPath[totalStepNum] = idleUAV[IdleUavHomeIndex].pPath[totalStepNum-1];
    idleUAV[IdleUavHomeIndex].stepNum += 1;

    randTargtePoint[randTargteIndex].x = targetPointX;                //保存为已设置目标点
    randTargtePoint[randTargteIndex].y = targetPointY;
    randTargteIndex++;    
}



#define VALUEWEIGTH 1
#define STARTTOENDWEIGTH 1

//货物排序函数
int saGoodsCompareContent(const void *data1, const void *data2)  
{
    GOODSELECT *p1 = (GOODSELECT *)data1;
    GOODSELECT *p2 = (GOODSELECT *)data2;
    return (p2->rateOfValueAndDistance > p1->rateOfValueAndDistance) ? 1 : -1;                //由大到小排序
} 


//最难最复杂的函数
void TaskSchedule(void)                          //任务分配函数
{        
    static int delayStepNum = 0;
    if(delayStepNum < mapHLow)
    {
        delayStepNum++;
        return ;
    }
    int goodsLeftTime;
    int arriveGoodsStartPointStepNum = 0;        //到达货物点的总步子数
    int getGoodsValueTotalStepNum = 0;           //获得货物价值总步子数
    int GoodIndex = -1;  

    for(int i=0;i<idleStatusUAVNum;i++)                                 //对每台空闲无人机均遍历所有货物
    {      
        if(idleUAV[i].baseInfo.nStatus == UAV_CHARGE)                   //无人机正在充电,家里的无人机都要等其飞出来后再分配任务
        {
            continue ;
        }        
        for(int j=0;j<unmatchGoodsNum;j++)                              //遍历所有货物
        {
            if(idleUAV[i].baseInfo.nLoadWeight < unmatchGoods[j].goods.nWeight)       //无人机载重小于该货物
            {
                continue ;
            }
            if(idleUAV[i].baseInfo.nRemainElectricity < unmatchGoods[j].needCapacity)     //成立说明距离太远,电量不够用
            {
                continue ;
            }

            pathFrom.x = idleUAV[i].baseInfo.nX;
            pathFrom.y = idleUAV[i].baseInfo.nY;  

            pathTo.x = unmatchGoods[j].goods.nStartX;
            pathTo.y = unmatchGoods[j].goods.nStartY;
            goodsLeftTime = unmatchGoods[j].goods.nLeftTime;                     


            ASPath path;    
            int distance = 0;
            int pathHigh = 0;
            //在三维地图中寻找一条最短路
            SearchShortPath(pathFrom,pathTo,&path,&distance,&pathHigh);                                   //此处计算的distance是地面到地面的距离  
            if(idleUAV[i].baseInfo.nZ <= pathHigh)
            {
                arriveGoodsStartPointStepNum = distance - idleUAV[i].baseInfo.nZ;
            }
            else
            {                
                arriveGoodsStartPointStepNum = distance - pathHigh + (idleUAV[i].baseInfo.nZ - pathHigh);                
            }          
            if(arriveGoodsStartPointStepNum < goodsLeftTime)                     //这个减1是不确定,可能存在的一个bug是我正好到那里准备取货时,货物消失了
            {
                getGoodsValueTotalStepNum = arriveGoodsStartPointStepNum + (unmatchGoods[j].distanceStartToEnd * STARTTOENDWEIGTH);
                saGoodsSelect[saGoodsSelectIndex].goodsNO = unmatchGoods[j].goods.nNO;             
                saGoodsSelect[saGoodsSelectIndex].rateOfValueAndDistance =((float)(unmatchGoods[j].goods.nValue*VALUEWEIGTH)/(float)getGoodsValueTotalStepNum);         //          
                saGoodsSelect[saGoodsSelectIndex].path = path;                    //最短路
                saGoodsSelect[saGoodsSelectIndex].pathStepNum = arriveGoodsStartPointStepNum;           //到达货物点的总步子数
                saGoodsSelect[saGoodsSelectIndex].levelStepNum = distance - (pathHigh * 2);
                saGoodsSelect[saGoodsSelectIndex].pathHigh = pathHigh;            //最短路所在高度
                saGoodsSelectIndex++;
            }                
        }
        
        if(saGoodsSelectIndex == 0)              //
        {
            continue;
        }
    
        qsort(saGoodsSelect,saGoodsSelectIndex,sizeof(saGoodsSelect[0]),saGoodsCompareContent);    //
        saGoodsSelectIndex = 0;
        int goodsNO = saGoodsSelect[0].goodsNO;

        GoodIndex = SearchGoodsNo(unmatchGoods,unmatchGoodsNum,goodsNO);                          //在已经匹配的货物中搜寻索引   
        if(GoodIndex == -1)
        {
            continue;
        }               

        int returnFlag = 0;
        int numUavHome = countUavNumHome();                //统计家中共有多少架飞机
        if((idleUAV[i].baseInfo.nX == homeLocation.x)&&(idleUAV[i].baseInfo.nY == homeLocation.y)&&(idleUAV[i].baseInfo.nZ == homeLocation.z))
        {
            sendUavFromHomeFlag = 1;                       //说明从家中派遣了一架无人机去执行任务

            if(numUavHome>1)                               //当飞机在家的时候且家中不止一架飞机时,一次只能调度一架飞机执行任务,否则会出现撞击现象 
            {
                returnFlag = 1;
            }
        }         
        setMatchedInfo(&idleUAV[i],&unmatchGoods[GoodIndex],&saGoodsSelect[0]);    //无人机  货物  路径信息
        i -= 1;                          //每次成功分配一台空闲无人机后,后面的无人机所在索引都会往前移动一个单位
        if(returnFlag)                   //成立说明家中不止一架飞机
        {
            returnFlag=0;
            return ;
        }  
    }        
}

//功能: 碰撞检测函数
// 返回值: 1:有碰撞  0:无碰撞
//碰撞有三种情况:1. 同一时刻到达同一个位置 2.同一时刻两者交换位置 交换位置包括两种:垂直交换和水平交换(水平交换又包括两种:平行交换和对角交换) 3.对角线相交飞行
int DetectCrash(const PATH *A,int indexA,const PATH *B,int indexB)
{    
    
    if((indexA ==0)||(indexB == 0))            //追踪者可能会出现索引为0的情况
    {
        return 0;
    }

    if((A[indexA].x == B[indexB].x)&&(A[indexA].y == B[indexB].y)&&(A[indexA].z == B[indexB].z))                    //到达同一位置
    {
        return 1;
    }      
    //交换位置  
    if((A[indexA-1].x == B[indexB].x) && (A[indexA-1].y == B[indexB].y) && (A[indexA-1].z == B[indexB].z)   &&   (A[indexA].x == B[indexB-1].x) && (A[indexA].y == B[indexB-1].y) && (A[indexA].z == B[indexB-1].z))      
    {
        return 1;
    }
   
    //对角线相交飞行,只有在统一高度才会出现这种情况
    if(A[indexA].z == B[indexB].z)
    {
        //相交会有四种情况
        //情况1
        if( (A[indexA-1].x == (B[indexB-1].x - 1)) && (A[indexA-1].y == B[indexB-1].y)  &&  (A[indexA].x == (B[indexB].x + 1)) && (A[indexA].y == B[indexB].y))
        {
            return 1;
        }        
        //情况2
        if((A[indexA-1].x == B[indexB-1].x) && (A[indexA-1].y == (B[indexB-1].y - 1))  &&  (A[indexA].x == B[indexB].x) && (A[indexA].y == (B[indexB].y + 1)))
        {
            return 1;
        }
        //情况3
        if( (A[indexA-1].x == B[indexB-1].x) && (A[indexA-1].y == (B[indexB-1].y + 1))  &&  (A[indexA].x == B[indexB].x) && (A[indexA].y == (B[indexB].y - 1)))
        {
            return 1;
        }
        //情况4
        if( (A[indexA-1].x == (B[indexB-1].x + 1)) && (A[indexA-1].y == B[indexB-1].y)  &&  (A[indexA].x == (B[indexB].x - 1)) && (A[indexA].y == B[indexB].y))
        {
            return 1;
        }       
    }   
    return 0;
}


//功能:路径改变函数
//加一个最高飞行高度限制,万一出现这种情况就直接输了
void ChangeRode(PATH * path, int* pathIndex,int remainLevelStepNum,int *totalStepNum,UAV_TASK_STATUS Status,int crashType)
{
    int tmpIndex = *pathIndex;
    int tmpTotalStepNum = *totalStepNum;   

    if(Status == UAV_TASK_IDLE)                         //空闲状态下的无人机,直接让其上升一个高度
    {       
        if(path[*pathIndex-1].z + 1 > mapHHigh)           //说明飞机当前正处于最高高度飞行,不能再加1了,否则违规直接输
        {
            return ;
        }   
        if(*pathIndex == (*totalStepNum -1))                //成立说明空闲飞机处于最低飞行高度,可放心加1,否则如果在上升阶段出现碰撞,就直接让其相撞吧,因为要处理这种情况会很复杂
        { //静止的空闲飞机  
            if(crashType == 1)                                //碰撞类型为垂直
            {
                //水平移动一个单位
                if(path[*pathIndex-1].z < mapHLow)            //当前高度小于最低飞行高度,则保持在原地不动,这种的只会发生一次碰撞
                {
                    if(*pathIndex > 0)                        //以防万一
                    *pathIndex -= 1;
                    return;
                }
                if(path[*pathIndex-1].z > mapHLow) 
                {
                    return ;
                }
                if(threeDMap[mapHLow][path[*pathIndex-1].x][path[*pathIndex-1].y+1]==0)
                {
                    
                    path[*pathIndex].x = path[*pathIndex-1].x;
                    path[*pathIndex].y = path[*pathIndex-1].y+1;
                    *totalStepNum += 1;
                    path[*pathIndex+1].x = path[*pathIndex].x;
                    path[*pathIndex+1].y = path[*pathIndex].y;
                    path[*pathIndex+1].z = path[*pathIndex].z;
                    return;
                }
                if(threeDMap[mapHLow][path[*pathIndex-1].x+1][path[*pathIndex-1].y]==0)
                {
                    path[*pathIndex].x = path[*pathIndex-1].x+1;
                    path[*pathIndex].y = path[*pathIndex-1].y;
                    *totalStepNum += 1;
                    path[*pathIndex+1].x = path[*pathIndex].x;
                    path[*pathIndex+1].y = path[*pathIndex].y;
                    path[*pathIndex+1].z = path[*pathIndex].z;
                    return;
                }
                if(threeDMap[mapHLow][path[*pathIndex-1].x+1][path[*pathIndex-1].y+1]==0)
                {
                    path[*pathIndex].x = path[*pathIndex-1].x+1;
                    path[*pathIndex].y = path[*pathIndex-1].y+1;
                    *totalStepNum += 1;
                    path[*pathIndex+1].x = path[*pathIndex].x;
                    path[*pathIndex+1].y = path[*pathIndex].y;
                    path[*pathIndex+1].z = path[*pathIndex].z;
                    return;
                }
                if(threeDMap[mapHLow][path[*pathIndex-1].x][path[*pathIndex-1].y-1]==0)
                {
                    if((path[*pathIndex-1].y-1)>=0)
                    {
                        path[*pathIndex].x = path[*pathIndex-1].x;
                        path[*pathIndex].y = path[*pathIndex-1].y-1;
                        *totalStepNum += 1;
                        path[*pathIndex+1].x = path[*pathIndex].x;
                        path[*pathIndex+1].y = path[*pathIndex].y;
                        path[*pathIndex+1].z = path[*pathIndex].z;
                        return;
                    }                    
                }
                if(threeDMap[mapHLow][path[*pathIndex-1].x-1][path[*pathIndex-1].y-1]==0)
                {
                    if(((path[*pathIndex-1].x-1)>=0)&&((path[*pathIndex-1].y-1)>=0))
                    {
                        path[*pathIndex].x = path[*pathIndex-1].x-1;
                        path[*pathIndex].y = path[*pathIndex-1].y-1;
                        *totalStepNum += 1;
                        path[*pathIndex+1].x = path[*pathIndex].x;
                        path[*pathIndex+1].y = path[*pathIndex].y;
                        path[*pathIndex+1].z = path[*pathIndex].z;
                        return;
                    }                    
                }
                if(threeDMap[mapHLow][path[*pathIndex-1].x-1][path[*pathIndex-1].y]==0)
                {
                    if((path[*pathIndex-1].x-1)>=0)
                    {
                        path[*pathIndex].x = path[*pathIndex-1].x-1;
                        path[*pathIndex].y = path[*pathIndex-1].y;
                        *totalStepNum += 1;
                        path[*pathIndex+1].x = path[*pathIndex].x;
                        path[*pathIndex+1].y = path[*pathIndex].y;
                        path[*pathIndex+1].z = path[*pathIndex].z;
                        return;
                    }                    
                }
                if(threeDMap[mapHLow][path[*pathIndex-1].x-1][path[*pathIndex-1].y+1]==0)
                {
                    if((path[*pathIndex-1].x-1)>=0)
                    {
                        path[*pathIndex].x = path[*pathIndex-1].x-1;
                        path[*pathIndex].y = path[*pathIndex-1].y+1;
                        *totalStepNum += 1;
                        path[*pathIndex+1].x = path[*pathIndex].x;
                        path[*pathIndex+1].y = path[*pathIndex].y;
                        path[*pathIndex+1].z = path[*pathIndex].z;
                        return;
                    }                    
                }
                if(threeDMap[mapHLow][path[*pathIndex-1].x+1][path[*pathIndex-1].y-1]==0)
                {
                    if((path[*pathIndex-1].y-1)>=0)
                    {
                        path[*pathIndex].x = path[*pathIndex-1].x+1;
                        path[*pathIndex].y = path[*pathIndex-1].y-1;
                        *totalStepNum += 1;
                        path[*pathIndex+1].x = path[*pathIndex].x;
                        path[*pathIndex+1].y = path[*pathIndex].y;
                        path[*pathIndex+1].z = path[*pathIndex].z;
                        return;
                    }                    
                }
            }
            if((path[*pathIndex].x == path[*pathIndex-1].x)&&(path[*pathIndex].y == path[*pathIndex-1].y))
            {

                path[*pathIndex].z += 1;
                //避免第二次碰撞误认为是垂直类型
                *totalStepNum += 1;
                path[*pathIndex+1].x = path[*pathIndex].x;
                path[*pathIndex+1].y = path[*pathIndex].y;
                path[*pathIndex+1].z = path[*pathIndex].z;
            }
            return ;
        }
        else
        {            
            //运动的空闲飞机                      
            if(crashType == 1)                                //碰撞类型为垂直
            {                                                 //如果发生垂直碰撞,就让他炸吧 
                if(path[*pathIndex-1].z < mapHLow)            //当前高度小于最低飞行高度,则保持在原地不动,这种的只会发生一次碰撞
                {
                    if(*pathIndex > 0)                        //以防万一
                    *pathIndex -= 1;
                    return;
                }      
                return;         
            }               
            *totalStepNum = tmpTotalStepNum;
            int i=0;
            for( i = tmpIndex-1;i<= tmpIndex + remainLevelStepNum -1 ;i++)        //注意水平剩余步数的计算不能比其他的要多一步
            {
                path[i].z +=1;               
            }
           
            tmpIndex -= 1;
            *pathIndex = tmpIndex;
            int currentHigh = path[i-1].z;                  //当前高度,下降到最低高度
          
            for(int j=mapHLow;j<currentHigh;j++)            //下降到最低高度
            {
                path[i].x = path[i-1].x; 
                path[i].y = path[i-1].y;
                path[i].z = path[i-1].z - 1;
                i += 1; 
            }  
            path[*totalStepNum].x = path[*totalStepNum-1].x;  
            path[*totalStepNum].y = path[*totalStepNum-1].y; 
            path[*totalStepNum].z = path[*totalStepNum-1].z;   
            *totalStepNum += 1;            
        }                              
    }
    else                                                    //非空闲状态下 ,当前位置开始,所有位置点Z轴都要加1
    {       
        if(path[*pathIndex-1].z + 1 > mapHHigh)           //说明飞机当前正处于最高高度飞行,不能再加1了,否则违规直接输
        {
            return ;
        }    
        if(crashType == 1)                                //碰撞类型为垂直
        {                                                 //如果发生垂直碰撞,就让他炸吧 
            
            if(path[*pathIndex-1].z < mapHLow)            //当前高度小于最低飞行高度,则保持在原地不动,这种的只会发生一次碰撞
            {
                if(*pathIndex > 0)                        //以防万一
                *pathIndex -= 1;
                return;
            }          
            return ;
        }     
        tmpTotalStepNum +=1;                              //原本因为上升了一个高度和下降了一个高度,总的步数加2的,但在上升的过程中重复了一个索引,所以只需要加1
        *totalStepNum = tmpTotalStepNum;
        int i=0;
        for( i = tmpIndex-1;i<= tmpIndex - 1 + remainLevelStepNum;i++)                  //z轴加1的数量要比水平剩余水平步子数多一,因为其前一时刻的位置也要加一个1
        {
            path[i].z +=1;
        }
        tmpIndex -=1;
        *pathIndex = tmpIndex;
        int currentHigh = path[i-1].z;
        for(int j=0;j<currentHigh;j++)            //下降状态修改
        {
            path[i].x = path[i-1].x; 
            path[i].y = path[i-1].y;
            path[i].z = path[i-1].z - 1;
            i += 1; 
        }
    }   
}

//我方战机如果出现一个上升一个下降的情况,就直接让其撞毁算了
//避免碰撞函数
//策略:遇到碰撞的情况一个保持不动,一个让行,接收到任务的无人机优先,其次是执行任务的无人机,最后是空闲无人机(注意空闲无人机是保持不动的,所以出现碰撞情况后需要对其移动)
//避障优先级:          接受任务   >   执行任务   >   敢死队  >  空闲状态 > 追踪者
//还需要曾加敢死队和清道夫避撞
void AvoidCrash(void)
{
    int updateTotalUAVNum = idleStatusUAVNum + acceptedStatusUAVNum + executingStatusUAVNum + deatSquadsUAVNum + trackerUavNum;        //总的无人机数
    int accepted = acceptedStatusUAVNum;
    int taskNum = executingStatusUAVNum + acceptedStatusUAVNum;   
    int moveNum = executingStatusUAVNum + acceptedStatusUAVNum + deatSquadsUAVNum;
    int trackerNum = moveNum + trackerUavNum;
    for(int i =0;i<updateTotalUAVNum;i++)
    {
        for(int j=i+1;j<updateTotalUAVNum;j++)
        {
            if(i < accepted)                             //成立说明该无人机A处于取货状态下
            {             
                
                if(j < accepted)                         //成立说明该无人机B处于取货状态下
                {
                   
                    int detecResult = DetectCrash(acceptedUAV[i].pPath,acceptedUAV[i].pathIndex,acceptedUAV[j].pPath,acceptedUAV[j].pathIndex);
                    if(detecResult)                      //成立说明有碰撞
                    {   
                        int crashType = 0;     //0:水平碰撞  1:垂直碰撞
                        if((acceptedUAV[i].pPath[acceptedUAV[i].pathIndex].z != acceptedUAV[i].pPath[acceptedUAV[i].pathIndex-1].z)||(acceptedUAV[j].pPath[acceptedUAV[j].pathIndex].z != acceptedUAV[j].pPath[acceptedUAV[j].pathIndex-1].z))
                        {
                            crashType = 1;
                        }
                        
                        int remainLevelStepNum = acceptedUAV[j].stepNum  - acceptedUAV[j].pathIndex -acceptedUAV[j].baseInfo.nZ;
                        ChangeRode(acceptedUAV[j].pPath,&acceptedUAV[j].pathIndex,remainLevelStepNum,&acceptedUAV[j].stepNum,UAV_TASK_ACCEPTED,crashType);          
                    }
                }
                else if(j < taskNum)           //成立说明为载货无人机                  
                {                 
                    int detecResult = DetectCrash(acceptedUAV[i].pPath,acceptedUAV[i].pathIndex,executingUAV[j-accepted].pPath,executingUAV[j-accepted].pathIndex);
                    if(detecResult)                      //成立说明有碰撞
                    {
                        int crashType = 0;     //0:水平碰撞  1:垂直碰撞
                        if((acceptedUAV[i].pPath[acceptedUAV[i].pathIndex].z != acceptedUAV[i].pPath[acceptedUAV[i].pathIndex-1].z)||(executingUAV[j-accepted].pPath[executingUAV[j-accepted].pathIndex].z != executingUAV[j-accepted].pPath[executingUAV[j-accepted].pathIndex-1].z))
                        {
                            crashType = 1;
                        }                      
                        int remainLevelStepNum = executingUAV[j-accepted].stepNum  - executingUAV[j-accepted].pathIndex -executingUAV[j-accepted].baseInfo.nZ ;
                        ChangeRode(executingUAV[j-accepted].pPath,&executingUAV[j-accepted].pathIndex,remainLevelStepNum,&executingUAV[j-accepted].stepNum,UAV_TASK_EXECUTING,crashType);
                    }
                }
                else if(j < moveNum)              
                {
                    int detecResult = DetectCrash(acceptedUAV[i].pPath,acceptedUAV[i].pathIndex,deatSquadsUAV[j-taskNum].pPath,deatSquadsUAV[j-taskNum].pathIndex);
                    if(detecResult)                      //成立说明有碰撞
                    {
                        int crashType = 0;     //0:水平碰撞  1:垂直碰撞
                        if((acceptedUAV[i].pPath[acceptedUAV[i].pathIndex].z != acceptedUAV[i].pPath[acceptedUAV[i].pathIndex-1].z)||(deatSquadsUAV[j-taskNum].pPath[deatSquadsUAV[j-taskNum].pathIndex].z != deatSquadsUAV[j-taskNum].pPath[deatSquadsUAV[j-taskNum].pathIndex-1].z))
                        {
                            crashType = 1;
                        }
                        int remainLevelStepNum = deatSquadsUAV[j-taskNum].stepNum  - deatSquadsUAV[j-taskNum].pathIndex -deatSquadsUAV[j-taskNum].baseInfo.nZ ;
                        ChangeRode(deatSquadsUAV[j-taskNum].pPath,&deatSquadsUAV[j-taskNum].pathIndex,remainLevelStepNum,&deatSquadsUAV[j-taskNum].stepNum,UAV_TASK_DEATH,crashType);
                    }
                }
                else if(j < trackerNum)          //追踪者
                {
                    int detecResult = DetectCrash(acceptedUAV[i].pPath,acceptedUAV[i].pathIndex,trackerUav[j-moveNum].pPath,trackerUav[j-moveNum].pathIndex);
                    if(detecResult)                      //成立说明有碰撞
                    {
                        int crashType = 0;     //0:水平碰撞  1:垂直碰撞
                        if((acceptedUAV[i].pPath[acceptedUAV[i].pathIndex].z != acceptedUAV[i].pPath[acceptedUAV[i].pathIndex-1].z)||(trackerUav[j-moveNum].pPath[trackerUav[j-moveNum].pathIndex].z != trackerUav[j-moveNum].pPath[trackerUav[j-moveNum].pathIndex-1].z))
                        {
                            crashType = 1;
                        }
                        int remainLevelStepNum = trackerUav[j-moveNum].stepNum  - trackerUav[j-moveNum].pathIndex -(trackerUav[j-moveNum].baseInfo.nZ - trackerUav[j-moveNum].pPath[trackerUav[j-moveNum].stepNum-1].z);
                        ChangeRode(trackerUav[j-moveNum].pPath,&trackerUav[j-moveNum].pathIndex,remainLevelStepNum,&trackerUav[j-moveNum].stepNum,UAV_TASK_IDLE,crashType);
                    }
                }
                else                      //否则为空闲无人机        只要将其当前坐标加个1就可以了
                {
                    int detecResult = DetectCrash(acceptedUAV[i].pPath,acceptedUAV[i].pathIndex,idleUAV[j-trackerNum].pPath,idleUAV[j-trackerNum].pathIndex);
                    if(detecResult)                      //成立说明有碰撞
                    {
                        int crashType = 0;     //0:水平碰撞  1:垂直碰撞
                        if((acceptedUAV[i].pPath[acceptedUAV[i].pathIndex].z != acceptedUAV[i].pPath[acceptedUAV[i].pathIndex-1].z)||(idleUAV[j-trackerNum].pPath[idleUAV[j-trackerNum].pathIndex].z != idleUAV[j-trackerNum].pPath[idleUAV[j-trackerNum].pathIndex-1].z))
                        {
                            crashType = 1;
                        }
                        //int remainLevelStepNum = idleUAV[j-moveNum].stepNum - idleUAV[j-moveNum].pathIndex -idleUAV[j-moveNum].baseInfo.nZ ;
                        int remainLevelStepNum = idleUAV[j-trackerNum].stepNum - idleUAV[j-trackerNum].pathIndex ;
                        ChangeRode(idleUAV[j-trackerNum].pPath,&idleUAV[j-trackerNum].pathIndex,remainLevelStepNum,&idleUAV[j-trackerNum].stepNum,UAV_TASK_IDLE,crashType);
                    }
                }                    
            }
            else if(i < taskNum)                //成立说明A为载货无人机                                   
            {
                if(j < taskNum)          // 成立为载货
                {                       
                    int detecResult = DetectCrash(executingUAV[i-accepted].pPath,executingUAV[i-accepted].pathIndex,executingUAV[j-accepted].pPath,executingUAV[j-accepted].pathIndex);
                    if(detecResult)                      //成立说明有碰撞
                    {
                        int crashType = 0;     //0:水平碰撞  1:垂直碰撞
                        if((executingUAV[i-accepted].pPath[executingUAV[i-accepted].pathIndex].z != executingUAV[i-accepted].pPath[executingUAV[i-accepted].pathIndex-1].z)||(executingUAV[j-accepted].pPath[executingUAV[j-accepted].pathIndex].z != executingUAV[j-accepted].pPath[executingUAV[j-accepted].pathIndex-1].z))
                        {
                            crashType = 1;
                        }
                        int remainLevelStepNum = executingUAV[j-accepted].stepNum  - executingUAV[j-accepted].pathIndex -executingUAV[j-accepted].baseInfo.nZ;
                        ChangeRode(executingUAV[j-accepted].pPath,&executingUAV[j-accepted].pathIndex,remainLevelStepNum,&executingUAV[j-accepted].stepNum,UAV_TASK_EXECUTING,crashType);
                    }
                }
                else if(j < moveNum)                    //为空闲
                {
                    int detecResult = DetectCrash(executingUAV[i-accepted].pPath,executingUAV[i-accepted].pathIndex,deatSquadsUAV[j-taskNum].pPath,deatSquadsUAV[j-taskNum].pathIndex);
                    if(detecResult)                      //成立说明有碰撞
                    {
                        int crashType = 0;     //0:水平碰撞  1:垂直碰撞
                        if((executingUAV[i-accepted].pPath[executingUAV[i-accepted].pathIndex].z != executingUAV[i-accepted].pPath[executingUAV[i-accepted].pathIndex-1].z)||(deatSquadsUAV[j-taskNum].pPath[deatSquadsUAV[j-taskNum].pathIndex].z != deatSquadsUAV[j-taskNum].pPath[deatSquadsUAV[j-taskNum].pathIndex-1].z))
                        {
                            crashType = 1;
                        }
                        int remainLevelStepNum = deatSquadsUAV[j-taskNum].stepNum  - deatSquadsUAV[j-taskNum].pathIndex -deatSquadsUAV[j-taskNum].baseInfo.nZ ;
                        ChangeRode(deatSquadsUAV[j-taskNum].pPath,&deatSquadsUAV[j-taskNum].pathIndex,remainLevelStepNum,&deatSquadsUAV[j-taskNum].stepNum,UAV_TASK_DEATH,crashType);
                    }
                }
                else if(j < trackerNum)          //追踪者
                {
                    int detecResult = DetectCrash(executingUAV[i-accepted].pPath,executingUAV[i-accepted].pathIndex,trackerUav[j-moveNum].pPath,trackerUav[j-moveNum].pathIndex);
                    if(detecResult)                      //成立说明有碰撞
                    {
                        int crashType = 0;     //0:水平碰撞  1:垂直碰撞
                        if((executingUAV[i-accepted].pPath[executingUAV[i-accepted].pathIndex].z != executingUAV[i-accepted].pPath[executingUAV[i-accepted].pathIndex-1].z)||(trackerUav[j-moveNum].pPath[trackerUav[j-moveNum].pathIndex].z != trackerUav[j-moveNum].pPath[trackerUav[j-moveNum].pathIndex-1].z))
                        {
                            crashType = 1;
                        }
                        int remainLevelStepNum = trackerUav[j-moveNum].stepNum  - trackerUav[j-moveNum].pathIndex -(trackerUav[j-moveNum].baseInfo.nZ - trackerUav[j-moveNum].pPath[trackerUav[j-moveNum].stepNum-1].z);
                        ChangeRode(trackerUav[j-moveNum].pPath,&trackerUav[j-moveNum].pathIndex,remainLevelStepNum,&trackerUav[j-moveNum].stepNum,UAV_TASK_IDLE,crashType);
                    }
                }
                else
                {
                    int detecResult = DetectCrash(executingUAV[i-accepted].pPath,executingUAV[i-accepted].pathIndex,idleUAV[j-trackerNum].pPath,idleUAV[j-trackerNum].pathIndex);
                    if(detecResult)                      //成立说明有碰撞
                    {
                        int crashType = 0;     //0:水平碰撞  1:垂直碰撞
                        if((executingUAV[i-accepted].pPath[executingUAV[i-accepted].pathIndex].z != executingUAV[i-accepted].pPath[executingUAV[i-accepted].pathIndex-1].z)||(idleUAV[j-trackerNum].pPath[idleUAV[j-trackerNum].pathIndex].z != idleUAV[j-trackerNum].pPath[idleUAV[j-trackerNum].pathIndex-1].z))
                        {
                            crashType = 1;
                        }
                        //int remainLevelStepNum = idleUAV[j-moveNum].stepNum - idleUAV[j-moveNum].pathIndex -idleUAV[j-moveNum].baseInfo.nZ ;
                        int remainLevelStepNum = idleUAV[j-trackerNum].stepNum  - idleUAV[j-trackerNum].pathIndex;
                        ChangeRode(idleUAV[j-trackerNum].pPath,&idleUAV[j-trackerNum].pathIndex,remainLevelStepNum,&idleUAV[j-trackerNum].stepNum,UAV_TASK_IDLE,crashType);
                    }                            
                }
            }
            else if(i < moveNum)         //A为敢死队无人机                           
            {
                if(j < moveNum)
                {
                    int detecResult = DetectCrash(deatSquadsUAV[i-taskNum].pPath,deatSquadsUAV[i-taskNum].pathIndex,deatSquadsUAV[j-taskNum].pPath,deatSquadsUAV[j-taskNum].pathIndex);
                    if(detecResult)                      //成立说明有碰撞
                    {
                        int crashType = 0;     //0:水平碰撞  1:垂直碰撞
                        if((deatSquadsUAV[i-taskNum].pPath[deatSquadsUAV[i-taskNum].pathIndex].z != deatSquadsUAV[i-taskNum].pPath[deatSquadsUAV[i-taskNum].pathIndex-1].z)||(deatSquadsUAV[j-taskNum].pPath[deatSquadsUAV[j-taskNum].pathIndex].z != deatSquadsUAV[j-taskNum].pPath[deatSquadsUAV[j-taskNum].pathIndex-1].z))
                        {
                            crashType = 1;
                        }
                        int remainLevelStepNum = deatSquadsUAV[j-taskNum].stepNum  - deatSquadsUAV[j-taskNum].pathIndex -deatSquadsUAV[j-taskNum].baseInfo.nZ;
                        ChangeRode(deatSquadsUAV[j-taskNum].pPath,&deatSquadsUAV[j-taskNum].pathIndex,remainLevelStepNum,&deatSquadsUAV[j-taskNum].stepNum,UAV_TASK_IDLE,crashType);
                    }
                } 
                else if(j < trackerNum)          //追踪者
                {
                    int detecResult = DetectCrash(deatSquadsUAV[i-taskNum].pPath,deatSquadsUAV[i-taskNum].pathIndex,trackerUav[j-moveNum].pPath,trackerUav[j-moveNum].pathIndex);
                    if(detecResult)                      //成立说明有碰撞
                    {
                        int crashType = 0;     //0:水平碰撞  1:垂直碰撞
                        if((deatSquadsUAV[i-taskNum].pPath[deatSquadsUAV[i-taskNum].pathIndex].z != deatSquadsUAV[i-taskNum].pPath[deatSquadsUAV[i-taskNum].pathIndex-1].z)||(trackerUav[j-moveNum].pPath[trackerUav[j-moveNum].pathIndex].z != trackerUav[j-moveNum].pPath[trackerUav[j-moveNum].pathIndex-1].z))
                        {
                            crashType = 1;
                        }
                        int remainLevelStepNum = trackerUav[j-moveNum].stepNum  - trackerUav[j-moveNum].pathIndex - (trackerUav[j-moveNum].baseInfo.nZ - trackerUav[j-moveNum].pPath[trackerUav[j-moveNum].stepNum - 1].z);
                        ChangeRode(trackerUav[j-moveNum].pPath,&trackerUav[j-moveNum].pathIndex,remainLevelStepNum,&trackerUav[j-moveNum].stepNum,UAV_TASK_IDLE,crashType);
                    }
                }
                else   //B为空闲
                {
                    int detecResult = DetectCrash(deatSquadsUAV[i-taskNum].pPath,deatSquadsUAV[i-taskNum].pathIndex,idleUAV[j-trackerNum].pPath,idleUAV[j-trackerNum].pathIndex);
                    if(detecResult)                      //成立说明有碰撞
                    {
                        int crashType = 0;     //0:水平碰撞  1:垂直碰撞
                        if((deatSquadsUAV[i-taskNum].pPath[deatSquadsUAV[i-taskNum].pathIndex].z != deatSquadsUAV[i-taskNum].pPath[deatSquadsUAV[i-taskNum].pathIndex-1].z)||(idleUAV[j-trackerNum].pPath[idleUAV[j-trackerNum].pathIndex].z != idleUAV[j-trackerNum].pPath[idleUAV[j-trackerNum].pathIndex-1].z))
                        {
                            crashType = 1;
                        }
                        //int remainLevelStepNum = idleUAV[j-moveNum].stepNum - idleUAV[j-moveNum].pathIndex -idleUAV[j-moveNum].baseInfo.nZ;
                        int remainLevelStepNum = idleUAV[j-trackerNum].stepNum  - idleUAV[j-trackerNum].pathIndex;
                        ChangeRode(idleUAV[j-trackerNum].pPath,&idleUAV[j-trackerNum].pathIndex,remainLevelStepNum,&idleUAV[j-trackerNum].stepNum,UAV_TASK_IDLE,crashType);
                    }
                }                      
            }
            else if(i < trackerNum)
            {
                if(j < trackerNum)
                {
                    int detecResult = DetectCrash(trackerUav[i-moveNum].pPath,trackerUav[i-moveNum].pathIndex,trackerUav[j-moveNum].pPath,trackerUav[j-moveNum].pathIndex);
                    if(detecResult)                      //成立说明有碰撞
                    {
                        int crashType = 0;               //0:水平碰撞  1:垂直碰撞
                        if((trackerUav[i-moveNum].pPath[trackerUav[i-moveNum].pathIndex].z != trackerUav[i-moveNum].pPath[trackerUav[i-moveNum].pathIndex-1].z)||(trackerUav[j-moveNum].pPath[trackerUav[j-moveNum].pathIndex].z != trackerUav[j-moveNum].pPath[trackerUav[j-moveNum].pathIndex-1].z))
                        {
                            crashType = 1;
                        }
                        int remainLevelStepNum = trackerUav[j-moveNum].stepNum  - trackerUav[j-moveNum].pathIndex -(trackerUav[j-moveNum].baseInfo.nZ - trackerUav[j-moveNum].pPath[trackerUav[j-moveNum].stepNum - 1].z );
                        ChangeRode(trackerUav[j-moveNum].pPath,&trackerUav[j-moveNum].pathIndex,remainLevelStepNum,&trackerUav[j-moveNum].stepNum,UAV_TASK_IDLE,crashType);
                    }
                }
                else
                {
                    int detecResult = DetectCrash(trackerUav[i-moveNum].pPath,trackerUav[i-moveNum].pathIndex,idleUAV[j-trackerNum].pPath,idleUAV[j-trackerNum].pathIndex);
                    if(detecResult)                      //成立说明有碰撞
                    {
                        int crashType = 0;     //0:水平碰撞  1:垂直碰撞
                        if((trackerUav[i-moveNum].pPath[trackerUav[i-moveNum].pathIndex].z != trackerUav[i-moveNum].pPath[trackerUav[i-moveNum].pathIndex-1].z)||(idleUAV[j-trackerNum].pPath[idleUAV[j-trackerNum].pathIndex].z != idleUAV[j-trackerNum].pPath[idleUAV[j-trackerNum].pathIndex-1].z))
                        {
                            crashType = 1;
                        }
                        int remainLevelStepNum = idleUAV[j-trackerNum].stepNum  - idleUAV[j-trackerNum].pathIndex;
                        ChangeRode(idleUAV[j-trackerNum].pPath,&idleUAV[j-trackerNum].pathIndex,remainLevelStepNum,&idleUAV[j-trackerNum].stepNum,UAV_TASK_IDLE,crashType);
                    }
                }
            }
            else      //成立说明A为空闲无人机
            {
                //两个无人机均在空闲状态下会出现都在停机坪上,此时应直接退出  
                int tmpx = idleUAV[i-trackerNum].pPath[idleUAV[i-trackerNum].pathIndex].x;
                int tmpy = idleUAV[i-trackerNum].pPath[idleUAV[i-trackerNum].pathIndex].y;
                if((tmpx==parkingAapron.x) && (tmpy==parkingAapron.y))
                {
                    return ;
                }           
                int detecResult = DetectCrash(idleUAV[i-trackerNum].pPath,idleUAV[i-trackerNum].pathIndex,idleUAV[j-trackerNum].pPath,idleUAV[j-trackerNum].pathIndex);

                if(detecResult)                      //成立说明有碰撞
                {
                    int crashType = 0;     //0:水平碰撞  1:垂直碰撞
                    if((idleUAV[i-trackerNum].pPath[idleUAV[i-trackerNum].pathIndex].z != idleUAV[i-trackerNum].pPath[idleUAV[i-trackerNum].pathIndex-1].z)||(idleUAV[j-trackerNum].pPath[idleUAV[j-trackerNum].pathIndex].z != idleUAV[j-trackerNum].pPath[idleUAV[j-trackerNum].pathIndex-1].z))
                    {
                        crashType = 1;
                    }
                    //int remainLevelStepNum = idleUAV[j-moveNum].stepNum - idleUAV[j-moveNum].pathIndex -idleUAV[j-moveNum].baseInfo.nZ;
                    int remainLevelStepNum = idleUAV[j-trackerNum].stepNum  - idleUAV[j-trackerNum].pathIndex;                            
                    ChangeRode(idleUAV[j-trackerNum].pPath,&idleUAV[j-trackerNum].pathIndex,remainLevelStepNum,&idleUAV[j-trackerNum].stepNum,UAV_TASK_IDLE,crashType);                             
                }   
            }                  
        }
    }   
}

//功能:购买无人机
//返回购买无人机的类型号 -1:不购买 ,0:购买成功
int PurchaseUAV(const MATCH_STATUS * pstMatch,const UAV_PRICE *priceList,int uavTypeNum,int purUavArray[],int purNum[],int *purchaseTypeNum)
{
    static int count = 1;
    static int purchasedNum = 0;  
    int uavTypeNo = -1;                                                             //已购买无人机数量
    int initPurchastMinNum = mapWidth *0.15;
    int listIndex=0;
    //nUavWeNum该值还包含了我方撞毁无人机的数量
    if((purchasedNum < initPurchastMinNum)||((pstMatch->nUavWeNum - crashUAVNum - 4) < pstMatch->nUavEnemyNum))                                         //成立说明需要先购买满足数量要求的小无人机
    {
        listIndex = (int)(uavTypeNum *0.0);       
        if(totalObtainedGoodValue >= priceList[listIndex].nValue)                  //总的价值大于无人机费用价值,则可以购买
        {
            purchasedNum += 1;
            totalObtainedGoodValue -= priceList[listIndex].nValue;            
            sscanf(priceList[listIndex].szType,"F%d",&uavTypeNo);
            purUavArray[0] = uavTypeNo;
            purNum[0] = 1;
            *purchaseTypeNum = 1;
            return 0;            
        }       
    }        
    else if(count < 3)                                                  
    {                               
        listIndex = (int)(uavTypeNum *0.5);       
        if(totalObtainedGoodValue >= priceList[listIndex].nValue)                  //总的价值大于无人机费用价值,则可以购买
        {
            count += 1; 
            totalObtainedGoodValue -= priceList[listIndex].nValue;            
            sscanf(priceList[listIndex].szType,"F%d",&uavTypeNo);
            purUavArray[0] = uavTypeNo;
            purNum[0] = 1;
            *purchaseTypeNum = 1;
            return 0;          
        }      
    }
    else if(count < 5)                                                                 
    {            
        listIndex = (int)(uavTypeNum *0.0);       
        if(totalObtainedGoodValue >= priceList[listIndex].nValue)                  //总的价值大于无人机费用价值,则可以购买
        {
            count += 1; 
            totalObtainedGoodValue -= priceList[listIndex].nValue;            
            sscanf(priceList[listIndex].szType,"F%d",&uavTypeNo);
            purUavArray[0] = uavTypeNo;
            purNum[0] = 1;
            *purchaseTypeNum = 1;
            return 0; 
        }   
    }   
    else if(count < 6)                                                                 
    {            
        listIndex = (int)(uavTypeNum *0.75);       
        if(totalObtainedGoodValue >= priceList[listIndex].nValue)                  //总的价值大于无人机费用价值,则可以购买
        {
            count += 1; 
            totalObtainedGoodValue -= priceList[listIndex].nValue;            
            sscanf(priceList[listIndex].szType,"F%d",&uavTypeNo);
            purUavArray[0] = uavTypeNo;
            purNum[0] = 1;
            *purchaseTypeNum = 1;
            return 0; 
        }   
    }  
    else if(count < 8)                                                                 
    {            
        listIndex = (int)(uavTypeNum *0.0);       
        if(totalObtainedGoodValue >= priceList[listIndex].nValue)                  //总的价值大于无人机费用价值,则可以购买
        {
            count += 1; 
            totalObtainedGoodValue -= priceList[listIndex].nValue;            
            sscanf(priceList[listIndex].szType,"F%d",&uavTypeNo);
            purUavArray[0] = uavTypeNo;
            purNum[0] = 1;
            *purchaseTypeNum = 1;
            return 0;
        }    
    }     
    else if(count == 8)                                                                 
    {            
        listIndex = (int)(uavTypeNum *0.0);       
        if(totalObtainedGoodValue >= priceList[listIndex].nValue)                  //总的价值大于无人机费用价值,则可以购买
        {
            count = 1; 
            totalObtainedGoodValue -= priceList[listIndex].nValue;            
            sscanf(priceList[listIndex].szType,"F%d",&uavTypeNo);
            purUavArray[0] = uavTypeNo;
            purNum[0] = 1;
            *purchaseTypeNum = 1;
            return 0;
        }    
    }                     
    return -1;
}


//功能:电量管理
//两类无人机:在家充电和载货飞行无人机
void ElectricityManage(void)
{
    for(int i=0;i<executingStatusUAVNum;i++)
    {
        executingUAV[i].baseInfo.nRemainElectricity -= executingUAV[i].goodsWeight;                       //电量减少掉货物的重量
    }
    for(int i=0;i<idleStatusUAVNum;i++)
    {
        //成立说明该无人机在家里,则需要考虑充电的问题
        if((idleUAV[i].baseInfo.nX == homeLocation.x)&&(idleUAV[i].baseInfo.nY == homeLocation.y)&&(idleUAV[i].baseInfo.nZ == homeLocation.z))
        {
            if((idleUAV[i].baseInfo.nRemainElectricity == idleUAV[i].nCapacity))                                                //成立说明该号无人机已经充满电
            {
                continue;
            }
            idleUAV[i].baseInfo.nRemainElectricity += idleUAV[i].nCharge; 
            if(idleUAV[i].baseInfo.nRemainElectricity > idleUAV[i].nCapacity)
            {
                idleUAV[i].baseInfo.nRemainElectricity = idleUAV[i].nCapacity;
            }
        }
    }
}

void PackStateToFlayPlane(MATCH_STATUS * pstMatch,FLAY_PLANE *pstFlayPlane)
{
    int updateTotalUAVNum = idleStatusUAVNum + acceptedStatusUAVNum + executingStatusUAVNum + deatSquadsUAVNum +trackerUavNum;  
    pstFlayPlane->nUavNum = updateTotalUAVNum;  
    //printf("nUavNum = %d\n",pstFlayPlane->nUavNum);
    for(int i=0;i<executingStatusUAVNum;i++)
    {
        pstFlayPlane->astUav[i].nNO = executingUAV[i].nNO;       //无人机编号
        pstFlayPlane->astUav[i].nX = executingUAV[i].pPath[executingUAV[i].pathIndex].x;
        pstFlayPlane->astUav[i].nY = executingUAV[i].pPath[executingUAV[i].pathIndex].y;
        pstFlayPlane->astUav[i].nZ = executingUAV[i].pPath[executingUAV[i].pathIndex].z; 
        pstFlayPlane->astUav[i].nRemainElectricity =  executingUAV[i].baseInfo.nRemainElectricity;           
        pstFlayPlane->astUav[i].nGoodsNo = executingUAV[i].baseInfo.nGoodsNo; 
        // if(pstFlayPlane->astUav[i].nNO == 2)
        // printf("exect: UAVNo=%d matchNo=%d goodsNo=%d x=%d y=%d z=%d r=%d\n",pstFlayPlane->astUav[i].nNO,executingUAV[i].goodsNo,pstFlayPlane->astUav[i].nGoodsNo,pstFlayPlane->astUav[i].nX,pstFlayPlane->astUav[i].nY,pstFlayPlane->astUav[i].nZ,pstFlayPlane->astUav[i].nRemainElectricity);
       
        if(executingUAV[i].pathIndex < (executingUAV[i].stepNum - 1))
        {
            executingUAV[i].pathIndex++; 
        }
             
    }
    int execNum = executingStatusUAVNum;    
    int taskNum = execNum + acceptedStatusUAVNum;

    for(int i=execNum;i<taskNum;i++)
    {           
        pstFlayPlane->astUav[i].nNO = acceptedUAV[i-execNum].nNO;       //无人机编号
        pstFlayPlane->astUav[i].nX = acceptedUAV[i-execNum].pPath[acceptedUAV[i-execNum].pathIndex].x;
        pstFlayPlane->astUav[i].nY = acceptedUAV[i-execNum].pPath[acceptedUAV[i-execNum].pathIndex].y;
        pstFlayPlane->astUav[i].nZ = acceptedUAV[i-execNum].pPath[acceptedUAV[i-execNum].pathIndex].z;  
        pstFlayPlane->astUav[i].nRemainElectricity = acceptedUAV[i-execNum].baseInfo.nRemainElectricity;           
        pstFlayPlane->astUav[i].nGoodsNo = acceptedUAV[i-execNum].baseInfo.nGoodsNo;  

        // if(pstFlayPlane->astUav[i].nNO == 2)
        // {
        //     printf("accept: UAVNo=%d matchNo=%d good=%d x=%d y=%d z=%d r=%d\n",pstFlayPlane->astUav[i].nNO,acceptedUAV[i-execNum].goodsNo,pstFlayPlane->astUav[i].nGoodsNo,pstFlayPlane->astUav[i].nX,pstFlayPlane->astUav[i].nY,pstFlayPlane->astUav[i].nZ,pstFlayPlane->astUav[i].nRemainElectricity);
        // }
        if(acceptedUAV[i-execNum].pathIndex < (acceptedUAV[i-execNum].stepNum - 1))
        {
            acceptedUAV[i-execNum].pathIndex++;
        }        
    }
    int deatNum = taskNum + idleStatusUAVNum;   
    //以上模块均没问题
    for(int i=taskNum;i<deatNum;i++)
    {        
        pstFlayPlane->astUav[i].nNO = idleUAV[i-taskNum].nNO;          //无人机编号
        pstFlayPlane->astUav[i].nX = idleUAV[i-taskNum].pPath[idleUAV[i-taskNum].pathIndex].x;
        pstFlayPlane->astUav[i].nY = idleUAV[i-taskNum].pPath[idleUAV[i-taskNum].pathIndex].y;
        pstFlayPlane->astUav[i].nZ = idleUAV[i-taskNum].pPath[idleUAV[i-taskNum].pathIndex].z; 
        pstFlayPlane->astUav[i].nRemainElectricity = idleUAV[i-taskNum].baseInfo.nRemainElectricity;         
        pstFlayPlane->astUav[i].nGoodsNo = idleUAV[i-taskNum].baseInfo.nGoodsNo;

        // if(pstFlayPlane->astUav[i].nNO == 2)
        // printf("idle: UAVNo=%d matchNo=%d gooodNo=%d x=%d y=%d z=%d r=%d\n",pstFlayPlane->astUav[i].nNO,idleUAV[i-taskNum].goodsNo,pstFlayPlane->astUav[i].nGoodsNo,pstFlayPlane->astUav[i].nX,pstFlayPlane->astUav[i].nY,pstFlayPlane->astUav[i].nZ,pstFlayPlane->astUav[i].nRemainElectricity);
        
        if(idleUAV[i-taskNum].pathIndex < (idleUAV[i-taskNum].stepNum -1))         //因为索引是从0开始的
        {
            idleUAV[i-taskNum].pathIndex++; 
        }       
    }
    int trackNum = deatNum + deatSquadsUAVNum;
   //敢死队
   // printf("deat=%d\n",deatSquadsUAVNum);
    for(int i=deatNum;i<trackNum;i++)
    {        
        pstFlayPlane->astUav[i].nNO = deatSquadsUAV[i-deatNum].nNO;          //无人机编号
        pstFlayPlane->astUav[i].nX = deatSquadsUAV[i-deatNum].pPath[deatSquadsUAV[i-deatNum].pathIndex].x;
        pstFlayPlane->astUav[i].nY = deatSquadsUAV[i-deatNum].pPath[deatSquadsUAV[i-deatNum].pathIndex].y;
        pstFlayPlane->astUav[i].nZ = deatSquadsUAV[i-deatNum].pPath[deatSquadsUAV[i-deatNum].pathIndex].z;         
        pstFlayPlane->astUav[i].nRemainElectricity = deatSquadsUAV[i-deatNum].baseInfo.nRemainElectricity; 

        // if(pstFlayPlane->astUav[i].nNO == 2)
        // printf("deat: UAVNo=%d matchNo=%d gooodNo=%d x=%d y=%d z=%d r=%d\n",pstFlayPlane->astUav[i].nNO,deatSquadsUAV[i-deatNum].goodsNo,deatSquadsUAV[i-updateTotalUAVNum].goodsNo,pstFlayPlane->astUav[i].nX,pstFlayPlane->astUav[i].nY,pstFlayPlane->astUav[i].nZ,pstFlayPlane->astUav[i].nRemainElectricity);
        
        pstFlayPlane->astUav[i].nGoodsNo = deatSquadsUAV[i-deatNum].baseInfo.nGoodsNo;
        if(deatSquadsUAV[i-deatNum].pathIndex < (deatSquadsUAV[i-deatNum].stepNum - 1))
        {
            deatSquadsUAV[i-deatNum].pathIndex++; 
        }       
    }       
   
    for(int i=trackNum;i<updateTotalUAVNum;i++)
    {        
        pstFlayPlane->astUav[i].nNO = trackerUav[i-trackNum].nNO;          //无人机编号
        pstFlayPlane->astUav[i].nX = trackerUav[i-trackNum].pPath[trackerUav[i-trackNum].pathIndex].x;
        pstFlayPlane->astUav[i].nY = trackerUav[i-trackNum].pPath[trackerUav[i-trackNum].pathIndex].y;
        pstFlayPlane->astUav[i].nZ = trackerUav[i-trackNum].pPath[trackerUav[i-trackNum].pathIndex].z;         
        pstFlayPlane->astUav[i].nRemainElectricity = trackerUav[i-trackNum].baseInfo.nRemainElectricity; 

        //  if(pstFlayPlane->astUav[i].nNO == 2)
        //  printf("track: UAVNo=%d matchNo=%d gooodNo=%d x=%d y=%d z=%d r=%d\n",pstFlayPlane->astUav[i].nNO,trackerUav[i-trackNum].goodsNo,trackerUav[i-trackNum].goodsNo,pstFlayPlane->astUav[i].nX,pstFlayPlane->astUav[i].nY,pstFlayPlane->astUav[i].nZ,pstFlayPlane->astUav[i].nRemainElectricity);
        
        pstFlayPlane->astUav[i].nGoodsNo = trackerUav[i-trackNum].baseInfo.nGoodsNo;
        if(trackerUav[i-trackNum].pathIndex < (trackerUav[i-trackNum].stepNum - 1))
        {
            trackerUav[i-trackNum].pathIndex++; 
        }       
    }       

//购买无人机
    pstFlayPlane->nPurchaseNum = 0;                                         //
    
    static int purUavArray[10];
    static int purNum[10];
    static int purchaseTypeNum = 0;
    static int purchaseResult = -1;
    if(pstFlayPlane->nUavNum > 50)
    {
        return ;
    }
    purchaseResult = PurchaseUAV(pstMatch,UAVPriceList,uavPriceNum,purUavArray,purNum,&purchaseTypeNum);
    if(purchaseResult != -1)                                                                                                  //成立说明需要购买无人机
    {
        static int k = 0;
        for(int i=0;i < purchaseTypeNum;i++)                                                                                  //需要购买的类型数
        {
            pstFlayPlane->nPurchaseNum += purNum[i];
            char purchaseType[8] = "\0";
            char purchaseNO[8] = "\0";
            strcat(purchaseType,"F");
            sprintf(purchaseNO,"%d",purUavArray[i]);
            strcat(purchaseType,purchaseNO);
            for(int j=0;j<purNum[i];j++)
            {                   
                strcpy(pstFlayPlane->szPurchaseType[k],purchaseType);
                printf("type = %s\n",pstFlayPlane->szPurchaseType[k]); 
                k++; 
            }  
        }
        k = 0;
    }    
}

void InitThreeDMap(void)               //初始化时将地图设置为全部阻塞
{
    //飞行地图
    for(int k=0;k<dimLength;k++)
    {
        for(int i=0;i<dimLength;i++)
        {
            for(int j=0;j<dimLength;j++)
            {
            
                threeDMap[k][i][j]=1;
            }
        }
    }
    //货物检测地图
    for(int i=0;i<dimLength;i++)
    {
        for(int j=0;j<dimLength;j++)
        {
            goodsDetecMap[i][j]=1;
        }
    }    
}


//在初始化的地图里面加入建筑物
void AddBuildingToThreeDMap(const MAP_INFO *pstMap)
{
    //飞行地图
    for(int m=0;m<pstMap->nBuildingNum;m++)
    {       

        if(buildingMaxHigh < pstMap->astBuilding[m].nH)
        {
            buildingMaxHigh = pstMap->astBuilding[m].nH;         //获取建筑物最大高度,这个值的高度是可以飞行的
        }

        if(pstMap->astBuilding[m].nH<=mapHLow)                       //建筑物高度小于最低飞行高度,则不算是障碍物 nH高度是可以飞行的
        {
            continue ;
        }

        for(int k=mapHLow;k < pstMap->astBuilding[m].nH;k++)          //从最低飞行高度开始加入建筑物
        {
            for(int i=pstMap->astBuilding[m].nX;i<(pstMap->astBuilding[m].nX+pstMap->astBuilding[m].nL);i++)
            {
                for(int j=pstMap->astBuilding[m].nY;j<(pstMap->astBuilding[m].nY+pstMap->astBuilding[m].nW);j++)
                {
                    threeDMap[k][i][j]=1;                                                //设置障碍物
                }
            }
        }        
    }
    if(buildingMaxHigh >= pstMap->nMapZ)                      //万一建筑物最大高度高于地图高度呢
    {
        buildingMaxHigh = pstMap->nMapZ;
    }

    //货物检测地图
    for(int k=0;k<pstMap->nBuildingNum;k++)
    {       
        for(int i=pstMap->astBuilding[k].nX;i<(pstMap->astBuilding[k].nX+pstMap->astBuilding[k].nL);i++)
        {
            for(int j=pstMap->astBuilding[k].nY;j<(pstMap->astBuilding[k].nY+pstMap->astBuilding[k].nW);j++)
            {
                goodsDetecMap[i][j]=1;         //设置障碍物
            }
        }
    }
}




//将三维地图转换为平面地图形式,以最低飞行高度所在平面为地图
void CreatThreeDMap(const MAP_INFO *pstMap)
{
    //飞行地图
    for(int k=0;k<pstMap->nMapZ;k++)                  //第一个维度为高
    {
         for(int i=0;i<pstMap->nMapX;i++)
        {
            for(int j=0;j<pstMap->nMapY;j++)
            {
                threeDMap[k][i][j] = 0;                              //初始设置为全部可通路
            }
        }
    }
   
    //货物检测地图
    for(int i=0;i<pstMap->nMapX;i++)
    {
        for(int j=0;j<pstMap->nMapY;j++)
        {
            goodsDetecMap[i][j] = 0;                              //初始设置为全部可通路
        }
    }

    AddBuildingToThreeDMap(pstMap);     //将建筑物添加到地图中去
}

/** @fn     void AlgorithmCalculationFun()
 *  @brief	 
 *	@return void
 */
void  AlgorithmCalculationFun(MAP_INFO *pstMap, MATCH_STATUS * pstMatch, FLAY_PLANE *pstFlayPlane)
{
    ProcessState(pstMatch);
    
    InitTrackerUav();

    UpdateTrakerUav(pstMatch);
    
    InterceptionEnemyCarryGoodsUav();                   //只用空闲者拦截

    TaskSchedule();                                   //这个函数不能一开始就被调用,因为如果一开始都匹配到了任务的话,那么他们就会同一时刻出现在家的高度加1的位置点,此时就会全部被撞毁
    
    ScatterHomeIdleUav();

    TaskExecteInit();  
     
    FinishTaskDetec();    

    AvoidCrash();    
    
    //需要加一个电量管理函数
    ElectricityManage();

    PackStateToFlayPlane(pstMatch,pstFlayPlane); 
}

void PrintGoods(MATCH_STATUS * pstMatch)
{
    for(int i=0;i<pstMatch->nGoodsNum;i++)
    {
        printf("NO:%d  nLeftTime:%d\n",pstMatch->astGoods[i].nNO,pstMatch->astGoods[i].nLeftTime);
    }
    printf("\n");
}

void PrintUavPriceList(UAV_PRICE *price,MAP_INFO *pstMapInfo)
{
    for(int i=0;i<pstMapInfo->nUavPriceNum;i++)
    {
        printf("type=%s w=%d v=%d ca=%d ch=%d\n",price[i].szType,price[i].nLoadWeight,price[i].nValue,price[i].nCapacity,price[i].nCharge);
    }
    printf("\n");
}

int main(int argc, char *argv[])
{

    char        szIp[64] = { 0 };
    int         nPort = 0;
    char        szToken[128] = { 0 };
    int         nRet = 0;
    OS_SOCKET   hSocket;
    char        *pRecvBuffer = NULL;
    char        *pSendBuffer = NULL;
    int         nLen = 0;   
    clock_t startTime,finishTime;  
    if (argc != 4)
    {
        printf("error arg num\n");
        return -1;
    }
    //srand((unsigned)time(NULL));                          //随机数生成随机目标地图
    // ��������
    strcpy(szIp, argv[1]);
    nPort = atoi(argv[2]);
    strcpy(szToken, argv[3]);  

    printf("server ip %s, prot %d, token %s\n", szIp, nPort, szToken);    
    //地图初始化
    InitThreeDMap();

    // 
    nRet = OSCreateSocket(szIp, (unsigned int)nPort, &hSocket);
    if (nRet != 0)
    {
        printf("connect server error\n");
        return nRet;
    }
   
    // �������ܷ����ڴ�
    pRecvBuffer = (char*)malloc(MAX_SOCKET_BUFFER);
    if (pRecvBuffer == NULL)
    {
        return -1;
    }

    pSendBuffer = (char*)malloc(MAX_SOCKET_BUFFER);
    if (pSendBuffer == NULL)
    {
        free(pRecvBuffer);
        return -1;
    }

    memset(pRecvBuffer, 0, MAX_SOCKET_BUFFER);

    // 接受到 hello, what's your token?  
    nRet = RecvJuderData(hSocket, pRecvBuffer);   
    if (nRet != 0)
    {
        return nRet;
    }
    CONNECT_NOTICE  stNotice;

    nRet = ParserConnect(pRecvBuffer + SOCKET_HEAD_LEN, &stNotice);
    if (nRet != 0)
    {
        return nRet;
    }
    
    // ���ɱ������ݵ�json
    TOKEN_INFO  stToken;

    strcpy(stToken.szToken, szToken);  // �����ǵ��Խ׶Σ������������Ե�token�����ҵĶ�ս�л�ȡ��
                                       // ʵ�ʱ�������Ҫ�������Եģ�����demoд�ģ��з��������ô��롣
    strcpy(stToken.szAction, "sendtoken");

    memset(pSendBuffer, 0, MAX_SOCKET_BUFFER);
    nRet = CreateTokenInfo(&stToken, pSendBuffer, &nLen);
    if (nRet != 0)
    {
        return nRet;
    }
    // 向裁判表明身份,发送令牌(Player -> Judger)
    nRet = SendJuderData(hSocket, pSendBuffer, nLen);
    if (nRet != 0)
    {
        return nRet;
    }
    //返回身份验证结果,并获取匹配信息(Judger -> Player)��
    memset(pRecvBuffer, 0, MAX_SOCKET_BUFFER);
    nRet = RecvJuderData(hSocket, pRecvBuffer);
    if (nRet != 0)
    {
        return nRet;
    }
    // 
    TOKEN_RESULT      stResult;
    nRet = ParserTokenResult(pRecvBuffer + SOCKET_HEAD_LEN, &stResult);
    if (nRet != 0)
    {
        return 0;
    }
    // 
    if (stResult.nResult != 0)
    {
        printf("token check error\n");
        return -1;
    }
    // ѡ�������з����������Լ���׼������(Player -> Judger)
    READY_PARAM     stReady;

    strcpy(stReady.szToken, szToken);
    strcpy(stReady.szAction, "ready");

    memset(pSendBuffer, 0, MAX_SOCKET_BUFFER);
    nRet = CreateReadyParam(&stReady, pSendBuffer, &nLen);
    if (nRet != 0)
    {
        return nRet;
    }
    //选手向裁判服务器表明自己已准备就绪(Player -> Judger)
    nRet = SendJuderData(hSocket, pSendBuffer, nLen);
    if (nRet != 0)
    {
        return nRet;
    }
    //对战开始通知,此数据接受包中含有地图信息(Judger -> Player) 
    memset(pRecvBuffer, 0, MAX_SOCKET_BUFFER);
    nRet = RecvJuderData(hSocket, pRecvBuffer);
    if (nRet != 0)
    {
        return nRet;
    }
// ��������
    //Mapinfo �ṹ�����ܴܺ󣬲�̫�ʺϷ���ջ�У����Զ���Ϊȫ�ֻ����ڴ�����
    MAP_INFO            *pstMapInfo;
    MATCH_STATUS        *pstMatchStatus;
    FLAY_PLANE          *pstFlayPlane;

    pstMapInfo = (MAP_INFO *)malloc(sizeof(MAP_INFO));
    if (pstMapInfo == NULL)
    {
        return -1;
    }

    pstMatchStatus = (MATCH_STATUS *)malloc(sizeof(MATCH_STATUS));
    if (pstMapInfo == NULL)
    {
        return -1;
    }

    pstFlayPlane = (FLAY_PLANE *)malloc(sizeof(FLAY_PLANE));
    if (pstFlayPlane == NULL)
    {
        return -1;
    }

    memset(pstMapInfo, 0, sizeof(MAP_INFO));
    memset(pstMatchStatus, 0, sizeof(MATCH_STATUS));
    memset(pstFlayPlane, 0, sizeof(FLAY_PLANE));

    nRet = ParserMapInfo(pRecvBuffer + SOCKET_HEAD_LEN, pstMapInfo);
    if (nRet != 0)
    {
        return nRet;
    }

    mapWidth = pstMapInfo->nMapX; 
    mapLength = pstMapInfo->nMapY;


    homeLocation.x = pstMapInfo->nParkingX;
    homeLocation.y = pstMapInfo->nParkingY;
    homeLocation.z = 0;

    mapHLow = pstMapInfo->nHLow;
    mapHHigh = pstMapInfo->nHHigh;

    serachPathOfZ = pstMapInfo->nHLow;                         //初始高度为地图最低飞行高度

    parkingAapron.x = pstMapInfo->nParkingX;
    parkingAapron.y = pstMapInfo->nParkingY;
    //保存无人机价格表
    uavPriceNum = pstMapInfo->nUavPriceNum;
    for(int i=0;i < uavPriceNum;i++)
    {
        UAVPriceList[i] = pstMapInfo->astUavPrice[i];
    }
    qsort(UAVPriceList,uavPriceNum,sizeof(UAVPriceList[0]),UAVPriceListContent); 
    PrintUavPriceList(UAVPriceList,pstMapInfo);
    //根据接收到的地图数据包信息设置地图
    CreatThreeDMap(pstMapInfo);    

    //flayplane
    pstFlayPlane->nPurchaseNum = 0;
    pstFlayPlane->nUavNum = pstMapInfo->nUavNum;
    for (int i = 0; i < pstMapInfo->nUavNum; i++)
    {
        pstFlayPlane->astUav[i] = pstMapInfo->astUav[i];
    }    
    while (1)
    {
        startTime =clock();
        
        if (pstMatchStatus->nTime != 0)                                          //相当于第一次进入该函数的flag
        {    
            
            AlgorithmCalculationFun(pstMapInfo, pstMatchStatus, pstFlayPlane);     //目前执行时间最大耗时1.2ms
           
        }
       
        memset(pSendBuffer, 0, MAX_SOCKET_BUFFER);
        nRet = CreateFlayPlane(pstFlayPlane, szToken, pSendBuffer, &nLen);
        if (nRet != 0)
        {
            return nRet;
        }
        
        nRet = SendJuderData(hSocket, pSendBuffer, nLen);
        if (nRet != 0)
        {
            return nRet;
        }      
        //printf("send:%s\n",pSendBuffer);
        memset(pRecvBuffer, 0, MAX_SOCKET_BUFFER);

        nRet = RecvJuderData(hSocket, pRecvBuffer);     
        if (nRet != 0)
        {
            return nRet;
        }      
        //printf("recive:%s\n",pRecvBuffer);
        nRet = ParserMatchStatus(pRecvBuffer + SOCKET_HEAD_LEN, pstMatchStatus);
        
        if (nRet != 0)
        {
            return nRet;
        }
        totalObtainedGoodValue = pstMatchStatus->nWeValue;
        finishTime =clock();
        printf("duration=%fs\n",(double)(finishTime-startTime)/CLOCKS_PER_SEC);

        if (pstMatchStatus->nMacthStatus == 1)
        {
            // 
            printf("game over, we value %d, enemy value %d\n", pstMatchStatus->nWeValue, pstMatchStatus->nEnemyValue);
            return 0;
        }        
    }

    // 
    OSCloseSocket(hSocket);
    // 
    free(pRecvBuffer);
    free(pSendBuffer);
    free(pstMapInfo);
    free(pstMatchStatus);
    free(pstFlayPlane);

    return 0;
}

