/**	@file       CmdParse.h
 *	@note       Hikvision Digital Technology Co., Ltd. All Right Reserved.
 *	@brief		
 *
 *	@author     lipengfei
 *	@date       2018/05/10
 *	@note       ��ʷ��¼��
 *	@note       V1.0.0  
 *	@warning	
 */
#ifndef __CMDPARSE_H__
#define __CMDPARSE_H__

#include "JsonParse.h"
#include "AStar.h"

#define SOCKET_HEAD_LEN          8                      ///< 8���ֽڵ�ͷ������


typedef struct _CONNECT_NOTICE_
{
    char    szNotice[64];
    char    szMsg[128];
}CONNECT_NOTICE;

typedef struct _TOKEN_INFO_
{
    char    szToken[64];
    char    szAction[64];
}TOKEN_INFO;


typedef struct _TOKEN_RESULT_
{
    char    szToken[64];
    char    szNotice[64];
    char    szRoundId[64];
    char    szPalyerId[64];
    int     nResult;
}TOKEN_RESULT;

typedef struct _READY_PARAM_
{
    char    szToken[64];
    char    szAction[64];
}READY_PARAM;



typedef struct _BUILDING_
{
    int     nX;
    int     nY;
    int     nL;
    int     nW;
    int     nH;
}BUILDING;

typedef struct _FOG_
{
    int     nX;
    int     nY;
    int     nL;
    int     nW;
    int     nB;
    int     nT;
}FOG;




#define MAX_BUILDING_NUM        128      
#define MAX_FOG_NUM        128

#define MAX_UAV_NUM         512

#define MAX_UAV_PRICE_NUM    64

#define MAX_GOODS_NUM       256



typedef enum _UAV_STATUS_
{
    UAV_NOMAL = 0,
    UAV_CRASH,          
    UAV_FOG,
    UAV_CHARGE
}UAV_STATUS;

typedef enum _UAV_TASK_STATUS_
{
    UAV_TASK_IDLE = 0,                //空闲状态
    UAV_TASK_ACCEPTED,                //接受到任务,准备执行
    UAV_TASK_EXECUTING,                //开始执行任务
    UAV_TASK_DEATH                    //敢死队状态
}UAV_TASK_STATUS;

typedef enum _GOOD_MATCH_STATUS_
{
    UNMATCHED = 0,                   //只有在空闲时才会是未匹配到的状态
    MATCHED,                         //刚匹配到无人机
    PICKED                           //被无人机捡起来了
}GOOD_MATCH_STATUS;



typedef struct _PATH_
{
    int x;
    int y;
    int z;
}PATH;

typedef struct _GOAL_POINT_
{
    int x;
    int y;
    int z;
}GOAL_POINT;

typedef struct _UAV_
{
    int     nNO;
    char    szType[8];
    int     nX;
    int     nY;
    int     nZ;
    int     nLoadWeight;            ///<
    int     nRemainElectricity;
    UAV_STATUS  nStatus;            //环境状态
    int     nGoodsNo;
}UAV;

//Added by wenying
typedef struct   _UAVS_
{
    int     nNO;    
    int     nEnemyUavNO;                //需要追踪的敌人无人机编号  
    int pathIndex;                     //无人机在飞行过程中每个航点都需要该变量获取
    UAV_TASK_STATUS taskState;         //
    GOOD_MATCH_STATUS goodMatchState;
    int goodsNo;                       //如果没有匹配到则默认为-1值,匹配到了就为对应货物的编号
    int goodsWeight;
    int stepNum;                       //整个路径需要行走的步数
    GOAL_POINT starGgoalPoint;         //货物出现的地点
    GOAL_POINT endGgoalPoint;          //货物需要被放置的点
    PATH pPath[10000];
    int nCharge;                        //充电量
    int nCapacity;                      //电池容量
    UAV baseInfo;
}UAVS;

typedef struct _UAV_PRICE_
{
    char    szType[8];
    int     nLoadWeight;
    int     nValue;
    int     nCapacity;
    int     nCharge;
}UAV_PRICE;

/** @struct
 * 	@brief	
 *	@note
 */
typedef struct _MAP_INFO_
{
    int     nMapX;
    int     nMapY;
    int     nMapZ;
    int     nParkingX;
    int     nParkingY;

    int     nHLow;
    int     nHHigh;

    int     nBuildingNum;
    BUILDING    astBuilding[MAX_BUILDING_NUM];
    int     nFogNum;
    FOG         astFog[MAX_FOG_NUM];
    int     nUavNum;
    UAV     astUav[MAX_UAV_NUM];
    int     nUavPriceNum;
    UAV_PRICE   astUavPrice[MAX_UAV_PRICE_NUM];
}MAP_INFO;


typedef struct _FLAY_PLANE_
{
    int     nUavNum;
    UAV     astUav[MAX_UAV_NUM];

    int     nPurchaseNum;                               //下一步想要购买的无人机数量及类型
    char    szPurchaseType[MAX_UAV_PRICE_NUM][8];       //此数组只用于存放无人机型号:如F1 F2 F3等
}FLAY_PLANE;

typedef struct _GOODS_
{
    int     nNO;
    int     nStartX;
    int     nStartY;
    int     nEndX;
    int     nEndY;
    int     nWeight;
    int     nValue;
    int     nStartTime;
    int     nRemainTime;
    int     nLeftTime;
    int     nState;
}GOODS;

typedef struct _GOODS_STATE_           //
{
    int nNO;                                  //货物标号
    int UAVNO;                                //匹配到的无人机编号,默认为-1
    int index;                                //存放在数组中的索引
    ASPath pathStartToEnd;                    //记录该货物起点到终点的路径,一出现后就保存好,这样就不需要每次都计算了
    int pathHigh;                             //最短路径所在高度
    int levelStepNum;                         //货物起点到终点的水平距离
    int distanceStartToEnd;
    int needCapacity;                         //需要的电量
    GOOD_MATCH_STATUS goodMatchState;    
    GOODS goods;
}GOODS_STATE;

typedef struct _MATCH_STATUS_
{
    int     nTime;
    int     nMacthStatus;
    int     nUavWeNum;
    UAV     astWeUav[MAX_UAV_NUM];
    int     nWeValue;
    int     nUavEnemyNum;
    UAV     astEnemyUav[MAX_UAV_NUM];
    int     nEnemyValue;
    int     nGoodsNum;
    GOODS   astGoods[MAX_GOODS_NUM];
}MATCH_STATUS;

/** @fn     int ParserConnect(char *pBuffer, CONNECT_NOTICE *pstNotice)
 *  @brief	
 *	@param  -I   - char * pBuffer
 *	@param  -I   - CONNECT_NOTICE * pstNotice
 *	@return int
 */
int ParserConnect(char *pBuffer, CONNECT_NOTICE *pstNotice);


/** @fn     int ParserTokenResult(char *pBuffer, TOKEN_RESULT *pResult)
 *  @brief	
 *	@param  -I   - char * pBuffer
 *	@param  -I   - TOKEN_RESULT * pResult
 *	@return int
 */
int ParserTokenResult(char *pBuffer, TOKEN_RESULT *pResult);


/** @fn     int ParserMapInfo(char *pBuffer, MAP_INFO *pstMap)
 *  @brief	
 *	@param  -I   - char * pBuffer
 *	@param  -I   - MAP_INFO * pstMap
 *	@return int
 */
int ParserMapInfo(char *pBuffer, MAP_INFO *pstMap);


/** @fn     int ParserUav(cJSON *pUavArray, UAV *astUav, int *pNum)
 *  @brief	
 *	@param  -I   - cJSON * pUavArray
 *	@param  -I   - UAV * astUav
 *	@param  -I   - int * pNum
 *	@return int
 */
int ParserUav(cJSON *pUavArray, UAV *astUav, int *pNum);

/** @fn     int ParserMatchStatus(char *pBuffer, MATCH_STATUS *pstStatus)
 *  @brief	
 *	@param  -I   - char * pBuffer
 *	@param  -I   - MATCH_STATUS * pstStatus
 *	@return int
 */
int ParserMatchStatus(char *pBuffer, MATCH_STATUS *pstStatus);


/** @fn     int CreateTokenInfo(TOKEN_INFO *pstInfo, char *pBuffer)
 *  @brief	
 *	@param  -I   - TOKEN_INFO * pstInfo
 *	@param  -I   - char * pBuffer
 *	@return int
 */
int CreateTokenInfo(TOKEN_INFO *pstInfo, char *pBuffer, int *pLen);

/** @fn     int CreateReadyParam(READY_PARAM *pstParam, char *pBuffer, int *pLen)
 *  @brief	
 *	@param  -I   - READY_PARAM * pstParam
 *	@param  -I   - char * pBuffer
 *	@param  -I   - int * pLen
 *	@return int
 */
int CreateReadyParam(READY_PARAM *pstParam, char *pBuffer, int *pLen);


/** @fn     int CreateFlayPlane(FLAY_PLANE *pstPlane, char *pBuffer, int *pLen)
 *  @brief	
 *	@param  -I   - FLAY_PLANE * pstPlane
 *	@param  -I   - char * pBuffer
 *	@param  -I   - int * pLen
 *	@return int
 */
int CreateFlayPlane(FLAY_PLANE *pstPlane, char *szToken, char *pBuffer, int *pLen);

#endif

