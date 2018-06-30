
#include "AStar.h"
#include <math.h>
#include <string.h>
#include <stdint.h>

struct __ASNeighborList 
{
    const ASPathNodeSource *source;           //包含有函数地址的结构体
    size_t capacity;
    size_t count;
    float *costs;
    void *nodeKeys;
};

struct __ASPath 
{                            //创建的路径节点信息
    size_t nodeSize;              
    size_t count;
    float cost;
    int8_t nodeKeys[];
};


typedef struct 
{             
    unsigned isClosed:1;                       //已被访问节点
    unsigned isOpen:1;                         //未被访问节点
    unsigned isGoal:1;                         //是否是目标节点
    unsigned hasParent:1;                      //是否有父节点
    unsigned hasEstimatedCost:1;               //是否已经经过代价估计
    float estimatedCost;                       //从指定的方格移动到目标点的估算成本
    float cost;                                //从起点移动到指点方格的代价 (以上两者之和就是总的预估代价)
    size_t openIndex;
    size_t parentIndex;
    int8_t nodeKey[];                          //里面存放的是该节点具体的坐标信息,当前为数组数据类型,后面会将其强制转换为结构体数据类型
} NodeRecord;


struct __VisitedNodes 
{                      //已访问节点信息保存结构体
    const ASPathNodeSource *source;          //包含有函数地址的结构体
    void *context;        
    size_t nodeRecordsCapacity;              //节点记录容量
    size_t nodeRecordsCount;                 //统计未被访问节点个数
    void *nodeRecords;
    size_t *nodeRecordsIndex;           // 
    size_t openNodesCapacity;
    size_t openNodesCount;              //open集合中节点数
    size_t *openNodes;                  //根据F的大小进行排序
};
typedef struct __VisitedNodes *VisitedNodes;        

typedef struct 
{
    VisitedNodes nodes;                   //已访问节点
    size_t index;                         //已访问节点索引
} Node;

size_t initIndex = -1;
static const Node NodeNull = {NULL, initIndex};   //初始化空节点

/********************************************/

static inline VisitedNodes VisitedNodesCreate(const ASPathNodeSource *source, void *context)    //分配一个内存空间用于保存已访问的节点
{
    VisitedNodes nodes = (__VisitedNodes*)calloc(1, sizeof(struct __VisitedNodes));
    nodes->source = source;
    nodes->context = context;
    return nodes;
}

static inline void VisitedNodesDestroy(VisitedNodes visitedNodes)    //释放已拜访节点内存
{
    free(visitedNodes->nodeRecordsIndex);
    free(visitedNodes->nodeRecords);
    free(visitedNodes->openNodes);
    free(visitedNodes);
}

static inline int NodeIsNull(Node n)                                //判断当前节点是否为空节点,返回1说明是空节点
{
    return (n.nodes == NodeNull.nodes) && (n.index == NodeNull.index);
}

static inline Node NodeMake(VisitedNodes nodes, size_t index)      //生成一个节点 构造一个节点
{
    return (Node){nodes, index};
}

static inline NodeRecord *NodeGetRecord(Node node)                 //获取节点记录表,返回的是一个结构体
{
    return (NodeRecord *)(node.nodes->nodeRecords + (node.index * (node.nodes->source->nodeSize + sizeof(NodeRecord))));
}

static inline void *GetNodeKey(Node node)                         //获取当前节点具体坐标信息结构体   
{
    return NodeGetRecord(node)->nodeKey;
}

static inline int NodeIsInOpenSet(Node n)        //判断当前节点是否在未被访问节点集合里面
{
    return NodeGetRecord(n)->isOpen;
}

static inline int NodeIsInClosedSet(Node n)    //判断当前节点是否在已被访问节点集合里面
{
    return NodeGetRecord(n)->isClosed;
}

static inline void RemoveNodeFromClosedSet(Node n)  //从已被访问节点集合里面删除节点
{
    NodeGetRecord(n)->isClosed = 0;
}

static inline void AddNodeToClosedSet(Node n)       //将刚访问过的节点加入到被访问节点集合里面
{
    NodeGetRecord(n)->isClosed = 1;
}

static inline float GetNodeRank(Node n)            //获取节点的排列序号
{
    NodeRecord *record = NodeGetRecord(n);
    return record->estimatedCost + record->cost;   //总的预估代价
}

static inline float GetNodeCost(Node n)            //获取由当前节点移动到指定方格的代价
{
    return NodeGetRecord(n)->cost;
}

static inline float GetNodeEstimatedCost(Node n)  //获取有指定方格到目标点的估计代价
{
    return NodeGetRecord(n)->estimatedCost;
}

static inline void SetNodeEstimatedCost(Node n, float estimatedCost)  //对指定点设置其到终点的估计代价
{
    NodeRecord *record = NodeGetRecord(n);
    record->estimatedCost = estimatedCost;
    record->hasEstimatedCost = 1;
}

static inline int NodeHasEstimatedCost(Node n)   //判断该节点是否已有估计代价值
{
    return NodeGetRecord(n)->hasEstimatedCost;
}

static inline void SetNodeIsGoal(Node n)        //将当前节点设置为目标节点(目的地)
{
    if (!NodeIsNull(n)) 
    {
        NodeGetRecord(n)->isGoal = 1;
    }
}

static inline int NodeIsGoal(Node n)            //判断当前节点是否为目的地(目标节点)
{
    return !NodeIsNull(n) && NodeGetRecord(n)->isGoal;
}

static inline Node GetParentNode(Node n)       //获取该节点的父节点
{
    NodeRecord *record = NodeGetRecord(n);
    if (record->hasParent) 
    {
        return NodeMake(n.nodes, record->parentIndex);
    } 
    else 
    {
        return NodeNull;
    }
}

static inline int NodeRankCompare(Node n1, Node n2)   //比较两个节点总的代价 -1:n1<n2 0:n1=n2 1:n1>n2
{
    const float rank1 = GetNodeRank(n1);
    const float rank2 = GetNodeRank(n2);
    if (rank1 < rank2) 
    {
        return -1;
    } else if (rank1 > rank2) 
    {
        return 1;
    } 
    else 
    {
        return 0;
    }
}

static inline float GetPathCostHeuristic(Node a, Node b)      // 计算由指定位置到目标点的预估代价值
{
    if (a.nodes->source->pathCostHeuristic && !NodeIsNull(a) && !NodeIsNull(b)) 
    {
        return a.nodes->source->pathCostHeuristic(GetNodeKey(a), GetNodeKey(b), a.nodes->context);
    } 
    else 
    {
        return 0;
    }
}

static inline int NodeKeyCompare(Node node, void *nodeKey)
{
    if (node.nodes->source->nodeComparator) 
    {
        return node.nodes->source->nodeComparator(GetNodeKey(node), nodeKey, node.nodes->context);
    } 
    else 
    {
        return memcmp(GetNodeKey(node), nodeKey, node.nodes->source->nodeSize);
    }
}

static inline Node GetNode(VisitedNodes nodes, void *nodeKey)    //在节点集合里面查找当前节点是否存在,若不存在,则将该节点插入节点集合中,并返回一个对该节点的引用,其中包含有该节点在整个集合中排序
{
    if (!nodeKey) 
    {               //查找的节点是否为空
        return NodeNull;
    }
    
    size_t first = 0;
    if (nodes->nodeRecordsCount > 0) 
    {
        size_t last = nodes->nodeRecordsCount-1;

        while (first <= last) 
        {                               //折半查找法
            const size_t mid = (first + last) / 2;
            const int comp = NodeKeyCompare(NodeMake(nodes, nodes->nodeRecordsIndex[mid]), nodeKey);

            if (comp < 0) 
            {
                first = mid + 1;
            } else if (comp > 0 && mid > 0) 
            {
                last = mid - 1;
            } 
            else if (comp > 0) 
            {
                break;
            } 
            else 
            {
                return NodeMake(nodes, nodes->nodeRecordsIndex[mid]);                  //如果列表中本身就含有,
            }
        }
    } 

    if (nodes->nodeRecordsCount == nodes->nodeRecordsCapacity) 
    {                      //
        nodes->nodeRecordsCapacity = 1 + (nodes->nodeRecordsCapacity * 2);            //对节点容量进行扩容
        nodes->nodeRecords = realloc(nodes->nodeRecords, nodes->nodeRecordsCapacity * (sizeof(NodeRecord) + nodes->source->nodeSize));  //realloc改变nodes->nodeRecords所指内存大小
        nodes->nodeRecordsIndex = (size_t *)realloc(nodes->nodeRecordsIndex, nodes->nodeRecordsCapacity * sizeof(size_t));              //节点索引数组大小
    }
    
    Node node = NodeMake(nodes, nodes->nodeRecordsCount);
    nodes->nodeRecordsCount++;
    
    memmove(&nodes->nodeRecordsIndex[first+1], &nodes->nodeRecordsIndex[first], (nodes->nodeRecordsCapacity - first - 1) * sizeof(size_t));   //内存拷贝,插入节点
    nodes->nodeRecordsIndex[first] = node.index;             //将该节点索引保存至索引数组中去
    
    NodeRecord *record = NodeGetRecord(node);
    memset(record, 0, sizeof(NodeRecord));
    memcpy(record->nodeKey, nodeKey, nodes->source->nodeSize);    //保存节点坐标

    return node;
}

static inline void SwapOpenSetNodesAtIndexes(VisitedNodes nodes, size_t index1, size_t index2)    //交换open集合中的两个节点索引号
{
    if (index1 != index2) 
    {
        NodeRecord *record1 = NodeGetRecord(NodeMake(nodes, nodes->openNodes[index1]));
        NodeRecord *record2 = NodeGetRecord(NodeMake(nodes, nodes->openNodes[index2]));
        
        const size_t tempOpenIndex = record1->openIndex;
        record1->openIndex = record2->openIndex;
        record2->openIndex = tempOpenIndex;
        
        const size_t tempNodeIndex = nodes->openNodes[index1];
        nodes->openNodes[index1] = nodes->openNodes[index2];
        nodes->openNodes[index2] = tempNodeIndex;
    }
}

static inline void DidRemoveFromOpenSetAtIndex(VisitedNodes nodes, size_t index)     //直接从open集合中移除索引值为:index的节点
{
    size_t smallestIndex = index;
    
    do 
    {
        if (smallestIndex != index) 
        {
            SwapOpenSetNodesAtIndexes(nodes, smallestIndex, index);
            index = smallestIndex;
        }

        const size_t leftIndex = (2 * index) + 1;
        const size_t rightIndex = (2 * index) + 2;
        
        if (leftIndex < nodes->openNodesCount && NodeRankCompare(NodeMake(nodes, nodes->openNodes[leftIndex]), NodeMake(nodes, nodes->openNodes[smallestIndex])) < 0) 
        {
            smallestIndex = leftIndex;
        }
        
        if (rightIndex < nodes->openNodesCount && NodeRankCompare(NodeMake(nodes, nodes->openNodes[rightIndex]), NodeMake(nodes, nodes->openNodes[smallestIndex])) < 0) 
        {
            smallestIndex = rightIndex;
        }
    } while (smallestIndex != index);
}

static inline void RemoveNodeFromOpenSet(Node n)  //如果该节点被访问过,则直接从open集合中移除
{
    NodeRecord *record = NodeGetRecord(n);

    if (record->isOpen) 
    {
        record->isOpen = 0;
        n.nodes->openNodesCount--;
        
        const size_t index = record->openIndex;
        SwapOpenSetNodesAtIndexes(n.nodes, index, n.nodes->openNodesCount);
        DidRemoveFromOpenSetAtIndex(n.nodes, index);
    }
}

static inline void DidInsertIntoOpenSetAtIndex(VisitedNodes nodes, size_t index)   //直接以index方式插入节点到open集合中
{
    while (index > 0) 
    {
        const size_t parentIndex = floorf((index-1) / 2);
        
        if (NodeRankCompare(NodeMake(nodes, nodes->openNodes[parentIndex]), NodeMake(nodes, nodes->openNodes[index])) < 0) 
        {
            break;
        } 
        else 
        {
            SwapOpenSetNodesAtIndexes(nodes, parentIndex, index);
            index = parentIndex;
        }
    }
}

static inline void AddNodeToOpenSet(Node n, float cost, Node parent)    //在open集合中插入未被访问节点
{
    NodeRecord *record = NodeGetRecord(n);

    if (!NodeIsNull(parent)) 
    {
        record->hasParent = 1;
        record->parentIndex = parent.index;
    } 
    else 
    {
        record->hasParent = 0;
    }

    if (n.nodes->openNodesCount == n.nodes->openNodesCapacity) 
    {
        n.nodes->openNodesCapacity = 1 + (n.nodes->openNodesCapacity * 2);
        n.nodes->openNodes = (size_t *)realloc(n.nodes->openNodes, n.nodes->openNodesCapacity * sizeof(size_t));
    }

    const size_t openIndex = n.nodes->openNodesCount;
    n.nodes->openNodes[openIndex] = n.index;
    n.nodes->openNodesCount++;

    record->openIndex = openIndex;
    record->isOpen = 1;
    record->cost = cost;

    DidInsertIntoOpenSetAtIndex(n.nodes, openIndex);
}

static inline int HasOpenNode(VisitedNodes nodes)   //open集合中是否还有节点
{
    return nodes->openNodesCount > 0;
}

static inline Node GetOpenNode(VisitedNodes nodes)  //获取open集合中的节点
{
    return NodeMake(nodes, nodes->openNodes[0]);
}

static inline ASNeighborList NeighborListCreate(const ASPathNodeSource *source) //创建邻接节点列表
{
    ASNeighborList list = (__ASNeighborList *)calloc(1, sizeof(struct __ASNeighborList));
    list->source = source;
    return list;
}

static inline void NeighborListDestroy(ASNeighborList list)      //邻接节点删除
{
    free(list->costs);
    free(list->nodeKeys);
    free(list);
}

static inline float NeighborListGetEdgeCost(ASNeighborList list, size_t index)  //获取邻接节点的总的预估代价
{
    return list->costs[index];
}

static void *NeighborListGetNodeKey(ASNeighborList list, size_t index)          //获取邻接节点的坐标
{
    return list->nodeKeys + (index * list->source->nodeSize);
}

/********************************************/

void ASNeighborListAdd(ASNeighborList list, void *node, float edgeCost)        //以参数节点为中心,添加与其邻接的所有可行节点
{
    if (list->count == list->capacity) 
    {
        list->capacity = 1 + (list->capacity * 2);
        list->costs = (float *)realloc(list->costs, sizeof(float) * list->capacity);
        list->nodeKeys = realloc(list->nodeKeys, list->source->nodeSize * list->capacity);
    }
    list->costs[list->count] = edgeCost;
    memcpy(list->nodeKeys + (list->count * list->source->nodeSize), node, list->source->nodeSize);
    list->count++;
}

ASPath ASPathCreate(const ASPathNodeSource *source, void *context,void *startNodeKey, void *goalNodeKey)   //A*寻路搜索
{
    if (!startNodeKey || !source || !source->nodeNeighbors || source->nodeSize == 0) 
    {
        return NULL;
    }
    
    VisitedNodes visitedNodes = VisitedNodesCreate(source, context);
    ASNeighborList neighborList = NeighborListCreate(source);
    Node current = GetNode(visitedNodes, startNodeKey);
    Node goalNode = GetNode(visitedNodes, goalNodeKey);
    ASPath path = NULL;
    SetNodeIsGoal(goalNode);
    
    SetNodeEstimatedCost(current,  GetPathCostHeuristic(current, goalNode));
    AddNodeToOpenSet(current, 0, NodeNull);                                                         //初始化时该节点就是父节点,其父节点为空地址
    while (HasOpenNode(visitedNodes) && !NodeIsGoal((current = GetOpenNode(visitedNodes)))) 
    {
        if (source->earlyExit) 
        {
            const int shouldExit = source->earlyExit(visitedNodes->nodeRecordsCount, GetNodeKey(current), goalNodeKey, context);

            if (shouldExit > 0) 
            {
                SetNodeIsGoal(current);
                break;
            } 
            else if (shouldExit < 0) 
            {
                break;
            }
        }
        
        RemoveNodeFromOpenSet(current);
        AddNodeToClosedSet(current);
        
        neighborList->count = 0;
        source->nodeNeighbors(neighborList, GetNodeKey(current), context);

        for (size_t n=0; n<neighborList->count; n++) 
        {
            const float cost = GetNodeCost(current) + NeighborListGetEdgeCost(neighborList, n);
            Node neighbor = GetNode(visitedNodes, NeighborListGetNodeKey(neighborList, n));
            
            if (!NodeHasEstimatedCost(neighbor)) 
            {                        //设置该邻接节点到目标点的预估代价
                SetNodeEstimatedCost(neighbor, GetPathCostHeuristic(neighbor, goalNode));
            }
            
            if (NodeIsInOpenSet(neighbor) && cost < GetNodeCost(neighbor)) 
            {   //如果该邻接节点在open集合中,且其代价大于当前节点,则将其从open集合中移除
                RemoveNodeFromOpenSet(neighbor);
            }
            
            if (NodeIsInClosedSet(neighbor) && cost < GetNodeCost(neighbor)) 
            {  //如果该邻接节点在closed集合中,且其代价大于当前节点,则将其从closed集合中移除
                RemoveNodeFromClosedSet(neighbor);
            }
            
            if (!NodeIsInOpenSet(neighbor) && !NodeIsInClosedSet(neighbor)) 
            {   //即不在open集合中也不在closed集合中,说明是一个未被处理过的节点,将其加入open集合中
                AddNodeToOpenSet(neighbor, cost, current);
            }
        }
    }
    
    if (NodeIsNull(goalNode)) 
    {            //如果没有目标节点,则将当前节点设置为目标节点
        SetNodeIsGoal(current);
    }
    
    if (NodeIsGoal(current)) 
    {             //如果当前节点是目标节点
        size_t count = 0;
        Node n = current;
        
        while (!NodeIsNull(n)) 
        {
            count++;
            n = GetParentNode(n);
        }
        //printf("count=%d\n",count);
        path = (__ASPath *)malloc(sizeof(struct __ASPath) + (count * source->nodeSize));
        path->nodeSize = source->nodeSize;
        path->count = count;
        path->cost = GetNodeCost(current);
        n = current;
        for (size_t i=count; i>0; i--) 
        {
            memcpy(path->nodeKeys + ((i - 1) * source->nodeSize), GetNodeKey(n), source->nodeSize);           
            n = GetParentNode(n);
        }
    }    
    
    NeighborListDestroy(neighborList);
    VisitedNodesDestroy(visitedNodes);

    return path;
}
void ASPathDestroy(ASPath path)
{
    free(path);
}

ASPath ASPathCopy(ASPath path)
{
    if (path) 
    {
        const size_t size = sizeof(struct __ASPath) + (path->count * path->nodeSize);
        ASPath newPath = (__ASPath *)malloc(size);
        memcpy(newPath, path, size);
        return newPath;
    } 
    else 
    {
        return NULL;
    }
}

float ASPathGetCost(ASPath path)
{
    return path? path->cost : INFINITY;
}

size_t ASPathGetCount(ASPath path)
{
    return path? path->count : 0;
}

void *ASPathGetNode(ASPath path, size_t index)
{
    return (path && index < path->count)? (path->nodeKeys + (index * path->nodeSize)) : NULL;
}
