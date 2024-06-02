#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <assert.h>
#include <opencv2/imgproc/imgproc.hpp>
class Queueclass
{
private:
public:
    /* data */
    // typedef int QDateType; //队列存储数据类型
    typedef cv::Point QDateType; // 队列存储数据类型

    typedef struct QueueNode // 队列元素节点
    {
        QDateType val;
        struct QueueNode *next;
    } QueueNode;

    typedef struct Queue // 队列
    {
        QueueNode *head;
        QueueNode *tail;
    } Queue;
    Queue pq;
    void QueueInti(Queue *pq);
    // 队列初始化
    void QueueDestory(Queue *pq);
    // 队列的销毁
    void QueuePush(Queue *pq, QDateType x);
    // 入队
    void QueuePop(Queue *pq);
    // 出队
    QDateType QueueFront(Queue *pq);
    // 取出队首元素
    int QueueSize(Queue *pq);
    // 求队列的长度
    bool QueueEmpty(Queue *pq);
    // 判断队是否为空

    Queueclass(/* args */);
    ~Queueclass();
};

Queueclass::Queueclass(/* args */)
{
}

Queueclass::~Queueclass()
{
}
// 1.队列的初始化
//     将头尾置为空指针即可。
void Queueclass::QueueInti(Queue *pq)
{
    assert(pq); // 防止pq为空指针
    pq->head = pq->tail = NULL;
}
// 2.队列的销毁
//     遍历队列元素，然后将每一个元素释放。
void Queueclass::QueueDestory(Queue *pq)
{
    assert(pq); // 防止pq为空指针
    QueueNode *cur = pq->head;
    while (cur)
    {
        QueueNode *next = cur->next;
        free(cur);
        cur = next;
    }
    pq->tail = pq->head = NULL;
}

// 3.入队
// 对于入队，我们首先需要去开辟一个新的节点来存储数据，然后将这个节点加入到tail后即可。此时我们就要分别考虑。
//     如果为空队列，那么我们不仅要改变tail，还要改变head的值
//         如果不为空队列，只用改变tail即可。
void Queueclass::QueuePush(Queue *pq, QDateType x)
{
    assert(pq); // 防止pq为空指针

    QueueNode *newNode = (QueueNode *)malloc(sizeof(QueueNode));
    if (NULL == newNode)
    {
        printf("malloc error\n");
        exit(-1);
    }
    newNode->val = x;
    newNode->next = NULL; // 开辟一个新节点存储数据

    if (pq->tail == NULL) // 判断是否为空队列
    {
        assert(pq->head == NULL);
        pq->head = pq->tail = newNode;
    }
    else
    {
        pq->tail->next = newNode;
        pq->tail = newNode;
    }
}

// 4.出队  对于出队，我们同样需要考虑两种情况
// 队列为空，改变head的同时改变tail
//     队列不为空，改变head即可。
void Queueclass::QueuePop(Queue *pq)
{
    assert(pq);                   // 防止pq为空指针
    assert(pq->head && pq->tail); // 防止队列为空队列
    if (pq->head->next == NULL)
    {
        free(pq->head);
        pq->head = pq->tail = NULL;
    }
    else
    {
        QueueNode *next = pq->head->next;
        free(pq->head);
        pq->head = next;
    }
}

// 5. 取出队首元素
//     没啥说的，直接访问头节点取出即可
Queueclass::QDateType Queueclass::QueueFront(Queue *pq)
{
    assert(pq);                   // 防止pq为空指针
    assert(pq->head && pq->tail); // 防止队列为空队列
    return pq->head->val;
}

// 6.判断是否为空队列
//     我们只需要判断头指针是否为NULL，如果是则为空
bool Queueclass::QueueEmpty(Queue *pq)
{
    assert(pq);

    return pq->head == NULL;
}

// 7. 求队伍长度
//     创建一个变量，遍历队伍求长度。
int Queueclass::QueueSize(Queue *pq)
{
    assert(pq);
    QueueNode *cur = pq->head;
    int count = 0;
    while (cur)
    {
        cur = cur->next;
        count++;
    }
    return count;
}
