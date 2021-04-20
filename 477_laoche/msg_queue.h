#ifndef MESSAGE_QUEUE_H
#define MESSAGE_QUEUE_H

#include <map>
#include <vector>

#include <pthread.h>

#include "msg_channel_index.h"
#include "common/thpool.h"

using std::map;
using std::vector;

class MsgQueueListener {
public:
    /**
     * 处理某个channel上发送的消息
     * 参数指向的内存区域保证在函数返回前有效
     * @param
     *     channel 频道编号
     *     buf 消息内容的首地址
     *     size 内容长度，单位byte
     */
    virtual void recieve_message(const int channel, char *buf, const int size) = 0;
};

class MsgSharedMemory {
public:
    MsgSharedMemory() {
        pthread_mutex_init(&mutex, nullptr);
    };
    ~MsgSharedMemory() {
        pthread_mutex_destroy(&mutex);
    }
    pthread_mutex_t mutex;
    int count = 0;
    std::vector<char> data;
    void increase(int i) {
        pthread_mutex_lock(&mutex);
        count += i;
        pthread_mutex_unlock(&mutex);
    };
    bool free(int i) {
        bool ret = false;
        pthread_mutex_lock(&mutex);
        count -= i;
        ret = (count < 1);
        pthread_mutex_unlock(&mutex);
        return ret;
    }
};

class MsgQueueEntry {
public:
    MsgQueueEntry(int c, MsgQueueListener* p) :channel(c), listener(p) {};
    int channel;
    MsgQueueListener * listener;
    bool equals(MsgQueueEntry & e2) {
        return ((channel == e2.channel) && (listener == e2.listener));
    };
};

struct message_param {
    MsgQueueListener *p;
    MsgSharedMemory *m;
    int channel;
};
void message_queue_handle_message(void *param);

/**
 * 异步消息队列
 */
class MsgQueue {
public:
    MsgQueue(int thread_num);
    ~MsgQueue();

    /**
     * 将某个listener绑定到给定的频道编号上
     * 该linstener会收到所有发到该频道上的信息
     * 原则上不允许重复绑定
     * @param
     *     channel 接听的频道编号
     *     p_listener 接听者对象的指针
     * @return
     *     listener_id 为本次绑定分配的唯一编号
     */
    int add_listener(const int channel, MsgQueueListener * p_listener);

    /**
     * 根据绑定listener时分配的id移除该绑定
     * 移除后listener将不再收到此次绑定的频道上的消息
     * @param
     *     listener_id 执行add_listener函数时返回的ID
     */
    void remove_listener(const int listener_id);

    /**
     * 移除后listener将不再收到此次绑定的频道上的消息
     * @param
     *     channel 接听的频道编号
     *     p_listener 接听者对象的指针
     */
    void remove_listener(const int channel, MsgQueueListener * p_listener);

    /**
     * 清空所有绑定信息
     */
    void clear();

    /**
     * 向每个接听该频道的listener发送消息
     * 为每个listener启动一个单独的线程进行处理
     * @param
     *     channel 发送到的频道编号
     *     buf 要发送的消息在内存中的位置
     *     size 要发送的消息的长度
     */
    void send(int channel, char * buf, int size);

    void print_debug_string();

private:
    pthread_mutex_t mutex;

    map<int, vector<MsgQueueListener*> > chann_lis;
    map<int, MsgQueueEntry> id_entry;
    int id_allocated = 1;

    threadpool p_thpool;
};

#endif
