#include <stdio.h>

#include "msg_queue.h"

MsgQueue::MsgQueue(int thread_num) {
    p_thpool = thpool_init(thread_num);
    pthread_mutex_init(&mutex, nullptr);
};

MsgQueue::~MsgQueue() {
    thpool_destroy(p_thpool);
    pthread_mutex_destroy(&mutex);
}

int MsgQueue::add_listener(const int channel, MsgQueueListener * p_listener) {
    pthread_mutex_lock(&mutex);
    int ret = -1;

    auto search1 = chann_lis.find(channel);
    bool exist = false;
    if(search1 == chann_lis.end()) {
        vector<MsgQueueListener*> tmp;
        chann_lis.insert(std::pair<int, vector<MsgQueueListener*>>(channel, tmp));
        search1 = chann_lis.find(channel);
    }
    else {
        for(MsgQueueListener *p : search1->second) {
            if(p == p_listener) {
                exist = true;
                break;
            }
        }
    }
    if(!exist) {
        search1->second.push_back(p_listener);
        ret = id_allocated ++;
        MsgQueueEntry entry(channel, p_listener);
        id_entry.insert(std::pair<int, MsgQueueEntry>(ret, entry));
    }

    pthread_mutex_unlock(&mutex);
    return ret;
}

void MsgQueue::remove_listener(const int listener_id) {
    pthread_mutex_lock(&mutex);

    auto search1 = id_entry.find(listener_id);
    if(search1 != id_entry.end()) {
        MsgQueueEntry entry = search1->second;
        auto search2 = chann_lis.find(entry.channel);
        if(search2 != chann_lis.end()) {
            for(auto it = search2->second.begin(); it != search2->second.end(); it++) {
                if(*it == entry.listener) {
                    search2->second.erase(it);
                    break;
                }
            }
            if(search2->second.empty()) {
                chann_lis.erase(search2);
            }
        }
        id_entry.erase(search1);
    }

    pthread_mutex_unlock(&mutex);
}

void MsgQueue::remove_listener(const int channel, MsgQueueListener * p_listener) {
    pthread_mutex_lock(&mutex);
    int id = -1;
    MsgQueueEntry entry(channel, p_listener);
    for(auto &e : id_entry) {
        if(entry.equals(e.second)) {
            id = e.first;
            break;
        }
    }
    pthread_mutex_unlock(&mutex);
    if(id > 0) {
        remove_listener(id);
    }
}

void MsgQueue::clear() {
    pthread_mutex_lock(&mutex);
    chann_lis.clear();
    id_entry.clear();
    pthread_mutex_unlock(&mutex);
}

void MsgQueue::send(int channel, char * buf, const int size) {

    if(channel == CHANNEL_IGNORE) {
        return;
    }
    
    pthread_mutex_lock(&mutex);
    auto search1 = chann_lis.find(channel);
    if(search1 != chann_lis.end()) {
        MsgSharedMemory *mem = new MsgSharedMemory();
        mem->data.reserve(size);
        for(int i = 0;i <size;i++) {
            mem->data.push_back(buf[i]);
        }
        mem->increase(search1->second.size());
        for(auto iter = search1->second.begin(); iter != search1->second.end(); iter++) {
            MsgQueueListener *p = *iter;
            struct message_param * param = new struct message_param();
            param->p = p;
            param->m = mem;
            param->channel = channel;
            thpool_add_work(p_thpool, message_queue_handle_message, (void*)param);
        }
    }
    pthread_mutex_unlock(&mutex);
}

void message_queue_handle_message(void *param) {
    struct message_param *par = (struct message_param *) param;
    MsgQueueListener *p_listener = par->p;
    MsgSharedMemory *mem = par->m;
    int channel = par->channel;
    delete param;

    p_listener->recieve_message(channel, mem->data.data(), mem->data.size());

    if(mem->free(1)) {
        delete mem;
    }
}

void MsgQueue::print_debug_string() {
    pthread_mutex_lock(&mutex);
    printf("[DEBUG PRINT]\n");
    for(auto & entry : chann_lis) {
        printf("Channel %4d :\n", entry.first);
        for(MsgQueueListener* p : entry.second) {
            printf("\t %ld :\n", p);
        }
    }
    printf("ID list:\n");
    for(auto & entry : id_entry) {
        printf("\t<%d, %d> :\n", entry.first, entry.second.channel);
    }
    printf("[DEBUG PRINT END]\n");
    pthread_mutex_unlock(&mutex);
}


