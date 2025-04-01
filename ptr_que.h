/* Copyright 2025 国家地方共建人形机器人创新中心/人形机器人（上海）有限公司
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Designed and built with love @zhihu by @cjrcl.
 */

#pragma once

#include <stdio.h>
#include <pthread.h>
#include <semaphore.h>
#include <deque>

#define MAX_QUE_SIZE 1024
#define QUE_PRI_LOW 0
#define QUE_PRI_HIGH 1

template<typename T, int const N = MAX_QUE_SIZE>
class PtrQue{
private:
    std::deque<T*> queue;
    size_t maxQueueSize;
    pthread_mutex_t mutex;
    sem_t semWrite, semRead;
    int mutexInit(pthread_mutex_t* mutex){
        pthread_mutexattr_t mutexAttribute;
        pthread_mutexattr_init(&mutexAttribute);
        if(pthread_mutexattr_setrobust(&mutexAttribute, PTHREAD_MUTEX_ROBUST) == 0){
            if(pthread_mutexattr_settype(&mutexAttribute, PTHREAD_MUTEX_ERRORCHECK) == 0){
                if(pthread_mutex_init(mutex, &mutexAttribute) == 0){
                    printf("mutex initialized\n");
                    pthread_mutexattr_destroy(&mutexAttribute);
                    return 0;
                }else{
                    printf("pthread_mutex_init() failed\n");
                }
            }else{
                printf("pthread_mutexattr_settype() failed\n");
            }
        }else{
            printf("pthread_mutexattr_setrobust_np() failed\n");
        }
        pthread_mutexattr_destroy(&mutexAttribute);
        return -1;
    }
public:
    PtrQue(){
        maxQueueSize = N;
        mutexInit(&mutex);
        sem_init(&semWrite, 0, maxQueueSize);
        sem_init(&semRead, 0, 0);
    }
    void put(T* msg, int const priority = QUE_PRI_LOW){
        sem_wait(&semWrite);
        pthread_mutex_lock(&mutex);
        if(priority == QUE_PRI_LOW){
            queue.push_back(msg);
        }else if(priority == QUE_PRI_HIGH){
            queue.push_front(msg);
        }
        pthread_mutex_unlock(&mutex);
        sem_post(&semRead);
    }
    T* get(){
        T* msg = nullptr;
        sem_wait(&semRead);
        pthread_mutex_lock(&mutex);
        auto it = queue.begin();
        if(it != queue.end()){
            msg = *it;
            queue.pop_front();
        }
        pthread_mutex_unlock(&mutex);
        sem_post(&semWrite);
        return msg;
    }
    T* get_nonblocking(){
        T* msg = nullptr;
        if(sem_trywait(&semRead) == 0){
            pthread_mutex_lock(&mutex);
            auto it = queue.begin();
            if(it != queue.end()){
                msg = *it;
                queue.pop_front();
            }
            pthread_mutex_unlock(&mutex);
            sem_post(&semWrite);
        }
        return msg;
    }
    size_t size(){
        size_t size = 0;
        pthread_mutex_lock(&mutex);
        size = queue.size();
        pthread_mutex_unlock(&mutex);
        return size;
    }
    size_t maxSize(){
        return maxQueueSize;
    }
    ~PtrQue(){
        sem_destroy(&semRead);
        sem_destroy(&semWrite);
        pthread_mutex_destroy(&mutex);
    }
};