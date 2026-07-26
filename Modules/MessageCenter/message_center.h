#ifndef MERCURYCORE_MESSAGE_CENTER_H
#define MERCURYCORE_MESSAGE_CENTER_H

#pragma once

#include <cstdint>

class Subscriber;

class Publisher {
public:
    Publisher(const char *topic,uint16_t data_len);
    void publish(const void *data) const;

private:
    friend class Subscriber;
    static constexpr uint8_t TOPIC_NAME_LEN = 32;
    char topic_[TOPIC_NAME_LEN + 1]{};
    uint16_t data_len_;
    Subscriber *first_subscriber_;
    static constexpr uint8_t MAX_TOPIC_CNT = 32;
    static Publisher *instances_[MAX_TOPIC_CNT];
    static uint8_t count_;

    static Publisher *find_(const char *topic);   /* 在已注册的发布者中按主题查找，未找到返回 nullptr */
    void link_subscriber_(Subscriber *sub);        /* 将订阅者接入本发布者的订阅链表（含数据长度校验） */
};


class Subscriber {
public:
    Subscriber(const char *topic,uint16_t data_len);
    bool receive(void *data);

private:
    friend class Publisher;
    static constexpr uint8_t QUEUE_SIZE = 1;
    char topic_[Publisher::TOPIC_NAME_LEN + 1]{};   /* 保存主题名，供发布者后注册时延迟认领 */
    void *queue_[QUEUE_SIZE]{};
    uint8_t front_;
    uint8_t back_;
    uint8_t size_;
    uint16_t data_len_;
    Subscriber *next_;
    Publisher *publisher_;
    static Subscriber *pending_head_;               /* 尚未匹配到发布者的订阅者挂起链表（按主题延迟绑定） */
};

#endif
