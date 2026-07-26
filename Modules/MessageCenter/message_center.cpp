#include "message_center.h"
#include <cstdlib>
#include <cstring>
#include "bsp_log.h"

Publisher *Publisher::instances_[MAX_TOPIC_CNT];
uint8_t Publisher::count_ = 0;
Subscriber *Subscriber::pending_head_ = nullptr;

/**
 * @brief 在已注册的发布者数组中按主题名查找发布者
 * @param topic 主题名称
 * @return 匹配的发布者指针，未找到时返回 nullptr
 */
Publisher *Publisher::find_(const char *topic) {
    for (uint8_t i = 0; i < count_; i++) {
        if (strcmp(instances_[i]->topic_, topic) == 0) {
            return instances_[i];
        }
    }
    return nullptr;
}

/**
 * @brief 将一个订阅者接入本发布者的订阅链表尾部，并校验数据长度一致
 * @param sub 待绑定的订阅者
 */
void Publisher::link_subscriber_(Subscriber *sub) {
    if (data_len_ != sub->data_len_) {
        LOG_ERROR("Data len mismatch");
        for (;;);
    }
    sub->publisher_ = this;
    sub->next_ = nullptr;
    if (first_subscriber_ == nullptr) {
        first_subscriber_ = sub;
        return;
    }
    Subscriber *iter = first_subscriber_;
    while (iter->next_) {
        iter = iter->next_;
    }
    iter->next_ = sub;
}

/**
 * @brief Publisher 构造函数，接受一个主题名称和数据长度参数，将该实例加入静态实例数组，
 *        并认领此前因发布者尚未注册而挂起等待的同主题订阅者（使初始化顺序不再重要）
 * @param topic 主题名称，字符串类型，最大长度为 TOPIC_NAME_LEN
 * @param data_len 数据长度，表示每条消息的数据大小，以字节为单位
 */
Publisher::Publisher(const char *topic, const uint16_t data_len) : data_len_(data_len), first_subscriber_(nullptr) {
    if (count_ >= MAX_TOPIC_CNT) {
        LOG_ERROR("Topic overflow");
        for (;;);
    }
    strncpy(topic_, topic, TOPIC_NAME_LEN);
    topic_[TOPIC_NAME_LEN] = '\0';
    instances_[count_++] = this;

    /* 认领挂起链表中主题匹配的订阅者：先从挂起链表摘除，再接入本发布者的订阅链表 */
    Subscriber **pp = &Subscriber::pending_head_;
    while (*pp) {
        Subscriber *sub = *pp;
        if (strcmp(sub->topic_, topic_) == 0) {
            *pp = sub->next_;
            link_subscriber_(sub);
        } else {
            pp = &sub->next_;
        }
    }
}

/**
 * @brief 发布消息函数，接受一个指向数据的指针，并将该数据复制到所有订阅者的消息队列中
 * @param data 指向要发布的数据的指针，数据大小应与构造函数中指定的数据长度相匹配
 */
void Publisher::publish(const void *data) const {
    Subscriber *sub = first_subscriber_;

    while (sub) {
        if (sub->size_ == Subscriber::QUEUE_SIZE) {
            sub->front_ = (sub->front_ + 1) % Subscriber::QUEUE_SIZE;
            sub->size_--;
        }
        memcpy(sub->queue_[sub->back_], data, data_len_);
        sub->back_ = (sub->back_ + 1) % Subscriber::QUEUE_SIZE;
        sub->size_++;
        sub = sub->next_;
    }
}

/**
 * @brief Subscriber 构造函数，接受一个主题名称和数据长度参数。若对应主题的发布者已注册则立即绑定，
 *        否则将自身挂起到等待链表，待该主题的发布者注册时被认领，从而不依赖发布者/订阅者的构造顺序
 * @param topic 主题名称，字符串类型，最大长度为 Publisher::TOPIC_NAME_LEN
 * @param data_len 数据长度，表示每条消息的数据大小，以字节为单位
 */
Subscriber::Subscriber(const char *topic, const uint16_t data_len) : front_(0), back_(0), size_(0), data_len_(data_len),
                                                               next_(nullptr), publisher_(nullptr) {
    strncpy(topic_, topic, Publisher::TOPIC_NAME_LEN);
    topic_[Publisher::TOPIC_NAME_LEN] = '\0';
    for (auto &i: queue_) {
        i = malloc(data_len_);
    }

    if (Publisher *pub = Publisher::find_(topic_)) {
        pub->link_subscriber_(this);   /* 发布者已存在，立即绑定 */
    } else {
        next_ = pending_head_;         /* 发布者尚未注册，挂起等待被认领 */
        pending_head_ = this;
    }
}

/**
 * @brief 接收消息函数，从订阅者的消息队列中取出一条消息并复制到提供的缓冲区中
 * @param data 指向接收数据的缓冲区的指针，缓冲区大小应与构造函数中指定的数据长度相匹配
 * @return 如果成功接收到消息返回 true，否则返回 false（例如队列为空或尚未绑定到发布者时）
 */
bool Subscriber::receive(void *data) {
    if (size_ == 0) {
        return false;
    }
    memcpy(data, queue_[front_], data_len_);
    front_ = (front_ + 1) % QUEUE_SIZE;
    size_--;
    return true;
}
