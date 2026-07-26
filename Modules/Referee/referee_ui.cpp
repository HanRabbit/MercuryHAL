#include "referee_ui.h"

#include <cstdarg>
#include <cstdio>
#include <cstring>

/* UI 交互帧结构体(1 字节对齐, 匹配串口字节流), 仅本文件组帧使用 */
#pragma pack(1)

/* 删除图层帧 */
struct UI_Delete_t {
    xFrameHeader frame_header;
    uint16_t cmd_id;
    ext_student_interactive_header_data_t datahead;
    uint8_t delete_operate;
    uint8_t layer;
    uint16_t frametail;
};

/* 字符图形推送帧 */
struct UI_CharRefresh_t {
    xFrameHeader frame_header;
    uint16_t cmd_id;
    ext_student_interactive_header_data_t datahead;
    String_Data_t string_data;
    uint16_t frametail;
};

/* 图形推送帧头(图形数据段长度可变, 单独拼接在其后) */
struct UI_GraphHeader_t {
    xFrameHeader frame_header;
    uint16_t cmd_id;
    ext_student_interactive_header_data_t datahead;
};

#pragma pack()

/**
 * @brief RefereeUI 构造函数
 * @param referee 关联的裁判系统实例
 */
RefereeUI::RefereeUI(Referee *referee) : referee_(referee) {}

/**
 * @brief 填充图形名(逆序拷贝以匹配客户端字节序)
 */
void RefereeUI::setGraphName(Graph_Data_t &graph, const char name[3]) {
    for (int i = 0; i < 3 && name[i] != '\0'; i++) {
        graph.graphic_name[2 - i] = name[i];
    }
}

/**
 * @brief 构造直线图形
 */
void RefereeUI::lineDraw(Graph_Data_t &graph, const char name[3], const uint32_t operate, const uint32_t layer,
                         const uint32_t color, const uint32_t width,
                         const uint32_t start_x, const uint32_t start_y, const uint32_t end_x, const uint32_t end_y) {
    setGraphName(graph, name);
    graph.operate_tpye = operate;
    graph.graphic_tpye = UI_Graph_Line;
    graph.layer = layer;
    graph.color = color;
    graph.start_angle = 0;
    graph.end_angle = 0;
    graph.width = width;
    graph.start_x = start_x;
    graph.start_y = start_y;
    graph.radius = 0;
    graph.end_x = end_x;
    graph.end_y = end_y;
}

/**
 * @brief 构造矩形图形
 */
void RefereeUI::rectangleDraw(Graph_Data_t &graph, const char name[3], const uint32_t operate, const uint32_t layer,
                              const uint32_t color, const uint32_t width,
                              const uint32_t start_x, const uint32_t start_y, const uint32_t end_x,
                              const uint32_t end_y) {
    setGraphName(graph, name);
    graph.operate_tpye = operate;
    graph.graphic_tpye = UI_Graph_Rectangle;
    graph.layer = layer;
    graph.color = color;
    graph.start_angle = 0;
    graph.end_angle = 0;
    graph.width = width;
    graph.start_x = start_x;
    graph.start_y = start_y;
    graph.radius = 0;
    graph.end_x = end_x;
    graph.end_y = end_y;
}

/**
 * @brief 构造整圆图形
 */
void RefereeUI::circleDraw(Graph_Data_t &graph, const char name[3], const uint32_t operate, const uint32_t layer,
                           const uint32_t color, const uint32_t width,
                           const uint32_t start_x, const uint32_t start_y, const uint32_t radius) {
    setGraphName(graph, name);
    graph.operate_tpye = operate;
    graph.graphic_tpye = UI_Graph_Circle;
    graph.layer = layer;
    graph.color = color;
    graph.start_angle = 0;
    graph.end_angle = 0;
    graph.width = width;
    graph.start_x = start_x;
    graph.start_y = start_y;
    graph.radius = radius;
    graph.end_x = 0;
    graph.end_y = 0;
}

/**
 * @brief 构造椭圆图形
 */
void RefereeUI::ovalDraw(Graph_Data_t &graph, const char name[3], const uint32_t operate, const uint32_t layer,
                         const uint32_t color, const uint32_t width,
                         const uint32_t start_x, const uint32_t start_y, const uint32_t end_x, const uint32_t end_y) {
    setGraphName(graph, name);
    graph.operate_tpye = operate;
    graph.graphic_tpye = UI_Graph_Ellipse;
    graph.layer = layer;
    graph.color = color;
    graph.start_angle = 0;
    graph.end_angle = 0;
    graph.width = width;
    graph.start_x = start_x;
    graph.start_y = start_y;
    graph.radius = 0;
    graph.end_x = end_x;
    graph.end_y = end_y;
}

/**
 * @brief 构造圆弧图形
 */
void RefereeUI::arcDraw(Graph_Data_t &graph, const char name[3], const uint32_t operate, const uint32_t layer,
                        const uint32_t color, const uint32_t start_angle, const uint32_t end_angle,
                        const uint32_t width,
                        const uint32_t start_x, const uint32_t start_y, const uint32_t end_x, const uint32_t end_y) {
    setGraphName(graph, name);
    graph.operate_tpye = operate;
    graph.graphic_tpye = UI_Graph_Arc;
    graph.layer = layer;
    graph.color = color;
    graph.start_angle = start_angle;
    graph.end_angle = end_angle;
    graph.width = width;
    graph.start_x = start_x;
    graph.start_y = start_y;
    graph.radius = 0;
    graph.end_x = end_x;
    graph.end_y = end_y;
}

/**
 * @brief 构造浮点数图形(value 为显示值乘以 1000 后的整数)
 */
void RefereeUI::floatDraw(Graph_Data_t &graph, const char name[3], const uint32_t operate, const uint32_t layer,
                          const uint32_t color, const uint32_t size, const uint32_t digit, const uint32_t width,
                          const uint32_t start_x, const uint32_t start_y, const int32_t value) {
    setGraphName(graph, name);
    graph.operate_tpye = operate;
    graph.graphic_tpye = UI_Graph_Float;
    graph.layer = layer;
    graph.color = color;
    graph.start_angle = size;
    graph.end_angle = digit;
    graph.width = width;
    graph.start_x = start_x;
    graph.start_y = start_y;
    graph.radius = value & 0x3FF;
    graph.end_x = value >> 10 & 0x7FF;
    graph.end_y = value >> 21 & 0x7FF;
}

/**
 * @brief 构造整数图形
 */
void RefereeUI::intDraw(Graph_Data_t &graph, const char name[3], const uint32_t operate, const uint32_t layer,
                        const uint32_t color, const uint32_t size, const uint32_t width,
                        const uint32_t start_x, const uint32_t start_y, const int32_t value) {
    setGraphName(graph, name);
    graph.operate_tpye = operate;
    graph.graphic_tpye = UI_Graph_Int;
    graph.layer = layer;
    graph.color = color;
    graph.start_angle = size;
    graph.end_angle = 0;
    graph.width = width;
    graph.start_x = start_x;
    graph.start_y = start_y;
    graph.radius = value & 0x3FF;
    graph.end_x = value >> 10 & 0x7FF;
    graph.end_y = value >> 21 & 0x7FF;
}

/**
 * @brief 构造字符图形(格式化用法同 printf)
 */
void RefereeUI::charDraw(String_Data_t &graph, const char name[3], const uint32_t operate, const uint32_t layer,
                         const uint32_t color, const uint32_t size, const uint32_t width,
                         const uint32_t start_x, const uint32_t start_y, const char *fmt, ...) {
    setGraphName(graph.Graph_Control, name);
    graph.Graph_Control.operate_tpye = operate;
    graph.Graph_Control.graphic_tpye = UI_Graph_Char;
    graph.Graph_Control.layer = layer;
    graph.Graph_Control.color = color;
    graph.Graph_Control.start_angle = size;
    graph.Graph_Control.width = width;
    graph.Graph_Control.start_x = start_x;
    graph.Graph_Control.start_y = start_y;
    graph.Graph_Control.radius = 0;
    graph.Graph_Control.end_x = 0;
    graph.Graph_Control.end_y = 0;

    /* 使用 vsnprintf 防止格式化结果溢出 show_Data(30 字节) */
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(reinterpret_cast<char *>(graph.show_Data), sizeof(graph.show_Data), fmt, ap);
    va_end(ap);

    if (n < 0) {
        n = 0;
    } else if (n > static_cast<int>(sizeof(graph.show_Data))) {
        n = sizeof(graph.show_Data);
    }
    graph.Graph_Control.end_angle = static_cast<uint32_t>(n); /* 字符串长度 */
}

/**
 * @brief 推送图形数组到客户端(协议仅支持 1 / 2 / 5 / 7 个图形)
 */
void RefereeUI::graphRefresh(const Graph_Data_t *graphs, const uint8_t cnt) {
    if (referee_ == nullptr || graphs == nullptr) {
        return;
    }

    /* 图形个数与交互数据命令码映射, 非法个数直接返回 */
    uint16_t data_cmd_id;
    switch (cnt) {
        case 1:
            data_cmd_id = UI_Data_ID_Draw1;
            break;
        case 2:
            data_cmd_id = UI_Data_ID_Draw2;
            break;
        case 5:
            data_cmd_id = UI_Data_ID_Draw5;
            break;
        case 7:
            data_cmd_id = UI_Data_ID_Draw7;
            break;
        default:
            return;
    }

    const Referee::id_t &id = referee_->getData().id;
    const auto data_length = static_cast<uint16_t>(Interactive_Data_LEN_Head + UI_Operate_LEN_PerDraw * cnt);
    const auto frame_length = static_cast<uint16_t>(LEN_HEADER + LEN_CMDID + data_length + LEN_TAIL);

    /* 组帧缓冲: 最大 7 图形帧长 120 字节 */
    uint8_t buffer[LEN_HEADER + LEN_CMDID + Interactive_Data_LEN_Head + UI_Operate_LEN_PerDraw * 7 + LEN_TAIL]{};

    /* 先在对齐的打包结构体中填好帧头 + 命令码 + 交互数据帧头, 再整体 memcpy 到缓冲区 */
    UI_GraphHeader_t head{};
    head.frame_header.SOF = REFEREE_SOF;
    head.frame_header.DataLength = data_length;
    head.frame_header.Seq = seq_;
    head.frame_header.CRC8 = Referee::getCrc8(reinterpret_cast<uint8_t *>(&head), LEN_CRC8, Referee::CRC8_INIT);
    head.cmd_id = ID_student_interactive;
    head.datahead.data_cmd_id = data_cmd_id;
    head.datahead.sender_ID = id.robot_id;
    head.datahead.receiver_ID = id.client_id;
    memcpy(buffer, &head, sizeof(head));

    /* 逐个拷贝图形数据 */
    for (uint8_t i = 0; i < cnt; i++) {
        memcpy(buffer + sizeof(head) + UI_Operate_LEN_PerDraw * i, &graphs[i], UI_Operate_LEN_PerDraw);
    }

    /* 帧尾 CRC16 并发送 */
    Referee::appendCrc16(buffer, frame_length);
    referee_->send(buffer, frame_length);
    seq_++;
}

/**
 * @brief 推送字符图形到客户端
 */
void RefereeUI::charRefresh(const String_Data_t &string_data) {
    if (referee_ == nullptr) {
        return;
    }

    const Referee::id_t &id = referee_->getData().id;
    constexpr uint8_t data_length = Interactive_Data_LEN_Head + UI_Operate_LEN_DrawChar;

    UI_CharRefresh_t frame{};
    frame.frame_header.SOF = REFEREE_SOF;
    frame.frame_header.DataLength = data_length;
    frame.frame_header.Seq = seq_;
    frame.frame_header.CRC8 = Referee::getCrc8(reinterpret_cast<uint8_t *>(&frame), LEN_CRC8, Referee::CRC8_INIT);

    frame.cmd_id = ID_student_interactive;
    frame.datahead.data_cmd_id = UI_Data_ID_DrawChar;
    frame.datahead.sender_ID = id.robot_id;
    frame.datahead.receiver_ID = id.client_id;
    frame.string_data = string_data;

    frame.frametail = Referee::getCrc16(reinterpret_cast<uint8_t *>(&frame),
                                        LEN_HEADER + LEN_CMDID + data_length, Referee::CRC16_INIT);

    referee_->send(reinterpret_cast<uint8_t *>(&frame), LEN_HEADER + LEN_CMDID + data_length + LEN_TAIL);
    seq_++;
}

/**
 * @brief 删除客户端图层
 */
void RefereeUI::uiDelete(const uint8_t del_operate, const uint8_t layer) {
    if (referee_ == nullptr) {
        return;
    }

    const Referee::id_t &id = referee_->getData().id;
    constexpr uint8_t data_length = Interactive_Data_LEN_Head + UI_Operate_LEN_Del;

    UI_Delete_t frame{};
    frame.frame_header.SOF = REFEREE_SOF;
    frame.frame_header.DataLength = data_length;
    frame.frame_header.Seq = seq_;
    frame.frame_header.CRC8 = Referee::getCrc8(reinterpret_cast<uint8_t *>(&frame), LEN_CRC8, Referee::CRC8_INIT);

    frame.cmd_id = ID_student_interactive;
    frame.datahead.data_cmd_id = UI_Data_ID_Del;
    frame.datahead.sender_ID = id.robot_id;
    frame.datahead.receiver_ID = id.client_id;
    frame.delete_operate = del_operate;
    frame.layer = layer;

    frame.frametail = Referee::getCrc16(reinterpret_cast<uint8_t *>(&frame),
                                        LEN_HEADER + LEN_CMDID + data_length, Referee::CRC16_INIT);

    referee_->send(reinterpret_cast<uint8_t *>(&frame), LEN_HEADER + LEN_CMDID + data_length + LEN_TAIL);
    seq_++;
}
