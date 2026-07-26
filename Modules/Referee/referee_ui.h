#pragma once

#include <cstdint>
#include "referee.h"
#include "referee_protocol.h"

/**
 * @brief RoboMaster 裁判系统操作手界面 UI 绘制工具
 *
 * 提供图形(直线/矩形/圆/椭圆/圆弧/浮点数/整数/字符)的构造与推送接口:
 * - xxxDraw 系列仅填充 Graph_Data_t / String_Data_t 图形描述结构体, 不发送;
 * - graphRefresh / charRefresh / uiDelete 组装 0x0301 交互帧(帧头 + CRC)后经 Referee::send 发送。
 * 发送目标客户端 ID 取自 Referee 解析到的机器人状态(0x0201), 须在裁判系统在线后调用。
 *
 * @note 依赖旧框架模式枚举的 UI 业务刷新逻辑(模式变化检测等)未随本模块移植,
 *       待新项目模式体系建立后基于本工具类重写。
 */
class RefereeUI {
public:
    /**
     * @brief RefereeUI 构造函数
     * @param referee 关联的裁判系统实例, 用于获取机器人 ID 并发送 UI 帧
     */
    explicit RefereeUI(Referee *referee);

    /* ---- 图形构造: 仅填充图形描述结构体, 需调用 graphRefresh / charRefresh 推送生效 ---- */

    /**
     * @brief 构造直线图形
     * @param graph 图形数据结构体
     * @param name 图形名(3 字符, 用于后续 Change/Del 标识)
     * @param operate 图形操作(UI_Graph_ADD / Change / Del)
     * @param layer 图层(0-9)
     * @param color 颜色(UI_Graph_Color_e)
     * @param width 线宽
     * @param start_x,start_y 起点坐标
     * @param end_x,end_y 终点坐标
     */
    static void lineDraw(Graph_Data_t &graph, const char name[3], uint32_t operate, uint32_t layer,
                         uint32_t color, uint32_t width,
                         uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y);

    /**
     * @brief 构造矩形图形, 参数同 lineDraw, 终点为对角顶点
     */
    static void rectangleDraw(Graph_Data_t &graph, const char name[3], uint32_t operate, uint32_t layer,
                              uint32_t color, uint32_t width,
                              uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y);

    /**
     * @brief 构造整圆图形
     * @param start_x,start_y 圆心坐标
     * @param radius 半径
     */
    static void circleDraw(Graph_Data_t &graph, const char name[3], uint32_t operate, uint32_t layer,
                           uint32_t color, uint32_t width,
                           uint32_t start_x, uint32_t start_y, uint32_t radius);

    /**
     * @brief 构造椭圆图形
     * @param start_x,start_y 圆心坐标
     * @param end_x,end_y xy 半轴长度
     */
    static void ovalDraw(Graph_Data_t &graph, const char name[3], uint32_t operate, uint32_t layer,
                         uint32_t color, uint32_t width,
                         uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y);

    /**
     * @brief 构造圆弧图形
     * @param start_angle,end_angle 起止角度(0-360, 顺时针, 12 点方向为 0)
     * @param start_x,start_y 圆心坐标
     * @param end_x,end_y xy 半轴长度
     */
    static void arcDraw(Graph_Data_t &graph, const char name[3], uint32_t operate, uint32_t layer,
                        uint32_t color, uint32_t start_angle, uint32_t end_angle, uint32_t width,
                        uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y);

    /**
     * @brief 构造浮点数图形
     * @param size 字号
     * @param digit 小数位数
     * @param value 显示值乘以 1000 后的 32 位整数
     */
    static void floatDraw(Graph_Data_t &graph, const char name[3], uint32_t operate, uint32_t layer,
                          uint32_t color, uint32_t size, uint32_t digit, uint32_t width,
                          uint32_t start_x, uint32_t start_y, int32_t value);

    /**
     * @brief 构造整数图形
     * @param size 字号
     * @param value 显示的 32 位整数值
     */
    static void intDraw(Graph_Data_t &graph, const char name[3], uint32_t operate, uint32_t layer,
                        uint32_t color, uint32_t size, uint32_t width,
                        uint32_t start_x, uint32_t start_y, int32_t value);

    /**
     * @brief 构造字符图形, 格式化用法同 printf
     * @param graph 字符数据结构体
     * @param size 字号
     * @param fmt 格式化字符串(结果不超过 30 字符)
     */
    static void charDraw(String_Data_t &graph, const char name[3], uint32_t operate, uint32_t layer,
                         uint32_t color, uint32_t size, uint32_t width,
                         uint32_t start_x, uint32_t start_y, const char *fmt, ...);

    /* ---- UI 推送: 组装交互帧并发送, 使图形生效 ---- */

    /**
     * @brief 推送图形数组到客户端
     * @param graphs 图形数组首地址
     * @param cnt 图形个数, 协议仅支持 1 / 2 / 5 / 7
     */
    void graphRefresh(const Graph_Data_t *graphs, uint8_t cnt);

    /**
     * @brief 推送字符图形到客户端
     * @param string_data 字符数据结构体
     */
    void charRefresh(const String_Data_t &string_data);

    /**
     * @brief 删除客户端图层
     * @param del_operate 删除操作(UI_Delete_Operate_e)
     * @param layer 要删除的图层(0-9, 删除全部时无效)
     */
    void uiDelete(uint8_t del_operate, uint8_t layer);

private:
    /**
     * @brief 填充图形名: 按内存地址增大方向逆序拷贝, 与客户端解析顺序保持一致
     */
    static void setGraphName(Graph_Data_t &graph, const char name[3]);

    Referee *referee_ = nullptr;
    uint8_t seq_ = 0; /* UI 交互帧包序号, 每发送一帧自增 */
};
