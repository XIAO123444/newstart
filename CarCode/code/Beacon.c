#include "Beacon.h"
#include "math.h"
int16 available_beacon_num=0; //记录信标灯数量
int16 on_beacon_num_count=0; //记录点亮的信标灯数量
float R=0;      //翼展半径
uint8 Beacon_num[4]={0,0,0,0};   //记录四个象限内信标灯数量
Struct_Beacon_t_typedef Beacon_raw_info[MAX_BEACON_NUM]; //信标灯信息原始结构体数组
Struct_Beacon_t_typedef Beacon_last_info; //信标灯信息临时结构体
Struct_Beacon_t_typedef Beacon_show_info[MAX_BEACON_NUM]; //用于显示的信标灯信息结构体数组
#define ips200_x_max (240)
#define ips200_y_max (320)
float Matrix_Beacon[MAX_BEACON_NUM][MAX_BEACON_NUM]={0};        //信标灯距离矩阵




void Beacon_init(void)
{
    for(int16 i =0;i<MAX_BEACON_NUM;i++)
    {
        if(i==0)
        {
            Beacon_raw_info[i].color=RGB565_RED;
        }
        else
        {
            Beacon_raw_info[i].color=RGB565_WHITE;
        }
        Beacon_raw_info[i].beacon_id=i;
        Beacon_raw_info[i].next_beacon_id=-1;
        Beacon_raw_info[i].lighton=0;
        Beacon_raw_info[i].x=ips200_x_max/2;
        Beacon_raw_info[i].y=ips200_y_max/2;
    }
}
/*——-------------------------------------------------------------------------------------------------
函数简介     计算信标灯距离矩阵
参数说明     无
返回参数     无
使用示例        caculate_beacon_distance_matrix();
备注信息
---------------------------------------------------------------------------------------------------*/

void caculate_beacon_distance_matrix()
{
    for(int16 i=0;i<available_beacon_num;i++)
    {
        for(int16 j=0;j<available_beacon_num;j++)
        {
            if(i==j)
            {
                Matrix_Beacon[i][j]=0;
            }
            else
            {
                int16 delta_x=Beacon_show_info[i].x-Beacon_show_info[j].x;
                int16 delta_y=Beacon_show_info[i].y-Beacon_show_info[j].y;
                Matrix_Beacon[i][j]=sqrt((float)(delta_x*delta_x+delta_y*delta_y));
            }
        }
    }
}
int8 show_circle[][2] = {{-2,0},{-2,1},{-2,-1},{-1,2},{0,2},{1,2},{2,1},{2,0},{2,-1},{1,-2},{0,-2},{-1,-2}};

/*——-------------------------------------------------------------------------------------------------
函数简介     显示单个信标灯点
参数说明     x               信标灯X坐标
参数说明     y               信标灯Y坐标
参数说明     color           信标灯颜色
返回参数     无
使用示例        show_beacon_point(120,160,RGB565_RED);
备注信息
---------------------------------------------------------------------------------------------------*/

void show_beacon_point(float x,float y,rgb565_color_enum color)
{

    for(int i=0;i<12;i++)
    {
        if(x>2&&x<ips200_x_max-2&&y>2&&y<ips200_y_max-2)
        {
            ips200_draw_point(((uint16)x+show_circle[i][0]), ((uint16)y+show_circle[i][1]), color);
        }
    }
}
/*——-------------------------------------------------------------------------------------------------
函数简介     移除单个信标灯点
参数说明     x               信标灯X坐标
参数说明     y               信标灯Y坐标
参数说明     color           信标灯颜色
返回参数     无
使用示例        show_beacon_point(120,160,RGB565_RED);
备注信息
---------------------------------------------------------------------------------------------------*/
void remove_beacon_point(int16 x,int16 y)
{
    show_beacon_point(x,y,RGB565_BLACK);
}



// 计算当前路径长度（使用已计算的距离矩阵 Matrix_Beacon）
static float path_length(const int16 *order, int16 len, uint8 close_loop)
{
    float sum = 0.0f;
    for(int16 k = 0; k < len - 1; k++)
    {
        int16 a = order[k];
        int16 b = order[k + 1];
        sum += Matrix_Beacon[a][b];
    }
    if(close_loop && len > 1)
    {
        sum += Matrix_Beacon[order[len - 1]][order[0]];
    }
    return sum;
}

// 最近邻生成初始路径（起点固定为 0，仅访问 lighton==1 的节点）
static int16 build_nn_route(int16 *order, int16 max_out)
{
    // 收集必须访问的节点（lighton==1），起点固定 0
    // 注：若 0 不为必经点也要作为起点加入路径
    bool must[MAX_BEACON_NUM] = {0};
    int16 must_cnt = 0;

    for(int16 i = 0; i < available_beacon_num; i++)
    {
        if(Beacon_raw_info[i].lighton == 1)
        {
            must[i] = true;
            must_cnt++;
        }
    }
    // 路径容量保护
    if(must_cnt + 1 > max_out) must_cnt = max_out - 1;

    // 起点
    int16 len = 0;
    order[len++] = 0;

    // 剩余可选集合（仅 lighton==1 且非起点）
    bool visited[MAX_BEACON_NUM] = {0};
    visited[0] = true;

    int16 remaining = 0;
    for(int16 i = 0; i < available_beacon_num; i++)
    {
        if(must[i] && i != 0) remaining++;
    }

    int16 cur = 0;
    while(remaining > 0)
    {
        float best = 1e30f;
        int16 best_j = -1;
        for(int16 j = 0; j < available_beacon_num; j++)
        {
            if(j == cur) continue;
            if(!must[j] || visited[j]) continue;
            float d = Matrix_Beacon[cur][j];
            if(d < best)
            {
                best = d;
                best_j = j;
            }
        }
        if(best_j < 0)
        {
            // 没找到下一个（异常），提前退出
            break;
        }
        order[len++] = best_j;
        visited[best_j] = true;
        cur = best_j;
        remaining--;
    }
    return len;
}

// 2-opt 改善（不闭环；若需要闭环，传 close_loop=1）
static void two_opt_improve(int16 *order, int16 len, uint8 close_loop, int max_passes)
{
    if(len <= 3) return;

    for(int pass = 0; pass < max_passes; pass++)
    {
        bool improved = false;

        // 典型 2-opt：尝试反转 [i..j] 段，比较边 (i-1,i) + (j,j+1)
        for(int16 i = 1; i < len - 1; i++)
        {
            for(int16 j = i + 1; j < len - (close_loop ? 0 : 1); j++)
            {
                int16 a = order[i - 1];
                int16 b = order[i];
                int16 c = order[j];
                int16 d = (j + 1 < len) ? order[j + 1] : (close_loop ? order[0] : -1);

                float old_cost = Matrix_Beacon[a][b] + ((d >= 0) ? Matrix_Beacon[c][d] : 0.0f);
                float new_cost = Matrix_Beacon[a][c] + ((d >= 0) ? Matrix_Beacon[b][d] : 0.0f);

                if(new_cost + 1e-6f < old_cost)
                {
                    // 反转 [i..j]
                    for(int16 l = i, r = j; l < r; l++, r--)
                    {
                        int16 tmp = order[l];
                        order[l] = order[r];
                        order[r] = tmp;
                    }
                    improved = true;
                }
            }
        }

        if(!improved) break;
    }
}

// 规划信标路径：从 0 出发，访问所有 lighton==1 的点（不回到起点），输出顺序到 out_order
// close_loop==1 时，2-opt 会按闭环优化（但 out_order 序列不重复放回起点）
// 返回路径长度（节点数）
int16 plan_beacon_route_nn_2opt(int16 *out_order, int16 max_out, uint8 close_loop)
{
    if(available_beacon_num <= 0 || max_out <= 0)
        return 0;

    // 先确保距离矩阵已计算
    // 若你的矩阵基于 Beacon_show_info，请保证 show 集与 raw 的索引一致
    // caculate_beacon_distance_matrix();

    int16 len = build_nn_route(out_order, max_out);

    // 用 2-opt 改善若干轮（可调）
    two_opt_improve(out_order, len, close_loop, 10);

    return len;
}

// 将路径写回 next_beacon_id（最后一个若不闭环则置 -1；闭环则指向起点）
void apply_route_set_next_ids(const int16 *order, int16 len, uint8 close_loop)
{
    if(len <= 0) return;
    for(int16 k = 0; k < len - 1; k++)
    {
        int16 u = order[k];
        int16 v = order[k + 1];
        Beacon_raw_info[u].next_beacon_id = v;
    }
    if(close_loop)
    {
        Beacon_raw_info[order[len - 1]].next_beacon_id = order[0];
    }
    else
    {
        Beacon_raw_info[order[len - 1]].next_beacon_id = -1;
    }
}

// 供上层一次性调用的“规划 + 回写”封装
void calculate_beacon_trace(void)
{
    int16 route[MAX_BEACON_NUM] = {0};
    // 不闭环（回到起点传 1）
    int16 len = plan_beacon_route_nn_2opt(route, MAX_BEACON_NUM, 0);
    apply_route_set_next_ids(route, len, 0);
}

/*——-------------------------------------------------------------------------------------------------
函数简介     显示所有信标灯信息
参数说明     无
返回参数     无
使用示例        show_beacon_info();
备注信息
---------------------------------------------------------------------------------------------------*/
void show_beacon_info()
{
    for(int i=0;i<Beacon_num[0];i++)
    {   
        show_beacon_point(Beacon_raw_info[i].x,Beacon_raw_info[i].y,Beacon_raw_info[i].color);
    }
}

// Cohen–Sutherland 编码工具（文件作用域，避免在函数内定义函数）
enum { BEACON_CS_LEFT = 1, BEACON_CS_RIGHT = 2, BEACON_CS_BOTTOM = 4, BEACON_CS_TOP = 8 };

static inline uint8 beacon_outcode(float x, float y)
{
    uint8 c = 0;
    if (x < 0.0f)                           c |= BEACON_CS_LEFT;
    else if (x > (float)(ips200_x_max - 1)) c |= BEACON_CS_RIGHT;
    if (y < 0.0f)                           c |= BEACON_CS_BOTTOM;
    else if (y > (float)(ips200_y_max - 1)) c |= BEACON_CS_TOP;
    return c;
}

void ips200_draw_line_clipped(float x0, float y0, float x1, float y1, uint16 color)
{
    // 删除函数内的 enum 和 outcode 定义，改用上面的工具函数
    uint8 c0 = beacon_outcode(x0, y0);
    uint8 c1 = beacon_outcode(x1, y1);

    // 反复裁剪直到完全接受或拒绝
    while (1)
    {
        if ((c0 | c1) == 0) {
            break;                  // 完全在内
        } else if (c0 & c1) {
            return;                 // 完全在外
        } else {
            float x, y;
            uint8 c_out = c0 ? c0 : c1;

            float dx = x1 - x0;
            float dy = y1 - y0;

            if (c_out & BEACON_CS_TOP) {
                float yb = (float)(ips200_y_max - 1);
                if (dy == 0.0f) return;                   // 保护除零
                x = x0 + dx * (yb - y0) / dy;
                y = yb;
            } else if (c_out & BEACON_CS_BOTTOM) {
                float yb = 0.0f;
                if (dy == 0.0f) return;
                x = x0 + dx * (yb - y0) / dy;
                y = yb;
            } else if (c_out & BEACON_CS_RIGHT) {
                float xb = (float)(ips200_x_max - 1);
                if (dx == 0.0f) return;
                y = y0 + dy * (xb - x0) / dx;
                x = xb;
            } else { // BEACON_CS_LEFT
                float xb = 0.0f;
                if (dx == 0.0f) return;
                y = y0 + dy * (xb - x0) / dx;
                x = xb;
            }

            if (c_out == c0) { x0 = x; y0 = y; c0 = beacon_outcode(x0, y0); }
            else             { x1 = x; y1 = y; c1 = beacon_outcode(x1, y1); }
        }
    }

    // DDA 连续打点（端点已在可视区内）
    float dx = x1 - x0;
    float dy = y1 - y0;
    float adx = dx > 0 ? dx : -dx;
    float ady = dy > 0 ? dy : -dy;
    int steps = (int)((adx > ady ? adx : ady)) + 1;
    if (steps < 1) steps = 1;

    float sx = dx / (float)steps;
    float sy = dy / (float)steps;

    float x = x0, y = y0;
    for (int i = 0; i <= steps; i++) {
        int xi = (int)(x + 0.5f);
        int yi = (int)(y + 0.5f);
        if ((unsigned)xi < (unsigned)ips200_x_max && (unsigned)yi < (unsigned)ips200_y_max) {
            ips200_draw_point((uint16)xi, (uint16)yi, color);
        }
        x += sx; y += sy;
    }
}

static inline float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// 在 Width×Length 的显示区域内，按各轴独立缩放将 raw_info 映射到 Beacon_show_info
// 返回缩放系数 K1(=sx)、K2(=sy)。优先以 lighton==1 的点作为缩放依据；若没有，则使用全部点。
void beacon_build_show_scaled(const Struct_Beacon_t_typedef raw_info[],
                              int16 Width, int16 Length,
                              float* K1, float* K2)
{
    if (!raw_info || Width <= 0 || Length <= 0) {
        if (K1) *K1 = 1.0f;
        if (K2) *K2 = 1.0f;
        return;
    }

    // 参与缩放的点的包围盒
    float min_x = 1e30f, max_x = -1e30f;
    float min_y = 1e30f, max_y = -1e30f;
    int   sel_cnt = 0;

    // 先统计 lighton==1 的点
    for (int16 i = 0; i < MAX_BEACON_NUM; i++) {
        if (raw_info[i].lighton == 1) {
            if (raw_info[i].x < min_x) min_x = raw_info[i].x;
            if (raw_info[i].x > max_x) max_x = raw_info[i].x;
            if (raw_info[i].y < min_y) min_y = raw_info[i].y;
            if (raw_info[i].y > max_y) max_y = raw_info[i].y;
            sel_cnt++;
        }
    }
    // 若没有被点亮的点，则用全部点
    if (sel_cnt == 0) {
        min_x = 1e30f; max_x = -1e30f;
        min_y = 1e30f; max_y = -1e30f;
        for (int16 i = 0; i < MAX_BEACON_NUM; i++) {
            if (raw_info[i].x < min_x) min_x = raw_info[i].x;
            if (raw_info[i].x > max_x) max_x = raw_info[i].x;
            if (raw_info[i].y < min_y) min_y = raw_info[i].y;
            if (raw_info[i].y > max_y) max_y = raw_info[i].y;
        }
    }

    // 防止退化（所有点同 x 或同 y）
    float dx = max_x - min_x;
    float dy = max_y - min_y;
    if (dx <= 0.0f) dx = 1.0f;
    if (dy <= 0.0f) dy = 1.0f;

    // 预留边距，避免贴边（可按需调整）
    const float margin = 2.0f;
    float avail_w = (float)Width  - 2.0f * margin;
    float avail_h = (float)Length - 2.0f * margin;
    if (avail_w <= 1.0f) avail_w = (float)Width;
    if (avail_h <= 1.0f) avail_h = (float)Length;

    float sx = avail_w / dx;
    float sy = avail_h / dy;

    // 返回缩放系数
    if (K1) *K1 = sx;
    if (K2) *K2 = sy;

    // 平移使 min 对齐到 margin 处
    float off_x = margin - min_x * sx;
    float off_y = margin - min_y * sy;

    // 写入显示数组（仅缩放+平移，保持相对位置）
    for (int16 i = 0; i < MAX_BEACON_NUM; i++) {
        Beacon_show_info[i] = raw_info[i];
        float x = raw_info[i].x * sx + off_x;
        float y = raw_info[i].y * sy + off_y;
        // 裁剪到可视范围
        x = clampf(x, 0.0f, (float)(Width  - 1));
        y = clampf(y, 0.0f, (float)(Length - 1));
        Beacon_show_info[i].x = x;
        Beacon_show_info[i].y = y;
    }
}

