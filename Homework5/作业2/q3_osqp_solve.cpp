#include <osqp.h>   // OSQP求解器的C接口
#include <iostream>

using std::cout;
using std::endl;

int main() {
    c_int n = 2, m = 1; // n变量个数（x和y），m约束个数（x+y<=4)

    // 构建P矩阵（二次项系数）
    c_float P_x[2] = {1.0, 10.0};
    c_int   P_i[2] = {0, 1};
    c_int   P_p[3] = {0, 1, 2};
    csc* P = csc_matrix(n, n, n, P_x, P_i, P_p);
    // 参数顺序：行数、列数、最大非零元素个数、数值数组、行索引数组、列指针数组

    // 构造q向量（一次项系数）
    c_float q[2] = {-3.0, -30.0};

    // 构造A矩阵（约束矩阵）
    c_float A_x[2] = {1.0, 1.0};
    c_int   A_i[2] = {0, 0};
    c_int   A_p[3] = {0, 1, 2};
    csc* A = csc_matrix(m, n, n, A_x, A_i, A_p);

    c_float l[1] = {-OSQP_INFTY}, u[1] = {4.0};
    // 这里l表示无下界，u表示上界为4

    // 打包数据
    OSQPData data = {n, m, P, A, q, l, u};

    // 设置求解器
    OSQPSettings* settings = (OSQPSettings*)c_malloc(sizeof(OSQPSettings)); //分配设置结构体内存
    osqp_set_default_settings(settings); //填充默认值
    settings->eps_abs = settings->eps_rel = 1e-8; //精度设为1e-8，关闭verbose输出
    settings->verbose = false;

    // 初始化
    OSQPWorkspace* work = nullptr;
    if (osqp_setup(&work, &data, settings)) {
        // osqp_setup把数据拷贝进求解器内部
        cout << "初始化失败" << endl;
        return 1;
    }

    // 求解
    osqp_solve(work);

    // 读取结果
    if (work->info->status_val == OSQP_SOLVED) {
        // work->info->status_val 求解状态码，OSQP_SOLVED = 1 表示成功
        double x = work->solution->x[0]; // x的最优值
        double y = work->solution->x[1]; // y的最优值
        double mu = work->solution->y[0]; //第一个约束的拉格朗日乘子
        cout << "x* = " << x << "\ny* = " << y << "\nmu = " << mu << endl;
        cout << "x+y = " << x+y << " (<=4)" << endl;
    }

    // 清理
    osqp_cleanup(work);
    c_free(settings);
    return 0;
}
