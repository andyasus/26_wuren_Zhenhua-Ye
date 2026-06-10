#include <iostream>
#include <Eigen/Core>
#include <cmath>

// 定义梯度函数
Eigen::Vector2d gradient(const Eigen::Vector2d& X){
    double dx = X(0) - 3;        // ∂f/∂x = (x-3)
    double dy = 10 * (X(1) - 3); // ∂f/∂y = 10*(y-3)
    return Eigen::Vector2d(dx, dy);
}

int main() {
    // 初始化参数
    Eigen::Vector2d X(0, 0);        // 机器狗初始位置
    Eigen::Vector2d target(3, 3);   // 信号源位置
    double eta = 0.18;               // 学习率（尝试后发现0.18最佳，eta太大发散，eta 太小收敛太慢）
    double tol = 1e-3;              // 终止误差
    int iteration = 0;              // 迭代计数器

    // 梯度下降循环
    while ((X - target).norm() > tol) {
        Eigen::Vector2d grad = gradient(X); // 计算梯度
        X = X - eta * grad;                 // 更新位置
        iteration++;                        // 迭代计数

        // 打印中间结果观察收敛过程
        std::cout << "iter " << iteration << ": X = " << X.transpose()
                  << ", error = " << (X - target).norm() << std::endl;
    }

    // 输出结果
    std::cout << "最优解: (" << X(0) << ", " << X(1) << ")" << std::endl;
    std::cout << "迭代次数: " << iteration << std::endl;
    std::cout << "最终误差: " << (X - target).norm() << std::endl;

    return 0;
}
