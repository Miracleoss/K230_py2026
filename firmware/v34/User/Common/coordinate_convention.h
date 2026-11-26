#ifndef USER_COMMON_COORDINATE_CONVENTION_H
#define USER_COMMON_COORDINATE_CONVENTION_H

/**
 * @brief 整机坐标系约定。
 *
 * 机体坐标系（从整机尾部看向整机头部）：
 *   +X：前向，+Y：上向，+Z：右向（右手系；向前 +X，向上 +Y，向右 +Z，向左 -Z）。
 *
 * BMI088 到机体的实测固定映射为：
 *   body_x = -sensor_y
 *   body_y = sensor_x
 *   body_z = sensor_z
 *
 * 该映射矩阵的行列式为 +1，与上述右手机体坐标约定一致；仍建议在接入
 * 姿态或控制算法前保留实物逐轴验证。
 */
enum class BodyAxis {
    X, ///< 机体 X 轴（前向）。
    Y, ///< 机体 Y 轴（上向）。
    Z  ///< 机体 Z 轴（右向）。
};

#endif /* USER_COMMON_COORDINATE_CONVENTION_H */
