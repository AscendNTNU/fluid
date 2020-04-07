/**
 * @file util.h
 */

#ifndef UTIL_H
#define UTIL_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>

#include <vector>

/**
 * @brief Holds a bunch of convenience functions.
 */
class Util {
   public:
    /**
     * @param current First point.
     * @param target Second point.
     *
     * @return Eucledian distance betweeen @p current and @p target.
     */
    static double distanceBetween(const geometry_msgs::Point& current, const geometry_msgs::Point& target) {
        double delta_x = target.x - current.x;
        double delta_y = target.y - current.y;
        double delta_z = target.z - current.z;

        return sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z);
    }

    /**
     * @brief Creates a path between @p first and @p last which consist of a series of points between them specified
     *        by @p density.
     *
     * @param first The first point.
     * @param last The last point.
     * @param density The density of points between @p first and @p last.
     *
     * @return The new path with inserted points.
     */
    static std::vector<geometry_msgs::Point> createPath(const geometry_msgs::Point& first,
                                                        const geometry_msgs::Point& last, const double& density) {
        double distance = distanceBetween(first, last);

        std::vector<geometry_msgs::Point> path;

        float delta_x = last.x - first.x;
        float delta_y = last.y - first.y;
        float delta_z = last.z - first.z;

        for (int i = 0; i < int(density * distance) + 1; i++) {
            geometry_msgs::Point temp;

            temp.x = first.x + i * delta_x / (density * distance);
            temp.y = first.y + i * delta_y / (density * distance);
            temp.z = first.z + i * delta_z / (density * distance);
            path.insert(path.end(), temp);
        }

        return path;
    }
};

#endif
