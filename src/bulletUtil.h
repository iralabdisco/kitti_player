#ifndef BULLET_DEBUG_UTIL
#define BULLET_DEBUG_UTIL
#include <sstream>
#include <iostream>
#include "tf/LinearMath/Vector3.h"
#include "tf/LinearMath/Transform.h"
#include "tf/LinearMath/Quaternion.h"
#include <cmath>

void printBullet(tf::Vector3 vector, const char* msg);
void printBullet(tf::Transform transform, const char* msg);
std::string bulletString(tf::Vector3 vector);
std::string bulletString(tf::Transform transform);

inline double normSquared(const tf::Vector3 &a)
{
	double sum = 0.0;
	for (int i = 0; i < 3; ++i)
		sum += a[i] * a[i];
	return sum;
}

/** Returns the euclidean norm of \a a.
 * \param a
 * \return sqrt(a[0]^2 + a[1]^2 + ... + a[dim]^2)
 */
inline double norm(const tf::Vector3 &a)
{
	return std::sqrt(normSquared(a));
}


#endif
