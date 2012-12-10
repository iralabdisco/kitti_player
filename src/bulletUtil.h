#ifndef BULLET_DEBUG_UTIL
#define BULLET_DEBUG_UTIL
#include <sstream>
#include <iostream>
#include "LinearMath/btVector3.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btQuaternion.h"
#include <cmath>



void printBullet(btVector3 vector, const char* msg);
void printBullet(btTransform transform, const char* msg);
std::string bulletString(btVector3 vector);
std::string bulletString(btTransform transform);




inline double normSquared(const btVector3 &a)
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
inline double norm(const btVector3 &a)
{
	return std::sqrt(normSquared(a));
}


#endif