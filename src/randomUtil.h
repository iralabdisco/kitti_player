/* $Id: randomUtil.h,v 1.2 2009/03/04 18:02:40 welle Exp $ */
/********************************************************
 * Author: Jochen Welle
 *
 * Erstellt im Rahmen der Diplomarbeit:
 * "FastSLAM basierte Erstellung eines 3D-Umgebungsmodells
 *  während der Fahrt" von Jochen Welle
 *
 * Copyright (c) 2009 Jochen Welle. All rights reserved.
 ********************************************************/
#ifndef RANDOM_UTIL_H
#define RANDOM_UTIL_H

#include <boost/thread/mutex.hpp>
#include "Random.h"

// #include "GlobalConfig.h"

// namespace drafting3d {

/** \addtogroup Drafting3d
 * @{
 */

/** 
 * \brief Provides global access to a Random instance
 * 
 * It provides also a mutex to prevent concurrency issues
*/
class RandGlobal
{
public:
	static Random& getRandomInstance();
	static void setRandomInstance(const Random& random);

	/* If you use the random instance in multiple threads, you must protect access to it.*/
	static boost::mutex& getRandomMutex();

	/** \brief Returns a random number in [low, high].
	 *
	 * \a low must be <= \a high
	 * T may be float or double
	 */
	template<typename T>
	static T getRandom(T low, T high);
private:
	static Random randomInstance;
	static boost::mutex randomMutex;

};

inline Random& RandGlobal::getRandomInstance()
{
	return randomInstance;
}

inline void RandGlobal::setRandomInstance(const Random& random)
{
	randomInstance = random;
}

inline boost::mutex& RandGlobal::getRandomMutex()
{
	return randomMutex;
}

template<typename T>
T RandGlobal::getRandom(T low, T high)
{
	return low + getRandomInstance().getFloat() * (high - low);
}

template<>
inline double RandGlobal::getRandom<double>(double low, double high)
{
	return low + getRandomInstance().getDouble() * (high - low);
}

/** @} */
// } // end of namespace
#endif
