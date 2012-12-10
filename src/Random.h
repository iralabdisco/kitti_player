#ifndef RANDOM_H
#define RANDOM_H

#include <boost/thread/mutex.hpp>
// #include <boost/random.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/linear_congruential.hpp>
#include <ctime>
#include <fstream>
// #include <RoSe/Random.h>

// #include "GlobalConfig.h"

// namespace drafting3d {

/** \addtogroup Drafting3d
 * @{
 */

// typedef boost::minstd_rand base_generator_type;
typedef boost::mt19937 base_generator_type;
typedef boost::uniform_real<> distribution_type;
typedef boost::variate_generator<base_generator_type, boost::uniform_real<> > dblGenType;
typedef boost::variate_generator<base_generator_type, boost::normal_distribution<> > nrmGenType;



/** 
 * \brief Class that provides random number generation utilities
 *
*/
class Random
{
public:
	Random();
	Random( const Random& other );

	static uint64_t GetSeed();

	/** \brief Return a double uniformly distributed between 0 and 1 */
	double getDouble();
	/** \brief Return a float uniformly distributed between 0 and 1 */
	float getFloat();
	/** \brief Return a double sampled from a standard normal distribution */
	double getGaussianDouble();
	/** \brief Return a double sampled from a normal distribution with mu=0 and standard deviation as of the parameter
		\param standardDeviation the standard deviation of the normal distribution from which the number is sampled
	*/
	double sampleNormalDistribution(double standardDeviation);
	/** 
	 * \brief Sample a number from a triangular distribution
	 *
	 * This method is kept for back compatibility although in the current software version is never used
	*/
	double sampleTriangularDistribution(double standardDeviation);

	private:
		dblGenType doubleGenerator;
		nrmGenType gaussianGenerator;

		// this is needed for the triangular distribution
		static double SQUARE_ROOT_6_HALF;
};

/** @} */
// } // end of namespace
#endif
