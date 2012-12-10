#include "Random.h"

// #include "GlobalConfig.h"

// namespace drafting3d {
/** \addtogroup Drafting3d
 * @{
 */

double Random::SQUARE_ROOT_6_HALF = std::sqrt(6.0)/2.0;

uint64_t Random::GetSeed()
{
        uint64_t seed;
        std::ifstream urandom;
        urandom.open("/dev/urandom");
        urandom.read(reinterpret_cast<char*> (&seed), sizeof (seed));
        urandom.close();
	return seed;
}

Random::Random():
	doubleGenerator(base_generator_type(Random::GetSeed()),boost::uniform_real<>(0,1)),
	gaussianGenerator(base_generator_type(Random::GetSeed()),boost::normal_distribution<>(0,1))
{
}

double Random::getDouble(){
	return doubleGenerator();
}

float Random::getFloat(){
	return (float)doubleGenerator();
}

double Random::getGaussianDouble(){
	return gaussianGenerator();
}


double Random::sampleNormalDistribution(double standardDeviation)
{
	// boost::mutex::scoped_lock lock(RandGlobal::getRandomMutex());
	// RandGlobal::getRandomInstance().getGaussianDouble() liefert eine standardnormalverteilte Zufallsvariable
	// X ~ N(u,s^2) => aX+b ~ N(au+b, (as)^2)
	// X ~ N(0,1)  =>  aX+b ~ N(b,a^2)
	// somit:
	return this->getGaussianDouble() * standardDeviation;

	// alternative Berechnungsmethode:
	// approximative normal distributed, but all values are in [-6*b, 6*b]
	// ( with b = standardDeviation)
	/*
	double sum = 0.0;
	for (int i = 0; i < 12; ++i)
		sum += RandGlobal::getRandomInstance().getDouble() * 2 * standardDeviation - standardDeviation;
	return 0.5 * sum;
	*/
}
double Random::sampleTriangularDistribution(double standardDeviation)
{
	// boost::mutex::scoped_lock lock(RandGlobal::getRandomMutex());
	return SQUARE_ROOT_6_HALF
		* ( (this->getDouble() * 2 * standardDeviation - standardDeviation)
			+ (this->getDouble() * 2 * standardDeviation - standardDeviation));
}

/** @} */
// } // end of namespace