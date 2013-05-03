/**
 * @file   TentacleRateLimiter.h
 *
 * @date   Apr 26, 2013
 * @author Adam Panzica
 * @brief  Class definition for TentacleRateLimiter
 */

#ifndef TENTACLERATELIMITER_H_
#define TENTACLERATELIMITER_H_

//*********** SYSTEM DEPENDANCIES ****************//
#include <utility>
//************ LOCAL DEPENDANCIES ****************//

//***********    NAMESPACES     ****************//

namespace aero_path_planning
{

/**
 * Class for providing a convienent method of rate-change limiting Tentacle selection. It does this by limiting the number-of-tentacle difference between
 * the last selected tentacle and the newly selected one.
 */
class TentacleRateLimiter
{
public:
	/**
	 * @author Adam Panzica
	 * @param [in] number_of_tentacles The number of tentacles per speed set
	 * @param [in] rate_limit          The maximum difference between a new tentacle and the last tentacle
	 *
	 * Rate limiting is by default enabled
	 */
	TentacleRateLimiter(int number_of_tentacles, int rate_limit);

	/**
	 * @author Adam Panzica
	 * @brief Given a raw tentacle selection, produces a rate-limited tentacle selection
	 * @param [in] raw_index The raw tentacle selection
	 * @return The index of the rate-limited tentacle selection
	 */
	int nextTentacle(int raw_index);

	/**
	 * @author Adam Panzica
	 * @brief Sets the rate limit
	 * @param [in] rate_limit The new rate-change limite
	 */
	void setRateLimit(int rate_limit);

	/**
	 * @author Adam Panzica
	 * @brief Gets the current rate-change limit
	 * @return The current rate-change limit
	 */
	int getRateLimit() const;

	/**
	 * @author Adam Panzica
	 * @brief disables/enables rate-change limiting
	 * @param [in] disable true to disable limiting, false to enable it
	 */
	void disableLimit(bool disable);

private:

	int num_tent_;
	int last_tentacle_;
	int origin_;
	int rate_limit_;
	bool enabled_;

	bool sameSide(int raw_index) const;
};

};

#endif /* TENTACLERATELIMITER_H_ */
