/*
 * AUTHOR: Charvak Kondapalli <ckondapalli@wpi.edu>
 *
 */
#ifndef SWARMTUPLE_H
#define SWARMTUPLE_H

#include <argos3/core/utility/math/vector2.h>

using namespace argos;

/*
 * Tuple definition
 */
class CSwarmTuple{

public:

	CSwarmTuple();

	CSwarmTuple(int id, CVector2 position, Real range, std::string info);

//	~CSwarmTuple(){}

	Real getFRange() const;

	void setFRange(Real fRange);

	const std::string getSInfo() const;

	void setSInfo(const std::string &sInfo);

	CVector2 getVfPosition() const;

	void setVfPosition(CVector2 const &vfPosition);

	int getId() const;

	void setId(const int id);

private:
//	static int instanceCounter;
	CVector2 m_vfPosition;
	int m_iID;
	Real m_fRange;
	std::string m_sInfo;

};

#endif
