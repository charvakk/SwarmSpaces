/*
 * AUTHOR: Charvak Kondapalli <ckondapalli@wpi.edu>
 *
 */

#include "swarmtuple.h"

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace std;
using namespace argos;

CSwarmTuple::CSwarmTuple(){
	m_iID = 0;
	m_fRange = 0;
	m_vfPosition = CVector2(1000.0, 1000.0);
}

CSwarmTuple::CSwarmTuple(int id, CVector2 position, Real range, string info){
	m_iID = id;
	m_vfPosition = position;
	m_fRange = range;
	m_sInfo = info;
}

//	~CSwarmTuple(){}

Real CSwarmTuple::getFRange() const {
	return m_fRange;
}

void CSwarmTuple::setFRange(Real fRange) {
	m_fRange = fRange;
}

const string CSwarmTuple::getSInfo() const {
	return m_sInfo;
}

void CSwarmTuple::setSInfo(const string &sInfo) {
	m_sInfo = sInfo;
}

CVector2 CSwarmTuple::getVfPosition() const {
	return m_vfPosition;
}

void CSwarmTuple::setVfPosition(CVector2 const &vfPosition) {
	m_vfPosition = vfPosition;
}

int CSwarmTuple::getId() const {
	return m_iID;
}

void CSwarmTuple::setId(const int id){
	m_iID = id;
}
