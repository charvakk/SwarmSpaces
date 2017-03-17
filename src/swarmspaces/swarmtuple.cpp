/*
 * AUTHOR: Charvak Kondapalli <ckondapalli@wpi.edu>
 *
 */

#include <argos3/core/utility/math/vector2.h>

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace std;
using namespace argos;

/*
 * Tuple definition
 */
class CSwarmTuple{

public:

	CSwarmTuple(){
		m_iID = 0;
		m_fRange = 0;
		m_vfPosition = CVector2(1000.0, 1000.0);
	}

	CSwarmTuple(int id, CVector2 position, Real range, string info){
		m_iID = id;
		m_vfPosition = position;
		m_fRange = range;
		m_sInfo = info;
	}

//	~CSwarmTuple(){}

	Real getFRange() const {
		return m_fRange;
	}

	void setFRange(Real fRange) {
		m_fRange = fRange;
	}

	const string getSInfo() const {
		return m_sInfo;
	}

	void setSInfo(const string sInfo) {
		m_sInfo = sInfo;
	}

	CVector2 getVfPosition() const {
		return m_vfPosition;
	}

	void setVfPosition(CVector2 const vfPosition) {
		m_vfPosition = vfPosition;
	}

	int getId() const {
		return m_iID;
	}

	void setId(const int id){
		m_iID = id;
	}

private:
//	static int instanceCounter;
	CVector2 m_vfPosition;
	int m_iID;
	Real m_fRange;
	string m_sInfo;

};
