#ifndef RDM_H
#define RDM_H

#include <despot/interface/pomdp.h>

namespace despot {

/* =============================================================================
 * RdmState class
 * =============================================================================*/

class RdmState: public State {
public:
	int mec_satisfaction;
	int mr_satisfaction;

	RdmState();

	RdmState(int satisfaction_mec, int satisfaction_mr);

	std::string text() const;
};

/* =============================================================================
 * Rdm class
 * =============================================================================*/

class Rdm: public DSPOMDP {
private:
	mutable MemoryPool<RdmState> memory_pool_;

public:
	// 22/06/2017  constants for states
	static const int TRUE, FALSE;
	// 22/06/2017  constants for actions
	static const int MST, RT;

	// 22/06/2017 constants for REC observations
	//static const int REC_LOWER_A, REC_IN_A_B, REC_IN_B_C, REC_GREATER_C;

	// 28/06/2017 constants for NCC observations
	//static const int NCC_LOWER_R, NCC_IN_R_S, NCC_GREATER_S;

	// 29/06/2017 constants for all possible observations
	static const int REC_LOWER_X_AND_NCC_LOWER_R, REC_LOWER_X_AND_NCC_IN_R_S, REC_LOWER_X_AND_NCC_GREATER_S;
	static const int REC_IN_X_Y_AND_NCC_LOWER_R, REC_IN_X_Y_AND_NCC_IN_R_S, REC_IN_X_Y_AND_NCC_GREATER_S;
	static const int REC_GREATER_Y_AND_NCC_LOWER_R, REC_GREATER_Y_AND_NCC_IN_R_S, REC_GREATER_Y_AND_NCC_GREATER_S;


	static const double NOISE;


	Rdm();
	Rdm(std::string params_file);

	bool Step(State& s, double random_num, int action, double& reward,
		OBS_TYPE& obs) const;
	int NumStates() const;
	int NumActions() const;
	double ObsProb(OBS_TYPE obs, const State& s, int a) const;

	State* CreateStartState(std::string type) const;
	Belief* InitialBelief(const State* start, std::string type = "DEFAULT") const;



	inline double GetMaxReward() const ;

	inline ValuedAction GetBestAction() const ;


/*	inline double GetMaxReward() const {
		return 0.1606;
	}

	inline ValuedAction GetMinRewardAction() const {
		//return ValuedAction(MST, 0.1191);
		return ValuedAction(MST, 0.0945);
	}
*/

	//ScenarioLowerBound* CreateScenarioLowerBound(std::string name = "DEFAULT",
		//std::string particle_bound_name = "DEFAULT") const;

	void PrintState(const State& state, std::ostream& out = std::cout) const;

	//15/05/2017 Luis
	void GetBelief(const Belief& belief) const;

	void PrintBelief(const Belief& belief, std::ostream& out = std::cout) const;
	void PrintObs(const State& state, OBS_TYPE obs, std::ostream& out = std::cout) const;
	void PrintAction(int action, std::ostream& out = std::cout) const;

	State* Allocate(int state_id, double weight) const;
	State* Copy(const State* particle) const;
	void Free(State* particle) const;
	int NumActiveParticles() const;
	double f_reward(int action, State& state) const;
	int f_posi(State& state) const;



	//void arrow() const; // 08/08/2017

};

} // namespace despot

#endif
