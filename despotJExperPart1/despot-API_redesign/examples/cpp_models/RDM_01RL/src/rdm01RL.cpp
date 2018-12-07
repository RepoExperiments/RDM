#include "rdm01RL.h"

#include <string>

#include <despot/core/builtin_lower_bounds.h>
#include <despot/core/builtin_policy.h>
#include <despot/core/builtin_upper_bounds.h>
#include <despot/core/particle_belief.h>
using namespace std;

namespace despot {

// 28/06/2017 In Despot the actions should be consecutive numbers but the observations could have a free numeric identification
const int Rdm::MST = 0;
const int Rdm::RT = 1;

const int Rdm::FALSE=0;
const int Rdm::TRUE=1;

// 29/06/2017 values for observations
const int Rdm::REC_LOWER_X_AND_NCC_LOWER_R=0;
const int Rdm::REC_LOWER_X_AND_NCC_IN_R_S=1;
const int Rdm::REC_LOWER_X_AND_NCC_GREATER_S=2;
const int Rdm::REC_IN_X_Y_AND_NCC_LOWER_R=3;
const int Rdm::REC_IN_X_Y_AND_NCC_IN_R_S=4;
const int Rdm::REC_IN_X_Y_AND_NCC_GREATER_S=5;
const int Rdm::REC_GREATER_Y_AND_NCC_LOWER_R=6;
const int Rdm::REC_GREATER_Y_AND_NCC_IN_R_S=7;
const int Rdm::REC_GREATER_Y_AND_NCC_GREATER_S=8;


const double Rdm::NOISE = 0.15;



/* =============================================================================
 * RdmState class
 * =============================================================================*/

RdmState::RdmState() :
	mec_satisfaction(0),mr_satisfaction(0) {
}

RdmState::RdmState(int satisfaction_mec, int satisfaction_mr) :
	mec_satisfaction(satisfaction_mec),mr_satisfaction(satisfaction_mr) {
}

string RdmState::text() const {
	string sstate="";
	if(mec_satisfaction == Rdm::TRUE){
		sstate="MEC=TRUE";
	}else{
		sstate="MEC=FALSE";
	}

	if(mr_satisfaction == Rdm::TRUE){
		sstate=sstate+" MR=TRUE";
	}else{
		sstate=sstate+" MR=FALSE";
	}

	return sstate;
}

/* =============================================================================
 * OptimalRdmPolicy class
 * =============================================================================*/

/*class OptimalTigerPolicy: public Policy {
public:
	OptimalTigerPolicy(const DSPOMDP* model,
		ParticleLowerBound* bound) :
		Policy(model, bound) {
	}

	// NOTE: optimal for noise = 0.15
	int Action(const vector<State*>& particles, RandomStreams& streams,
		History& history) const {

		//
		 //if (history.Size() == 0 || history.LastAction() != LISTEN) {
		 //actions->push_back(LISTEN);
		 //return actions;
		// }

		 //actions->push_back(history.LastObservation());
		 //


		int count_diff = 0;
		for (int i = history.Size() - 1;
			i >= 0 && history.Action(i) == Tiger::LISTEN; i--)
			count_diff += history.Observation(i) == Tiger::LEFT ? 1 : -1;

		if (count_diff >= 2)
			return Tiger::RIGHT;
		else if (count_diff <= -2)
			return Tiger::LEFT;
		else
			return Tiger::LISTEN;
	}
};
*/


/* =============================================================================
 * Rdm class
 * =============================================================================*/

Rdm::Rdm() {
}


// 08/08/2017

/*
  void Rdm::arrow() const{

	// To include logic for updating rewards and produce possible new decisions
	//maxReward=0.1606;
	//minReward=0.0945;
	//minAction=MST;

}*/


inline double Rdm::GetMaxReward() const {
	return Globals::config.maxReward;
}

inline ValuedAction Rdm::GetBestAction() const {
	return ValuedAction(Globals::config.minAction, Globals::config.minReward);
}

int Rdm::f_posi(State& s) const {

	int posi;
	RdmState& state = static_cast<RdmState&>(s);

	if(state.mec_satisfaction==1 && state.mr_satisfaction==1){
		posi=0;
	}else if(state.mec_satisfaction==1 && state.mr_satisfaction==0){
		posi=1;
	}else if(state.mec_satisfaction==0 && state.mr_satisfaction==1){
		posi=2;
	}else if(state.mec_satisfaction==0 && state.mr_satisfaction==0){
		posi=3;
	}

	//cout << "posi " << posi << endl;
	//cout << "satis MEC==T " << Globals::config.current_satisfaction_criteria[0] << endl;
	//cout << "satis MR==T " << Globals::config.current_satisfaction_criteria[1] << endl;


	return posi;

}


double Rdm::f_reward(int action,State& s) const {

	int posi=f_posi(s);

	double p_reward =Globals::config.rewards[action][posi];
	// printing posi and reward
	logi << "posi " << posi
		<< " reward: " << p_reward << endl;


	return p_reward ;

}



/*
 *
 * Example of rewards, Conf D6b2.json
 * Action MEC 	MR	reward
 * MST		T	T	90
 * MST		T	F	45
 * MST		F	T	25
 * MST		F	F	5
 *
 * RT		T	T	100
 * RT		T	F	10
 * RT		F	T	20
 * RT		F	F	0
 *
 *this is store in json as:
 *[
 *[90,45,25,5 ],
 *[100,10,20,0]
 *]
 *It means, MST TT, is posi[0,0], MST TF is posi[0,1] and so on ...
 */


/*
 * Main logic:
 * Based on the current state and action we should obtain a new state and a new observation
 * The arrays are sorted, e.g., in the transition model the position [0,0,0], represent action = MST(0), previous state MEC T and MR T, acu prob. (range) of MEC=T and MR=T
 */

bool Rdm::Step(State& s, double random_num, int action, double& reward,
	OBS_TYPE& obs) const {

	RdmState& state = static_cast<RdmState&>(s);
	bool terminal = false;

	Globals::config.jlog_["current_mec_state(s)"]=state.mec_satisfaction;
	Globals::config.jlog_["current_mr_state(s)"]=state.mr_satisfaction;

	// to look up within the 3 dimension transition model
	int currentStatePosi=f_posi(s); // this is the state s, we will determine the state s' lines below


	//double rewards[8];

	//  assigning rewards based on the current state and taken action, not in the future one
	//reward = f_reward(action, state);


	double random_num_obs=Random::RANDOM.NextDouble(0,1);

	for(int i=0;i<Globals::config.action_states_num;i++){


		/* 20/06/2018
		 * For each action, we have possible combinations of NFRs, e.g., 4,
		 * and for each possible combination of NFRs, we have a set of values, e.g., for the transition model 4 values, and for the observation model 9
		 *
		 * Because we have 2 NFRs, 4 possible combinations exists,
		 * the first one is managed with the if sentence and the other ones
		 * within the for at the else.
		 *
		 * For each one of the 4 possible
		 * combinations we have 12 possible observations.
		 *
		 * at this stage: the transition model has 3 dimensions (actions, combinations of NFRs, values for each combination (4))
		 * and the observation model has 3 dimensions (actions, combinations of NFRs, values for each combination (9) )
		 *
		 */
		if(action==i){ // i is representing the possible action 0 = MST and 1 = RT

			// we are determining P(s'|s,a)
			//
			// we have this if else because the comparison in if is different of the comparisons in else
			if (random_num<=Globals::config.transition_model[i][currentStatePosi][0]){// Action = i,posi of s, range for s' = (MEC=T & MR = T)

				// first element of nfr_states is T T
				/*
				 * nfr_states:
				 * T 	T
				 * T	F
				 * F	T
				 * F	F
				 */
				state.mec_satisfaction=Globals::config.nfr_states[0][0];
				state.mr_satisfaction=Globals::config.nfr_states[0][1];

				// logging for the new state
				//Globals::config.jlog_["mec_state"]=state.mec_satisfaction;
				//Globals::config.jlog_["mr_state"]=state.mr_satisfaction;
				// assigned rewards based on the new arrived state
				reward = f_reward(action, state);
				//Globals::config.jlog_["step_reward"]=reward;


				// to notice that here we are assigning the reward base on the new state ;) not the previous state,
				// this does not agree with original DESPOT logic
				//reward=Globals::config.rewards[i][0];

				// here we include logic for the observations
				// the first possible entry for observations is rec < x  ncc <r
				if(random_num_obs<=Globals::config.observation_model[i][0][0]){

					obs=Globals::config.obs_states[0];

				}else{

					// if we are here,  j would  be 0, so that is the reason we put 0 directly
					for (int k=1;k<Globals::config.obs_states_num;k++){
						if(random_num_obs>Globals::config.observation_model[i][0][k-1] &&
							random_num_obs<=Globals::config.observation_model[i][0][k]){
								obs=Globals::config.obs_states[k];
						}

					}

				}


			}else{
				// the other combinations of nfr_states and action=i
				// j goes from 1 to 3
				for (int j=1;j<Globals::config.nfr_states_num;j++){

					if(random_num>Globals::config.transition_model[i][currentStatePosi][j-1] &&
							random_num<=Globals::config.transition_model[i][currentStatePosi][j]){// MEC=T & MR = F  0.3825

						state.mec_satisfaction=Globals::config.nfr_states[j][0];
						state.mr_satisfaction=Globals::config.nfr_states[j][1];

						// logging for the new state
						//Globals::config.jlog_["mec_state"]=state.mec_satisfaction;
						//Globals::config.jlog_["mr_state"]=state.mr_satisfaction;
						// assigned rewards based on the new arrived state
						reward = f_reward(action, state);
						//Globals::config.jlog_["step_reward"]=reward;


						//reward=Globals::config.rewards[i][j];

						// the first possible entry for observations is ctvar < r  psvar <r  acvar < r
						if(random_num_obs<=Globals::config.observation_model[i][0][0]){

							obs=Globals::config.obs_states[0];

						}else{
							for (int k=1;k<Globals::config.obs_states_num;k++){
								if(random_num_obs>Globals::config.observation_model[i][j][k-1] &&
									random_num_obs<=Globals::config.observation_model[i][j][k]){
										obs=Globals::config.obs_states[k];
								}

							}

						}


					}

				}
			}

		}

	} // for

	//Globals::config.jlog_["arrived_current_mec_state"]=state.mec_satisfaction;
	//Globals::config.jlog_["arrived_current_mr_state"]=state.mr_satisfaction;


	return terminal;
}

int Rdm::NumStates() const {
	return Globals::config.nfr_states_num;
}

int Rdm::NumActions() const {
	return Globals::config.action_states_num;
}

// 27/10/2017: In theory, this function computes the probability of observing obs given the current state state,
// resulting from executing an action action in previous state

// 27/07/2018: we already have one observation as parameter in this method, we are only determining the prob. of that
// observation

double Rdm::ObsProb(OBS_TYPE obs, const State& s, int a) const {
	const RdmState& state = static_cast<const RdmState&>(s);
	double proba=0;


	for(int i=0;i<Globals::config.action_states_num;i++){

		// if action is equal to action identified by index "i"
		if(a==i){

			for(int j=0;j<Globals::config.nfr_states_num;j++){


				// if the current state is equal to one of the possible states in nfr_states
				if(state.mec_satisfaction==Globals::config.nfr_states[j][0] &&
						state.mr_satisfaction==Globals::config.nfr_states[j][1] ){


					if(obs==REC_LOWER_X_AND_NCC_LOWER_R){// the first one, it has value 0

						proba=Globals::config.observation_model[i][j][0];

					}else{// could be any of the others biggers than 0 (starting by k = 1 :)

						for(int k=1;k<Globals::config.obs_states_num;k++){

							if(obs==k){// the first one, it has value 0
								proba=Globals::config.observation_model[i][j][k]-Globals::config.observation_model[i][j][k-1];
							}


						}

					}


				}


			}
		}
	}

	// check planner this was originally created to show obsProb of the previous time slice as the one
	// of the current time slice
	Globals::config.obsProb=proba;

	Globals::config.jlog_["observation_probability"]=Globals::config.obsProb;


	return proba;


}

// 22/06/2017: NextInt(2)return 0 or 1 as a uniform distribution, i.e.,
// 50% of probabilities for being true or false the initial state
State* Rdm::CreateStartState(string type) const {
	return new RdmState(Random::RANDOM.NextInt(2),Random::RANDOM.NextInt(2));
}

Belief* Rdm::InitialBelief(const State* start, string type) const {
	vector<State*> particles;

	// 2/11/2017 original values were 0.25 0.25 0.25 0.25
	// now, we are giving more probabilities to T T

	RdmState* mec_t_mr_t = static_cast<RdmState*>(Allocate(-1, 0.25));
	mec_t_mr_t->mec_satisfaction = TRUE;
	mec_t_mr_t->mr_satisfaction = TRUE;
	particles.push_back(mec_t_mr_t);

	RdmState* mec_t_mr_f = static_cast<RdmState*>(Allocate(-1, 0.25));
	mec_t_mr_f->mec_satisfaction = TRUE;
	mec_t_mr_f->mr_satisfaction = FALSE;
	particles.push_back(mec_t_mr_f);

	RdmState* mec_f_mr_t = static_cast<RdmState*>(Allocate(-1, 0.25));
	mec_f_mr_t->mec_satisfaction = FALSE;
	mec_f_mr_t->mr_satisfaction = TRUE;
	particles.push_back(mec_f_mr_t);

	RdmState* mec_f_mr_f = static_cast<RdmState*>(Allocate(-1, 0.25));
	mec_f_mr_f->mec_satisfaction = FALSE;
	mec_t_mr_t->mr_satisfaction = FALSE;
	particles.push_back(mec_f_mr_f);

	return new ParticleBelief(particles, this);
}


void Rdm::PrintState(const State& state, ostream& out) const {
	const RdmState& rdmstate = static_cast<const RdmState&>(state);
	Globals::config.jlog_["arrived_mec_state_(s')"]=rdmstate.mec_satisfaction;
	Globals::config.jlog_["arrived_mr_state_(s')"]=rdmstate.mr_satisfaction;

	out << rdmstate.text() << endl;
}

void Rdm::PrintBelief(const Belief& belief, ostream& out) const {

}

//15/05/2017
void Rdm::GetBelief(const Belief& belief) const{

	int numberOfStateVariables=1;


	double statevariable1 [2];
	double statevariable2 [2];

	for (int i=0;i<2;i++){
		statevariable1[i]=0;
		statevariable2[i]=0;
	}

	// 29/06/2017
	const vector<State*>& particles =
			static_cast<const ParticleBelief&>(belief).particles();


	for (int i = 0; i < particles.size(); i++) {

		State* particle = particles[i];
		const RdmState* state = static_cast<const RdmState*>(particle);

		statevariable1[state->mec_satisfaction]+=particle->weight;
		statevariable2[state->mr_satisfaction]+=particle->weight;

	}

	// 07/07/2017 values true and false for NFRs, position 1 is true, position 0 is false
	Globals::config.current_satisfaction_criteria[0]=statevariable1[1];
	Globals::config.current_satisfaction_criteria[1]=statevariable2[1];



}


void Rdm::PrintObs(const State& state, OBS_TYPE obs, ostream& out) const {

	//json jlog_;
	/*
	 *  showing in console and logging to a file the obtained observation
	 *
	 */
	for(int i=0;i<Globals::config.obs_states_num;i++){
		if(obs==Globals::config.obs_states[i]){
			out << Globals::config.obs_descriptions[i] << endl;

			// check the planner, this was used to show the observations on the previous step as the ones
			// on the current step ;)
			Globals::config.obsDes=Globals::config.obs_descriptions[i];


			Globals::config.jlog_["observation_code"]=obs;
			Globals::config.jlog_["observation_description"]=Globals::config.obsDes;
			//Globals::config.jlog_["observation_after_action"]=Globals::config.obs_descriptions[i]+std::to_string(obs);
		}
	}

	Globals::config.obsCode=obs;

	//Globals::config.situ_ << jlog_;
	//Globals::config.situ_ << endl;

}

void Rdm::PrintAction(int action, ostream& out) const {
	if (action == Rdm::MST) {
		out << "MST" << endl;
	} else {
		out << "RT" << endl;
	}
}

State* Rdm::Allocate(int state_id, double weight) const {
	RdmState* particle = memory_pool_.Allocate();
	particle->state_id = state_id;
	particle->weight = weight;
	return particle;
}

State* Rdm::Copy(const State* particle) const {
	RdmState* new_particle = memory_pool_.Allocate();
	*new_particle = *static_cast<const RdmState*>(particle);
	new_particle->SetAllocated();
	return new_particle;
}

void Rdm::Free(State* particle) const {
	memory_pool_.Free(static_cast<RdmState*>(particle));
}

int Rdm::NumActiveParticles() const {
	return memory_pool_.num_allocated();
}

} // namespace despot
