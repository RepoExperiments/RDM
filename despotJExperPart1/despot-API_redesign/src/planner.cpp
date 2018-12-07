/*
 * Planner.cpp
 *
 *  Created on: 5 Oct 2017
 *      Author: panpan
 */

#include <despot/core/solver.h>
#include <despot/interface/belief.h>
#include <despot/interface/world.h>
#include <despot/logger.h>
#include <despot/planner.h>

namespace despot {

Planner::Planner(string lower_bounds_str,
		string base_lower_bounds_str, string upper_bounds_str, string base_upper_bounds_str)
		:PlannerBase(lower_bounds_str, base_lower_bounds_str, upper_bounds_str, base_upper_bounds_str){
	step_=0;
	round_=0;
}

Planner::~Planner() {
}



/*
 *  21/07/2018: function to determine the injection or not of deviation
 */
int Planner::deviation_on(double noise, int lower_bound, int upper_bound){
	int devi_on=0;

	// is generating a double between 0 and 1
	double random_num=Random::RANDOM.NextDouble();
	std::cout << random_num << endl;

	// the current value of noise is 0.75, i.e., the probability of
	// deviation on is .25
	if (random_num <= 1 - noise){
			//Based on the lower and upper bound we determine how many time slices will last the deviation
			devi_on = Random::RANDOM.NextInt(lower_bound,upper_bound);

	}

	return devi_on;
}


int Planner::ts_deviation(int lower_bound, int upper_bound){
	double factor;

	factor= Random::RANDOM.NextInt(lower_bound,upper_bound);

	return factor;
}


double Planner::prob_deviation(double lower_bound, double upper_bound){
	double factor;

	factor= Random::RANDOM.NextDouble(lower_bound,upper_bound);

	return factor;
}



/*
 * 02/08/2018:
 * The current version of the method is working on deviations only for the transition model
 */
void Planner::deviateModelv0(){

	// 1. saving the original transition model

	std::cout << "Saving original initial models "  << endl ;

	// initial transition model
	for (int i = 0; i < Globals::config.action_states_num; i++) { // 2
			for (int j = 0; j < Globals::config.parents_tranmodel_num; j++) { // 8

				for (int k = 0; k < Globals::config.nfr_num; k++) {  // 2

					Globals::config.ini_transition_model_saved[i][j][k] = Globals::config.ini_transition_model[i][j][k];
					//std::cout << " " << Globals::config.transition_model_saved[i][j][k] << " ";
				}
			}
			std::cout << endl ;
	}

	// initia observation model
	for (int i = 0; i < Globals::config.action_states_num; i++) {
			for (int j = 0; j < Globals::config.nfr_states_num; j++) {

				for (int k = 0; k < Globals::config.obs_ranges_num; k++) {

					Globals::config.ini_observation_model_saved[i][j][k] = Globals::config.ini_observation_model[i][j][k];
					//std::cout << " " << Globals::config.transition_model_saved[i][j][k] << " ";
				}
			}
			std::cout << endl ;
	}

	std::cout << "Saving original models (join and acu prob)"  << endl ;

	// transition model
	for (int i = 0; i < Globals::config.action_states_num; i++) {
			for (int j = 0; j < Globals::config.nfr_states_num; j++) {

				for (int k = 0; k < Globals::config.nfr_states_num; k++) {

					Globals::config.transition_model_saved[i][j][k] = Globals::config.transition_model[i][j][k];
					//std::cout << " " << Globals::config.transition_model_saved[i][j][k] << " ";
				}
			}
			std::cout << endl ;
	}

	// observation model
	for (int i = 0; i < Globals::config.action_states_num; i++) {
			for (int j = 0; j < Globals::config.nfr_states_num; j++) {

				for (int k = 0; k < Globals::config.obs_states_num; k++) {

					Globals::config.observation_model_saved[i][j][k] = Globals::config.observation_model[i][j][k];
					//std::cout << " " << Globals::config.transition_model_saved[i][j][k] << " ";
				}
			}
			std::cout << endl ;
	}


	// 2. Deviating the original "initial"  models
	double factorDevi=prob_deviation(Globals::config.prob_devi_lower,Globals::config.prob_devi_upper);
	std::cout << "Factor Devi " << factorDevi << endl ;


	// different from even, odd positions in the transition model represent the lower bounds for MEC and MR
	// we are deviating the lower bounds
	std::cout << " Deviated ini_Model " << endl;


	// deviating initial transition model
	for (int i = 0; i < Globals::config.action_states_num; i++) { // 2 actions
			for (int j = 0; j < Globals::config.parents_tranmodel_num; j++) { // 8 elements (between the probs of Mec and then MR)


				for (int k = 0; k < Globals::config.nfr_num; k++) { // 2 values, True and False, posi 0 is true,

					//i==0 && j>=4 are the probs of MR when the action selected was MST
					//i==1 && j<=3 are the probs of MEC when the action selected was RT
					//if((i == 0 && j>=4)  ){
					//if( (i == 1 && j<=3) ){
					if((i == 0 && j>=4) || (i == 1 && j<=3)  ){


						if(k==0){ // this is only to catch the value true

							double newLowerTrue=Globals::config.ini_transition_model_saved[i][j][k]-(Globals::config.ini_transition_model_saved[i][j][k]*factorDevi);
							double newLowerFalse=1-newLowerTrue;
							Globals::config.ini_transition_model[i][j][k] = newLowerTrue;
							Globals::config.ini_transition_model[i][j][k+1] = newLowerFalse;
						}
					//}

					//}else{
						//Globals::config.ini_transition_model[i][j][k] = Globals::config.ini_transition_model_saved[i][j][k];
					}

					std::cout << " " << Globals::config.ini_transition_model[i][j][k] << " ";
				}
			}
			std::cout << endl ;
	}




	// for the observation model the logic could be a bit more complex based on the way the data was placed on the configuration file
	//



	//std::cout << " Deviated ini_observation_Model " << endl;

	/*
	for (int i = 0; i < Globals::config.action_states_num; i++) {
			for (int j = 0; j < Globals::config.nfr_states_num; j++) {

				for (int k = 0; k < Globals::config.obs_ranges_num; k++) {



					  // j=0 or j=1  means we have prob. of the type: sMEC < x = 0.8, sMEC in x,y = 0.15, sMEC >y 0.05

					if(j==0 or j==1){

						if(k==0){

							double new_greatery=Globals::config.ini_observation_model_saved[i][j][k+2]+(Globals::config.ini_observation_model_saved[i][j][k+2]*factorDevi);
							double new_inxy=Globals::config.ini_observation_model_saved[i][j][k+1]+(Globals::config.ini_observation_model_saved[i][j][k+1]*factorDevi);
							double new_lowerx=1-(new_greatery+new_inxy);

							Globals::config.ini_observation_model[i][j][k] = new_lowerx;
							Globals::config.ini_observation_model[i][j][k+1] = new_inxy;
							Globals::config.ini_observation_model[i][j][k+2] = new_greatery;

						}



					// this else means j=2 or j=3, i.e., we have prob. of the type: sMR < r = 0.06, sMR in r,s = 0.16, sMR  > s 0.78

					}else{
						if(k==0){

							double new_lowerr=Globals::config.ini_observation_model_saved[i][j][k]+(Globals::config.ini_observation_model_saved[i][j][k]*factorDevi);
							double new_inrs=Globals::config.ini_observation_model_saved[i][j][k+1]+(Globals::config.ini_observation_model_saved[i][j][k+1]*factorDevi);
							double new_greaters=1-(new_greaters+new_inrs);

							Globals::config.ini_observation_model[i][j][k] = new_lowerr;
							Globals::config.ini_observation_model[i][j][k+1] = new_inrs;
							Globals::config.ini_observation_model[i][j][k+2] = new_greaters;

						}
						//Globals::config.ini_observation_model[i][j][k] = Globals::config.ini_observation_model_saved[i][j][k];

					}

					std::cout << " " << Globals::config.ini_observation_model[i][j][k] << " ";
				}
			}
			std::cout << endl ;
	}

    */



	// 3. based on the new ini_transition_model, we generate a new transition model
	transformer_forward_pass();



}


/*
 * 02/08/2018:
 * The current version of the method is working on deviations only for the transition model
 */
void Planner::restoreModelv0(){

	std::cout << "Restoring transition model " << endl ;


	for (int i = 0; i < Globals::config.action_states_num; i++) {
			for (int j = 0; j < Globals::config.parents_tranmodel_num; j++) {

				for (int k = 0; k < Globals::config.nfr_num; k++) {

					Globals::config.ini_transition_model[i][j][k] = Globals::config.ini_transition_model_saved[i][j][k];
					//std::cout << " " << Globals::config.transition_model_saved[i][j][k] << " ";
				}
			}
			std::cout << endl ;
	}


	for (int i = 0; i < Globals::config.action_states_num; i++) {
			for (int j = 0; j < Globals::config.nfr_states_num; j++) {

				for (int k = 0; k < Globals::config.nfr_states_num; k++) {
					Globals::config.transition_model[i][j][k] = Globals::config.transition_model_saved[i][j][k];
					//std::cout << " " << Globals::config.transition_model[i][j][k] << " ";
				}
			}
			std::cout << endl ;
	}


	 std::cout << "Restoring observation model " << endl ;


	for (int i = 0; i < Globals::config.action_states_num; i++) {
		for (int j = 0; j < Globals::config.nfr_states_num; j++) {

				for (int k = 0; k < Globals::config.obs_ranges_num; k++) {

					Globals::config.ini_observation_model[i][j][k] = Globals::config.ini_observation_model_saved[i][j][k];
						//std::cout << " " << Globals::config.transition_model_saved[i][j][k] << " ";
				}
		}
		std::cout << endl ;
	}


	for (int i = 0; i < Globals::config.action_states_num; i++) {
			for (int j = 0; j < Globals::config.nfr_states_num; j++) {

				for (int k = 0; k < Globals::config.obs_states_num; k++) {
					Globals::config.observation_model[i][j][k] = Globals::config.observation_model_saved[i][j][k];
					//std::cout << " " << Globals::config.transition_model[i][j][k] << " ";
				}
			}
			std::cout << endl ;
	}


}


/*
 * The different cases are:
 *  MOC = T and MR = T
 *  MOC = T and MR = F
 *  MOC = F and MR = T
 *  MOC = F and MR = F
 *
 * and they are related to the indices 0 to 3 respectively
 * From config.h we obtain: jMOC_T=0,jMR_T=2,jMOC_F=1,jMR_F=3
 */
void Planner::get_posiCases(int jcaso, int* posiMOC, int* posiMR){
	switch(jcaso){
		case 0: std::cout << "case MOC T and MR = T" << endl;*posiMOC=Globals::config.jMOC_T;*posiMR=Globals::config.jMR_T;break;
		case 1: std::cout << "case MOC T and MR = F" << endl;*posiMOC=Globals::config.jMOC_T;*posiMR=Globals::config.jMR_F;break;
		case 2: std::cout << "case MOC F and MR = T" << endl;*posiMOC=Globals::config.jMOC_F;*posiMR=Globals::config.jMR_T;break;
		case 3: std::cout << "case MOC F and MR = F" << endl;*posiMOC=Globals::config.jMOC_F;*posiMR=Globals::config.jMR_F;break;

	}
}

// transition_model has 3 dimension:
// dimension 1: action (there are two possible)
// dimension 2: per each action  4 possible combinations of NFRs
// dimension 3: per each possible combination, 4 possible joint-acu probabilities
void Planner::uni_probaTraModel(int iaction, int jnfr, double mocT, double mocF, double mrT, double mrF){

	Globals::config.transition_model[iaction][jnfr][0]=mocT*mrT;
	Globals::config.transition_model[iaction][jnfr][1]=Globals::config.transition_model[iaction][jnfr][0]+mocT*mrF;
	Globals::config.transition_model[iaction][jnfr][2]=Globals::config.transition_model[iaction][jnfr][1]+mocF*mrT;
	Globals::config.transition_model[iaction][jnfr][3]=Globals::config.transition_model[iaction][jnfr][2]+mocF*mrF;

	// showing the four possible combinations
	std::cout << mocT << " " << mrT <<  " " << mocT*mrT << " "<< Globals::config.transition_model[iaction][jnfr][0] << endl;
	std::cout << mocT << " " << mrF <<  " " << mocT*mrF << " "<< Globals::config.transition_model[iaction][jnfr][1] << endl;
	std::cout << mocF << " " << mrT <<  " " << mocF*mrT << " "<< Globals::config.transition_model[iaction][jnfr][2] << endl;
	std::cout << mocF << " " << mrF <<  " " << mocF*mrF << " "<< Globals::config.transition_model[iaction][jnfr][3] << endl;
	std::cout << endl ;

}

void Planner::uni_probaObsModel(int iaction, int jnfr, double smoc_x, double smoc_xy, double smoc_y, double smr_r, double smr_rs, double smr_s){

	Globals::config.observation_model[iaction][jnfr][0]=smoc_x*smr_r;
	Globals::config.observation_model[iaction][jnfr][1]=Globals::config.observation_model[iaction][jnfr][0]+smoc_x*smr_rs;
	Globals::config.observation_model[iaction][jnfr][2]=Globals::config.observation_model[iaction][jnfr][1]+smoc_x*smr_s;

	Globals::config.observation_model[iaction][jnfr][3]=Globals::config.observation_model[iaction][jnfr][2]+smoc_xy*smr_r;
	Globals::config.observation_model[iaction][jnfr][4]=Globals::config.observation_model[iaction][jnfr][3]+smoc_xy*smr_rs;
	Globals::config.observation_model[iaction][jnfr][5]=Globals::config.observation_model[iaction][jnfr][4]+smoc_xy*smr_s;

	Globals::config.observation_model[iaction][jnfr][6]=Globals::config.observation_model[iaction][jnfr][5]+smoc_y*smr_r;
	Globals::config.observation_model[iaction][jnfr][7]=Globals::config.observation_model[iaction][jnfr][6]+smoc_y*smr_rs;
	Globals::config.observation_model[iaction][jnfr][8]=Globals::config.observation_model[iaction][jnfr][7]+smoc_y*smr_s;

		// showing the 9 possible combinations
	std::cout << smoc_x << " " << smr_r <<  " " << smoc_x*smr_r << " "<< Globals::config.observation_model[iaction][jnfr][0] << endl;
	std::cout << smoc_x << " " << smr_rs <<  " " << smoc_x*smr_rs << " "<< Globals::config.observation_model[iaction][jnfr][1] << endl;
	std::cout << smoc_x << " " << smr_s <<  " " << smoc_x*smr_s << " "<< Globals::config.observation_model[iaction][jnfr][2] << endl;

	std::cout << smoc_xy << " " << smr_r <<  " " << smoc_xy*smr_r << " "<< Globals::config.observation_model[iaction][jnfr][3] << endl;
	std::cout << smoc_xy << " " << smr_rs <<  " " << smoc_xy*smr_rs << " "<< Globals::config.observation_model[iaction][jnfr][4] << endl;
	std::cout << smoc_xy << " " << smr_s <<  " " << smoc_xy*smr_s << " "<< Globals::config.observation_model[iaction][jnfr][5] << endl;

	std::cout << smoc_y << " " << smr_r <<  " " << smoc_y*smr_r << " "<< Globals::config.observation_model[iaction][jnfr][6] << endl;
	std::cout << smoc_y << " " << smr_rs <<  " " << smoc_y*smr_rs << " "<< Globals::config.observation_model[iaction][jnfr][7] << endl;
	std::cout << smoc_y << " " << smr_s <<  " " << smoc_y*smr_s << " "<< Globals::config.observation_model[iaction][jnfr][8] << endl;

	std::cout << endl ;


}
/*
 * Action states num is 2, and nfr_states_num is 4
 * The idea is to get from the initial probabilities of input the right values
 * for MOC and MR and for their monitorables, the right values are obtained with  get_posiCases
 * Then, the joint acu probabilities are constructed by using:
 * uni_probaTraModel and uni_probaObsModel
 *
 */
void Planner::transformer_forward_pass(){

	double vMOC_T,vMOC_F,vMR_T,vMR_F;
	double vsMOC_x, vsMOC_xy, vsMOC_y, vsMR_r, vsMR_rs, vsMR_s;

	int caseMOC, caseMR;

	for (int i = 0; i < Globals::config.action_states_num; i++) {

		/* there are 4 possible combinations for our 2 NFRs:
		 * T T
		 * T F
		 * F T
		 * F T
		*/
		for (int j=0;j<Globals::config.nfr_states_num;j++){

			// variables for the transition model
			// at this stage, the data is prepared at the json file to use here j and j+4
			vMOC_T=Globals::config.ini_transition_model[i][j][0];
			vMOC_F=Globals::config.ini_transition_model[i][j][1];
			vMR_T=Globals::config.ini_transition_model[i][j+4][0];
			vMR_F=Globals::config.ini_transition_model[i][j+4][1];

			// we send the action (i) and the possible combination (j) together with the values to build the
			// joint probability
			uni_probaTraModel(i,j, vMOC_T, vMOC_F, vMR_T, vMR_F);
		}

		for (int j=0;j<Globals::config.nfr_states_num;j++){
			get_posiCases(j, &caseMOC, &caseMR);

			// variables for the observation model
			vsMOC_x=Globals::config.ini_observation_model[i][caseMOC][0];
			vsMOC_xy=Globals::config.ini_observation_model[i][caseMOC][1];
			vsMOC_y=Globals::config.ini_observation_model[i][caseMOC][2];
			vsMR_r=Globals::config.ini_observation_model[i][caseMR][0];
			vsMR_rs=Globals::config.ini_observation_model[i][caseMR][1];
			vsMR_s=Globals::config.ini_observation_model[i][caseMR][2];

			uni_probaObsModel(i,j, vsMOC_x, vsMOC_xy, vsMOC_y, vsMR_r, vsMR_rs, vsMR_s);


		}
	}

}

void Planner::initRDMConfiguration() {

	/*
	 *  Initialising variables to get the average satisfaction and to count not poor situations
	 */
	Globals::config.ave_MEC = 0;
	Globals::config.ave_MR = 0;
	Globals::config.nobody_poor = 0;
	Globals::config.c1_c2_poor = 0;

	/*
	 *  The main set of input parameters are obtained from a json input file
	 */
	//std::ifstream i("./data/rdm_dis2018_v0.3_01.json"); // original weights, with preference for RT
	//std::ifstream i("./data/rdm_dis2018_v0.3_02.json");	  // updated weights, with preference for MST
	std::ifstream i("./data/rdm_jexperpart1bv0.1.json");	  // preference for MST, new format

	// creating object json and assigning the file to it ;)
	json j;
	i >> j;

	/*
	 * Reading num of actions, states, rewards, observations
	 */

	/*
	 * 21/07/2018:
	 * Variables for injecting deviations on the observations or transitions models
	 */
	Globals::config.activateDeviation = j["activate_deviation"].get<int>();
	Globals::config.prob_devi_upper=j["prob_devi_upper"].get<double>();
	Globals::config.prob_devi_lower=j["prob_devi_lower"].get<double>();

	Globals::config.ts_devi_upper=j["ts_devi_upper"].get<int>();
	Globals::config.ts_devi_lower=j["ts_devi_lower"].get<int>();
	Globals::config.devi_noise = j["devi_noise"].get<double>();



	Globals::config.nfr_num = j["nfr_num"].get<int>();
	Globals::config.nfr_states_num = j["nfr_states_num"].get<int>();
	Globals::config.obs_states_num = j["obs_states_num"].get<int>();
	Globals::config.action_states_num = j["action_states_num"].get<int>();

	json obs_states = j["obs_states"];
	for (int i = 0; i < Globals::config.obs_states_num; i++) {
		Globals::config.obs_states[i] = obs_states[i].get<int>();
	}

	json obs_descriptions = j["obs_descriptions"];
	for (int i = 0; i < Globals::config.obs_states_num; i++) {
		Globals::config.obs_descriptions[i] = obs_descriptions[i].get<
				std::string>();
	}

	json nfr_states = j["nfr_states"];
	for (int i = 0; i < Globals::config.nfr_states_num; i++) {
		for (int j = 0; j < Globals::config.nfr_num; j++) {
			Globals::config.nfr_states[i][j] = nfr_states[i][j].get<int>();
		}
	}
	std::cout << Globals::config.action_states_num << endl;

	json pom_criteria = j["pom_criteria"];
	for (int i = 0; i < Globals::config.nfr_num; i++) {
		for (int j = 0; j < Globals::config.nfr_num; j++) {
			Globals::config.pom_criteria[i][j] =
					pom_criteria[i][j].get<double>();
			cout << "i,j " << i << " " << j << " "
					<< Globals::config.pom_criteria[i][j] << endl;
		}
	}

	json criteria_poor_high = j["criteria_poor_high"];
	for (int i = 0; i < Globals::config.nfr_num; i++) {
		Globals::config.criteria_poor_high[i] =
				criteria_poor_high[i].get<double>();
	}

	json kappa = j["kappa"];
	Globals::config.kappa = kappa.get<int>();

	/*
	 * Reading Max and Min rewards, and Min action, for the initial preferences defined at design time
	 */
	//cout << "reading max min rewards " << endl;
	Globals::config.maxReward = j["initial_max_reward"].get<double>();
	Globals::config.minReward = j["initial_min_reward"].get<double>();
	Globals::config.minAction = j["initial_min_action"].get<int>();

	/*
	 * Reading num of elements for the scale of comparison and reading each element of the scale
	 */
	Globals::config.num_scale_categories = j["num_scale_categories"].get<int>();
	json factors = j["scale_factors"];
	for (int i = 0; i < Globals::config.num_scale_categories; i++) {
		Globals::config.factors[i] = factors[i].get<double>();
	}

	/*
	 * reading initial weights, they are organised by action and then by possible combinations of the NFRs
	 */
	json initial_weights = j["initial_weights"];
	for (int i = 0; i < Globals::config.action_states_num; i++) {
		for (int j = 0; j < Globals::config.nfr_states_num; j++) {
			//cout << "i,j " << i << " " << j << " " << endl;
			Globals::config.rewards[i][j] = initial_weights[i][j].get<double>();
		}
	}

	/*
	 *Flags for doing juggling and for controlling case when all NFRs are poor
	 */
	cout << "reading juggling poor flags " << endl;
	Globals::config.juggling = j["juggling"].get<int>();
	Globals::config.softer_approach = j["softer_approach"].get<int>();
	Globals::config.not_change_pom_if_not_poor =
			j["not_change_pom_if_not_poor"].get<int>();

	/*
	 *  29/07/2018 reading the initial transition model
	 *  it is assumed data is charged sorted, by action, and for each action the entries for each possible state
	 *  this is an array of [2][4][2]
	 *
	 *  */


	json transition_model = j["ini_transition_model"];
	for (int i = 0; i < Globals::config.action_states_num; i++) {
		for (int j = 0; j < Globals::config.parents_tranmodel_num; j++) {

			std::cout << "initial transition_model_ " << i << "," << j << " ";

			for (int k = 0; k < 2; k++) { // 2 because are the initial possible values for MEC and MR
				Globals::config.ini_transition_model[i][j][k] =
						transition_model[i][j][k].get<double>();

				std::cout << Globals::config.ini_transition_model[i][j][k]<< " ";
			}
			std::cout << std::endl;
		}
	}



	/*
	 *  29/07/2018 reading the initial observation model
	 * It is assumed data is charged sorted, first by action, then by possible states, and finally observations for each possible state
	 * This is an array of [2][4][3]
	 */
	json observation_model = j["ini_observation_model"];
	for (int i = 0; i < Globals::config.action_states_num; i++) {
		for (int j = 0; j < Globals::config.nfr_states_num; j++) {

			std::cout << "initial observation_model_ " << i << "," << j << " ";

			for (int k = 0; k < 3; k++) { // 3 is the number of possible values for each monitorable of MEC or MR
				Globals::config.ini_observation_model[i][j][k] =
						observation_model[i][j][k].get<double>();

						std::cout << Globals::config.ini_observation_model[i][j][k]<< " ";

			}
			std::cout << std::endl;

		}
	}

	// 29/07/2018:
	transformer_forward_pass();

	std::cout << "Initial Max reward " << Globals::config.maxReward
			<< std::endl;
	std::cout << "Initial Min reward " << Globals::config.minReward
			<< std::endl;
	std::cout << "Initial Min action " << Globals::config.minAction
			<< std::endl;

	std::cout << "Juggling: " << Globals::config.juggling << '\n';
	std::cout << "Softer approach for factor: "
			<< Globals::config.softer_approach << '\n';

	/*
	 *Getting rewards for taking into account only mec or only mr
	 */
	json rewards_mec = j["rewards_mec"];
	for (int i = 0; i < Globals::config.action_states_num; i++) {
		for (int j = 0; j < Globals::config.nfr_states_num; j++) {
			Globals::config.rewards_mec[i][j] = rewards_mec[i][j].get<double>();
		}
	}

	json rewards_mr = j["rewards_mr"];
	for (int i = 0; i < Globals::config.action_states_num; i++) {
		for (int j = 0; j < Globals::config.nfr_states_num; j++) {
			Globals::config.rewards_mr[i][j] = rewards_mr[i][j].get<double>();
		}
	}

}

bool Planner::RunStep(Solver* solver, World* world, Logger* logger) {

	logger->CheckTargetTime();

	double step_start_t = get_time_second();

	double start_t = get_time_second();
	ACT_TYPE action = solver->Search().action;


	if(action==0)
		Globals::config.jlog_["selected_action"]="MST";
	else
		Globals::config.jlog_["selected_action"]="RT";


	double end_t = get_time_second();
	double search_time = (end_t - start_t);
	logi << "[RunStep] Time spent in " << typeid(*solver).name()
			<< "::Search(): " << search_time << endl;

	OBS_TYPE obs;
	start_t = get_time_second();
	bool terminal = world->ExecuteAction(action, obs);
	end_t = get_time_second();
	double execute_time = (end_t - start_t);
	logi << "[RunStep] Time spent in ExecuteAction(): " << execute_time << endl;

	start_t = get_time_second();
	solver->BeliefUpdate(action, obs);
	end_t = get_time_second();
	double update_time = (end_t - start_t);
	logi << "[RunStep] Time spent in Update(): " << update_time << endl;

	return logger->SummarizeStep(step_++, round_, terminal, action, obs,
			step_start_t);
}

void Planner::PlanningLoop(Solver*& solver, World* world, Logger* logger) {

	Globals::config.situ_.open("./data/logi_.json", std::ofstream::out);


	initRDMConfiguration();

	Globals::config.obsCode=-1;
	Globals::config.obsDes="None";
	//Globals::config.plots_ << "{" << endl;
	Globals::config.situ_ << "{" << endl;

	/*
	 * 21/07/2018 variables for managing a model current deviation
	 */

	int deviationInProgress=0; // flag to identify an active deviation
	int contaCurrentDeviation=0; // counting the number of time slices of the current deviation
	int ts_devi_duration=0; // the number of time slices to be applied to the deviation



	for (int i = 0; i < Globals::config.sim_len; i++) {


		// 21/07/2018 setting coming from the configuration file: to identify if we are going to use deviations or not during this iteration
		std::cout << "Activate Deviation status " << Globals::config.activateDeviation << endl;

		if(Globals::config.activateDeviation){

			if (!deviationInProgress){

				// we get 0 or a the number of time slices to apply a deviation
				ts_devi_duration=deviation_on(Globals::config.devi_noise,Globals::config.ts_devi_lower,Globals::config.ts_devi_upper);
				std::cout << "Devi duration: " << ts_devi_duration << endl;

				if(ts_devi_duration>0){

					std::cout << "Updating the  obs/tran model  " << endl;

					// updating the obs/trans model
					// we are casting the array to a pointer of double
					deviateModelv0();
					deviationInProgress=1;
				}


			}else{ //deviation is in progress, so, we should control duration of current deviation
				std::cout << "Devi in progress " << contaCurrentDeviation << endl;

				contaCurrentDeviation+=1;
					// we should stop the deviation, and restore the original model

				if(contaCurrentDeviation>=ts_devi_duration){
					// restoring initial values control variables
					// restoring initial values obs/tran model
					deviationInProgress=0;
					contaCurrentDeviation=0;
					ts_devi_duration=0;

					restoreModelv0();

				}

			}

		}


		double step_start_t = get_time_second();


		//Globals::config.plots_ << "\"" << i << "\":";
		Globals::config.situ_ << "\"" << i << "\":";

		// 23/07/2018
		Globals::config.current_time_slice_=i;

		// those observations recorded at the log  will be the current one for the next time step
		//Globals::config.jlog_["current_observation_code"]=Globals::config.obsCode;
		//Globals::config.jlog_["observation_description"]=Globals::config.obsDes;
		//Globals::config.jlog_["observation_probability"]=Globals::config.obsProb;

		// here the observations of the previous time slice were recorded as the ones for the current time slice
		// it was the idea to show in the logs the observations, and belief of the current state were the action
		// was taken, however what DESPOT shows on the console is different:
		// given an action, it shows the new state and observations obtained after executing the action, it also
		// shows the obtained reward (current and accumulated reward values)




		bool terminal = RunStep(solver, world, logger);
		if (terminal)
			break;



		Globals::config.jlog_["flagUpdatedRewards"]=Globals::config.flagUpdatedRewards;
		Globals::config.jlog_["current_rewards"]=Globals::config.rewards;

		//Globals::config.plots_ << Globals::config.jsatis_;
		Globals::config.situ_ << Globals::config.jlog_;


		if(i<(Globals::config.sim_len-1)){
			//Globals::config.plots_ << ",";
			Globals::config.situ_ << ",";
		}
		//Globals::config.plots_ << endl;
		Globals::config.situ_ << endl;



	}


	default_out << "Average Satisfaction MEC: " << Globals::config.ave_MEC/Globals::config.sim_len << endl;
	default_out << "Average Satisfaction MR: " << Globals::config.ave_MR/Globals::config.sim_len << endl;
	default_out << "Not poor situations : " << Globals::config.nobody_poor << endl;
	default_out << "c1 & c2 poor : " << Globals::config.c1_c2_poor << endl;

	//json jlog_;

	Globals::config.jlog_["ave_mec"] = Globals::config.ave_MEC / Globals::config.sim_len;
	Globals::config.jlog_["ave_mr"] = Globals::config.ave_MR / Globals::config.sim_len;
	Globals::config.jlog_["not_poor_situations"] = Globals::config.nobody_poor;
	Globals::config.jlog_["c1_c2_poor_situations"] = Globals::config.c1_c2_poor;

	Globals::config.situ_ << Globals::config.jlog_;

	// 25/05/2017
	Globals::config.situ_.close();



}

int Planner::RunPlanning(int argc, char *argv[]) {

	/* =========================
	 * initialize parameters
	 * =========================*/
	string solver_type = ChooseSolver(); //"DESPOT";
	bool search_solver;
	int num_runs = 1;
	string world_type = "pomdp";
	string belief_type = "DEFAULT";
	int time_limit = -1;

	option::Option *options = InitializeParamers(argc, argv, solver_type,
			search_solver, num_runs, world_type, belief_type, time_limit);
	if(options==NULL)
		return 0;
	clock_t main_clock_start = clock();

	/* =========================
	 * initialize model
	 * =========================*/
	DSPOMDP *model = InitializeModel(options);
	assert(model != NULL);

	/* =========================
	 * initialize world
	 * =========================*/
	World *world = InitializeWorld(world_type, model, options);
	assert(world != NULL);

	/* =========================
	 * initialize belief
	 * =========================*/
	Belief* belief = model->InitialBelief(world->GetCurrentState(), belief_type);
	assert(belief != NULL);

	/* =========================
	 * initialize solver
	 * =========================*/
	Solver *solver = InitializeSolver(model, belief, solver_type, options);

	/* =========================
	 * initialize logger
	 * =========================*/
	Logger *logger = NULL;
	InitializeLogger(logger, options, model, belief, solver, num_runs,
			main_clock_start, world, world_type, time_limit, solver_type);
	//world->world_seed(world_seed);

	/* =========================
	 * Display parameters
	 * =========================*/
	DisplayParameters(options, model);

	/* =========================
	 * run planning
	 * =========================*/
	logger->InitRound(world->GetCurrentState());
	round_=0; step_=0;
	PlanningLoop(solver, world, logger);
	logger->EndRound();

	PrintResult(1, logger, main_clock_start);

	return 0;
}

} /* namespace despot */
