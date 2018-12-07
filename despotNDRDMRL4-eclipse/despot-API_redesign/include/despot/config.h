#ifndef CONFIG_H
#define CONFIG_H

#include <string>

#include "json.hpp"
using json = nlohmann::json;

namespace despot {

struct Config {

	// 06/06/2018
		json jsatis_,jlog_;

		std::ofstream situ_,plots_;

		int current_time_slice_;

		// 29/11/2017 This variables should be read from json file
		int nfr_num;// 2
		int nfr_states_num; //4
		int obs_states_num; //9
		int action_states_num; //2
		int rewards_num;//8

		int obs_ranges_num=3;//3
		int parents_tranmodel_num=8;

		//30/11/2017 variables for possible states and possible observations
		int nfr_states[4][2];
		int obs_states[9];
		std::string obs_descriptions[9];

		double pom_criteria[2][2];
		double criteria_poor_high[2];

		double current_satisfaction_criteria[2];

		double updated_weights_criteria[2];

		int kappa;



		double rewards[2][4];
		double rewards_mec[2][4];
		double rewards_mr[2][4];


		int jMOC_T=0,jMR_T=2,jMOC_F=1,jMR_F=3;

		double ini_transition_model[2][8][2];
		double ini_transition_model_saved[2][8][2];

		double transition_model[2][4][4];
		double transition_model_saved[2][4][4];


		double ini_observation_model[2][4][3];
		double ini_observation_model_saved[2][4][3];


		double observation_model[2][4][9];
		double observation_model_saved[2][4][9];


		double factors[8];
		int num_scale_categories;
		int not_poor_poor;
		int softer_approach;
		int not_change_pom_if_not_poor;

		double maxReward;
		double minReward;

		double ave_MEC_by_action;
		double ave_MR_by_action;
		int cont_current_action;
		int current_action, previous_action;

		double ave_MEC;
		double ave_MR,nobody_poor,c1_c2_poor;

		int juggling;

		// 21/06/2018
		int flagUpdatedRewards;
		double obsProb;
		int obsCode;
		std::string obsDes;

		int minAction;



		// 14/10/17: temporal variables because in new logic possibly wont be needed
		double mec_poor_low;
		double mec_poor_high;

		double mr_poor_low;
		double mr_poor_high;

		// 09/07/2018

		int activateDeviation;

		double prob_devi_upper;
		double prob_devi_lower;

		int ts_devi_upper;
		int ts_devi_lower;

		double devi_noise;



	int search_depth;
	double discount;
	unsigned int root_seed;
	double time_per_move;  // CPU time available to construct the search tree
	int num_scenarios;
	double pruning_constant;
	double xi; // xi * gap(root) is the target uncertainty at the root.
	int sim_len; // Number of steps to run the simulation for.
  std::string default_action;
	int max_policy_sim_len; // Maximum number of steps for simulating the default policy
	double noise;
	bool silence;

	Config() :
		search_depth(90),
		discount(0.95),
		root_seed(42),
		time_per_move(1),
		num_scenarios(500),
		pruning_constant(0),
		xi(0.95),
		sim_len(90),
		default_action(""),
		max_policy_sim_len(90),
		noise(0.1),
		silence(false) {
	}
};

} // namespace despot

#endif
