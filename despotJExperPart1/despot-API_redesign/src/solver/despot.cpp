#include <despot/core/builtin_upper_bounds.h>
#include <despot/core/builtin_lower_bounds.h>

#include <despot/solver/despot.h>
#include <despot/solver/pomcp.h>

using namespace std;

namespace despot {

DESPOT::DESPOT(const DSPOMDP* model, ScenarioLowerBound* lb, ScenarioUpperBound* ub, Belief* belief) :
	Solver(model, belief),
	root_(NULL), 
	lower_bound_(lb),
	upper_bound_(ub) {
	assert(model != NULL);
}

DESPOT::~DESPOT() {
}

ScenarioLowerBound* DESPOT::lower_bound() const {
	return lower_bound_;
}

ScenarioUpperBound* DESPOT::upper_bound() const {
	return upper_bound_;
}

VNode* DESPOT::Trial(VNode* root, RandomStreams& streams,
	ScenarioLowerBound* lower_bound, ScenarioUpperBound* upper_bound,
	const DSPOMDP* model, History& history, SearchStatistics* statistics) {
	VNode* cur = root;

	int hist_size = history.Size();

	do {
		if (statistics != NULL
			&& cur->depth() > statistics->longest_trial_length) {
			statistics->longest_trial_length = cur->depth();
		}

		ExploitBlockers(cur);

		if (Gap(cur) == 0) {
			break;
		}

		if (cur->IsLeaf()) {
			double start = clock();
			Expand(cur, lower_bound, upper_bound, model, streams, history);

			if (statistics != NULL) {
				statistics->time_node_expansion += (double) (clock() - start)
					/ CLOCKS_PER_SEC;
				statistics->num_expanded_nodes++;
				statistics->num_tree_particles += cur->particles().size();
			}
		}

		double start = clock();
		QNode* qstar = SelectBestUpperBoundNode(cur);
		VNode* next = SelectBestWEUNode(qstar);

		if (statistics != NULL) {
			statistics->time_path += (clock() - start) / CLOCKS_PER_SEC;
		}

		if (next == NULL) {
			break;
		}

		cur = next;
		history.Add(qstar->edge(), cur->edge());
	} while (cur->depth() < Globals::config.search_depth && WEU(cur) > 0);

	history.Truncate(hist_size);

	return cur;
}

void DESPOT::ExploitBlockers(VNode* vnode) {
	if (Globals::config.pruning_constant <= 0) {
		return;
	}

	VNode* cur = vnode;
	while (cur != NULL) {
		VNode* blocker = FindBlocker(cur);

		if (blocker != NULL) {
			if (cur->parent() == NULL || blocker == cur) {
				double value = cur->default_move().value;
				cur->lower_bound(value);
				cur->upper_bound(value);
				cur->utility_upper_bound = value;
			} else {
				const map<OBS_TYPE, VNode*>& siblings =
					cur->parent()->children();
				for (map<OBS_TYPE, VNode*>::const_iterator it = siblings.begin();
					it != siblings.end(); it++) {
					VNode* node = it->second;
					double value = node->default_move().value;
					node->lower_bound(value);
					node->upper_bound(value);
					node->utility_upper_bound = value;
				}
			}

			Backup(cur);

			if (cur->parent() == NULL) {
				cur = NULL;
			} else {
				cur = cur->parent()->parent();
			}
		} else {
			break;
		}
	}
}

VNode* DESPOT::ConstructTree(vector<State*>& particles, RandomStreams& streams,
	ScenarioLowerBound* lower_bound, ScenarioUpperBound* upper_bound,
	const DSPOMDP* model, History& history, double timeout,
	SearchStatistics* statistics) {
	if (statistics != NULL) {
		statistics->num_particles_before_search = model->NumActiveParticles();
	}

	for (int i = 0; i < particles.size(); i++) {
		particles[i]->scenario_id = i;
	}

	VNode* root = new VNode(particles);

	logd
		<< "[DESPOT::ConstructTree] START - Initializing lower and upper bounds at the root node.";
	InitBounds(root, lower_bound, upper_bound, streams, history);
	logd
		<< "[DESPOT::ConstructTree] END - Initializing lower and upper bounds at the root node.";

	if (statistics != NULL) {
		statistics->initial_lb = root->lower_bound();
		statistics->initial_ub = root->upper_bound();
	}

	double used_time = 0;
	int num_trials = 0;
	do {
		double start = clock();
		VNode* cur = Trial(root, streams, lower_bound, upper_bound, model, history, statistics);
		used_time += double(clock() - start) / CLOCKS_PER_SEC;

		start = clock();
		Backup(cur);
		if (statistics != NULL) {
			statistics->time_backup += double(clock() - start) / CLOCKS_PER_SEC;
		}
		used_time += double(clock() - start) / CLOCKS_PER_SEC;

		num_trials++;
	} while (used_time * (num_trials + 1.0) / num_trials < timeout
		&& (root->upper_bound() - root->lower_bound()) > 1e-6);

	if (statistics != NULL) {
		statistics->num_particles_after_search = model->NumActiveParticles();
		statistics->num_policy_nodes = root->PolicyTreeSize();
		statistics->num_tree_nodes = root->Size();
		statistics->final_lb = root->lower_bound();
		statistics->final_ub = root->upper_bound();
		statistics->time_search = used_time;
		statistics->num_trials = num_trials;
	}

	return root;
}

void DESPOT::Compare() {
	vector<State*> particles = belief_->Sample(Globals::config.num_scenarios);
	SearchStatistics statistics;

	RandomStreams streams = RandomStreams(Globals::config.num_scenarios,
		Globals::config.search_depth);

	VNode* root = ConstructTree(particles, streams, lower_bound_, upper_bound_,
		model_, history_, Globals::config.time_per_move, &statistics);

	CheckDESPOT(root, root->lower_bound());
	CheckDESPOTSTAR(root, root->lower_bound());
	delete root;
}

void DESPOT::InitLowerBound(VNode* vnode, ScenarioLowerBound* lower_bound,
	RandomStreams& streams, History& history) {
	streams.position(vnode->depth());
	ValuedAction move = lower_bound->Value(vnode->particles(), streams, history);
	move.value *= Globals::Discount(vnode->depth());
	vnode->default_move(move);
	vnode->lower_bound(move.value);
}

void DESPOT::InitUpperBound(VNode* vnode, ScenarioUpperBound* upper_bound,
	RandomStreams& streams, History& history) {
	streams.position(vnode->depth());
	double upper = upper_bound->Value(vnode->particles(), streams, history);
	vnode->utility_upper_bound = upper * Globals::Discount(vnode->depth());
	upper = upper * Globals::Discount(vnode->depth()) - Globals::config.pruning_constant;
	vnode->upper_bound(upper);
}

void DESPOT::InitBounds(VNode* vnode, ScenarioLowerBound* lower_bound,
	ScenarioUpperBound* upper_bound, RandomStreams& streams, History& history) {
	InitLowerBound(vnode, lower_bound, streams, history);
	InitUpperBound(vnode, upper_bound, streams, history);
	if (vnode->upper_bound() < vnode->lower_bound()
		// close gap because no more search can be done on leaf node
		|| vnode->depth() == Globals::config.search_depth - 1) {
		vnode->upper_bound(vnode->lower_bound());
	}
}



void DESPOT::arrow() const{

	// 1. Obtaining the current satisfaction level of NFRs
	model_->GetBelief(*belief_);
	//cout <<  "ARRoW: Mec=true " << Globals::config.current_satisfaction_criteria[0] <<endl;
	//cout <<  "ARRoW: Mr=true " << Globals::config.current_satisfaction_criteria[1] <<endl;

	//Globals::config.jlog_["current_belief_mec_true"]=Globals::config.current_satisfaction_criteria[0];
	//Globals::config.jlog_["current_belief_mr_true"]=Globals::config.current_satisfaction_criteria[1];

	//Globals::config.jsatis_["mec_t"]=Globals::config.current_satisfaction_criteria[0];
	//Globals::config.jsatis_["mr_t"]=Globals::config.current_satisfaction_criteria[1];


	// writing log to a file
	//Globals::config.situ_ << jlog_;
	//Globals::config.situ_ << endl;

	Globals::config.ave_MEC+=Globals::config.current_satisfaction_criteria[0];
    Globals::config.ave_MR+=Globals::config.current_satisfaction_criteria[1];

    // 23/07/2018:
	//Globals::config.ave_MEC_by_action+=Globals::config.current_satisfaction_criteria[0];
    //Globals::config.ave_MR_by_action+=Globals::config.current_satisfaction_criteria[1];


    cout << "showing current POM " << endl;
    showPOM();

    cout << "showing current criteria weights: " << endl;
    compute_criteria_weights();




   	if(DESPOT::isSomebodyPoor()==1){

   		// Only if juggling status  is on we procede to do it!
   	    if (Globals::config.juggling==1){

   	       Globals::config.flagUpdatedRewards=1;

    	   cout << "updating POM " << endl;

    	    updatePOM();


    	    cout << "showing updated  POM " << endl;

    	    showPOM();


    	    cout << "showing new criteria weights: " << endl;
    	    compute_criteria_weights();

    	    cout << "showing new rewards: " << endl;
    	    compute_rewards();
   	    }

   	}else{
   		//json jlog_;
   		Globals::config.nobody_poor+=1;


   	}


}



void DESPOT::showPOM() const{
	//json jlog_;

	//Globals::config.jlog_["pom_criteria"]=Globals::config.pom_criteria;


	for(int i=0;i<Globals::config.nfr_num;i++){
		for(int j=0;j<Globals::config.nfr_num;j++){
			cout << i << "," << j << "=" << Globals::config.pom_criteria[i][j] << " ";
		}
		cout << endl;
	}
	//Globals::config.situ_ << jlog_;

}


/*
 * Based on the current satisfaction values of the NFRs the POM is updated making pairwise comparisons, all cases
 * have been implemented, both poor, both not poor and one poor and the other not poor
 * In progress: compute the AI for the final POM
 */
void DESPOT::updatePOM() const{


	for(int i=0;i<Globals::config.nfr_num;i++){ // rows
		for(int j=0;j<Globals::config.nfr_num;j++){ // columns

			if(i<j){ // if we are at the upper part of the matrix

				// compute factor for the current pairwise comparison
				// case both poor
				if (DESPOT::isPoor(i)==1 && DESPOT::isPoor(j)==1){  // i and j are currently compared criteria
					Globals::config.pom_criteria[i][j]= computing_factor_poors(
															Globals::config.current_satisfaction_criteria[i],
															Globals::config.current_satisfaction_criteria[j],
															Globals::config.criteria_poor_high[i],
															Globals::config.criteria_poor_high[j]);
					Globals::config.c1_c2_poor+=1;


				// case one is poor
				}else if (DESPOT::isPoor(i)==1 || DESPOT::isPoor(j)==1){  // i and j are currently compared criteria
					// both can  be poor, in this case, the comparison is the same? computing_factor is enough?

					// if some criteria is poor, we compute the juggling factor :)

					Globals::config.pom_criteria[i][j]= computing_factor(
															Globals::config.current_satisfaction_criteria[i],
															Globals::config.current_satisfaction_criteria[j],
															Globals::config.criteria_poor_high[i],
															Globals::config.criteria_poor_high[j]);
					//flag_poor=1;

				// case no poors
				}else{
					// not poor but equal, it is necessary to compute a factor here :)
					//it means if any pair is not poor, based on their current satisfaction values, we still will update the factor ? is this right?
					// what about the initial preferences?

					// 03/12/2017: to check: it looks like if we aply softer approach, when the elements are not
					if(Globals::config.not_change_pom_if_not_poor!=1){
						Globals::config.pom_criteria[i][j]= computing_factor_no_poors(
															Globals::config.current_satisfaction_criteria[i],
															Globals::config.current_satisfaction_criteria[j],
															Globals::config.criteria_poor_high[i],
															Globals::config.criteria_poor_high[j]);
					}
				}

			}else if(i>j){// we are at the lower part of the matrix, i.e., the value is the opposite of the upper
				// lookup
				Globals::config.pom_criteria[i][j]=-1*Globals::config.pom_criteria[j][i];
			}

		}
	}


}

int DESPOT::isSomebodyPoor() const{
	int poor=0;
	for(int i=0;i<Globals::config.nfr_num;i++){
		if (Globals::config.current_satisfaction_criteria[i] < Globals::config.criteria_poor_high[i])poor=1;
	}
	return poor;
}

int DESPOT::isPoor(int posi_criterion) const{
	int poor=0;
	if (Globals::config.current_satisfaction_criteria[posi_criterion] < Globals::config.criteria_poor_high[posi_criterion] ) poor=1;
	return poor;
}



double DESPOT::computing_factor(double satisf_c1, double satisf_c2, double c1_poor_high, double c2_poor_high) const{

	//json jlog_;



	double factor=0.0;
	double tota,slice,botton_cate,c2_poor,c1_poor;
	double cates_ranges[Globals::config.num_scale_categories][2];

	double poor_cate_upper_bound,poor_cate_lower_bound,current_value_poor;
	double proportion_poor_space, lower_bound_factor,valid_space_factor,decrement_to_apply,final_factor;


	/*
	 * if c1 is poor we create the dynamic categories based on poor boundary of c2
	 */
	if(satisf_c1<c1_poor_high){
		tota=1-c2_poor_high;
		botton_cate=c2_poor_high;
		c1_poor=1;

	/*
	 * if c2 is poor we create the dynamic categories based on poor boundary of c1
	 */
	}else if(satisf_c2<c2_poor_high){
		tota=1-c1_poor_high;
		botton_cate=c1_poor_high;
		c2_poor=1;
	}

	slice=tota/Globals::config.num_scale_categories;

	// this part should be design time because is information
	// we already have at design time
	for(int i=0;i<Globals::config.num_scale_categories;i++){

		cates_ranges[i][0]=botton_cate;
		cout << "[" <<i<<"]" <<"[0]" << cates_ranges[i][0] <<  endl;
		botton_cate+=slice;
		cates_ranges[i][1]=botton_cate;
		cout << "[" <<i<<"]" <<"[1]" << cates_ranges[i][1] <<  endl;

	}

	/*
	 * if c1 is poor factor is computed based on the current satisfaction of c2. Factor is positive because
	 * it means the computed value (i.e.,factor) is given more importance to c1 in relation to c2
	 */
	if(c1_poor==1){
		poor_cate_upper_bound=c1_poor_high;
		poor_cate_lower_bound=0.0;
		current_value_poor=satisf_c1;

		// be aware of finding 1/12/2017: what if current satisfaction is 1 for
		for(int i=0;i<Globals::config.num_scale_categories;i++){

			if(satisf_c2>=cates_ranges[i][0] && satisf_c2<cates_ranges[i][1]){
				factor=Globals::config.factors[i];
				cout<< "case: c1 poor, c2 within " << cates_ranges[i][0]<< " and " << cates_ranges[i][1] << endl;
				//Globals::config.jlog_["case_c1_poor"]=factor;

			}
		}



		/*
		 * if c2 is poor factor is computed based on the current satisfaction of c1. Factor is negative because
		 * it means the computed value (i.e.,factor) is given more importance to c2 in relation to c1
		 */
	}else{
		poor_cate_upper_bound=c2_poor_high;
		poor_cate_lower_bound=0.0;
		current_value_poor=satisf_c2;

		for(int i=0;i<Globals::config.num_scale_categories;i++){

				if(satisf_c1>=cates_ranges[i][0] && satisf_c1<cates_ranges[i][1]){
					factor=Globals::config.factors[i]*-1;
					cout<< "case: c2 poor, c1 within " << cates_ranges[i][0]<< " and " << cates_ranges[i][1] << endl;
					//Globals::config.jlog_["case_c2_poor"]=factor;
				}

		}
	}



	/*
	 * Softer approach for factor:
	 * Given that is not the same a poor value close to the boundary than a poor value close to 0, we reduce the computed factor  to get a value
	 * between 0 and the current computed factor, closer the current poor value is to the boundary ... closer the new factor will be to 0 ... conversely,
	 * closer the current poor value is to 0, closer to the new factor will be to the originally computed factor ...
	 */

	if(Globals::config.softer_approach==1){
	    proportion_poor_space=1/(poor_cate_upper_bound/current_value_poor);
	    lower_bound_factor=0.0;

	   	valid_space_factor=factor-lower_bound_factor;

	   	decrement_to_apply=proportion_poor_space*valid_space_factor;
	   	factor=factor-decrement_to_apply;
	   	Globals::config.jlog_["softer_factor"]=factor;
		cout << "the smooth factor " << factor << endl;
	}

	//Globals::config.situ_ << jlog_;
	//Globals::config.situ_ << endl;


	return factor;
}


/*
 * Despite the compared NFRs are not poor, it is still necessary to compute the factor because we need to keep congruency among all the element at the runtime POM of the NFrs ;).
 * The strategy is to create categories from 1 to 8 for both compared NFRs (based on their own poor boundaries) and assign a factor to each one, then, the fina factor
 * will be different of both (f2 - f1 )
 */
double DESPOT::computing_factor_no_poors(double satisf_c1, double satisf_c2, double c1_poor_high, double c2_poor_high) const{
	double factorc1,factorc2=0.0;
	double tota_c1, tota_c2, slice_c1,slice_c2,c2_poor,c1_poor, botton_cate_c1, botton_cate_c2;
	double cates_ranges_c1 [Globals::config.num_scale_categories][2];
	double cates_ranges_c2 [Globals::config.num_scale_categories][2];

	// cates for c1 and c2
	botton_cate_c1=c1_poor_high;
	botton_cate_c2=c2_poor_high;
	tota_c1=1-c1_poor_high;
	tota_c2=1-c2_poor_high;

	slice_c1=tota_c1/Globals::config.num_scale_categories;
	slice_c2=tota_c2/Globals::config.num_scale_categories;


	// this part should be design time because is information
	// we already have at design time
	for(int i=0;i<Globals::config.num_scale_categories;i++){
		cates_ranges_c1[i][0]=botton_cate_c1;
		cout << "[" <<i<<"]" <<"[0]" << cates_ranges_c1[i][0] <<  endl;
		botton_cate_c1+=slice_c1;
		cates_ranges_c1[i][1]=botton_cate_c1;
		cout << "[" <<i<<"]" <<"[1]" << cates_ranges_c1[i][1] <<  endl;

		cates_ranges_c2[i][0]=botton_cate_c2;
		cout << "[" <<i<<"]" <<"[0]" << cates_ranges_c2[i][0] <<  endl;
		botton_cate_c2+=slice_c2;
		cates_ranges_c2[i][1]=botton_cate_c2;
		cout << "[" <<i<<"]" <<"[1]" << cates_ranges_c2[i][0] <<  endl;


	}


	/*
	 * The NFR with better satisfaction value will have less weight that the NFR with poorer NFR ;)
	 */

	for(int i=0;i<Globals::config.num_scale_categories;i++){

		if(satisf_c1>=cates_ranges_c1[i][0] && satisf_c1<cates_ranges_c1[i][1]){
			factorc1=Globals::config.factors[i];
		}
		if(satisf_c2>=cates_ranges_c2[i][0] && satisf_c2<cates_ranges_c2[i][1]){
			factorc2=Globals::config.factors[i];
		}
	}

	//json jlog_;
	//Globals::config.jlog_["case_no_poors_f2_f1"]={factorc2,factorc1};


	cout<< "f2 - f1 " << factorc2 << " - " << factorc1 << endl;

	//Globals::config.situ_ << jlog_;
	//Globals::config.situ_ << endl;

	return factorc2-factorc1;
}



/*
 * Equivalent to computing factor no poors
 */
double DESPOT::computing_factor_poors(double satisf_c1, double satisf_c2, double c1_poor_high, double c2_poor_high) const{
	double factorc1,factorc2=0.0;
	double tota_c1, tota_c2, slice_c1,slice_c2,c2_poor,c1_poor, botton_cate_c1, botton_cate_c2;
	double cates_ranges_c1 [Globals::config.num_scale_categories][2];
	double cates_ranges_c2 [Globals::config.num_scale_categories][2];

	// cates for c1 and c2
	botton_cate_c1=0;
	botton_cate_c2=0;
	tota_c1=c1_poor_high;
	tota_c2=c2_poor_high;

	slice_c1=tota_c1/Globals::config.num_scale_categories;
	slice_c2=tota_c2/Globals::config.num_scale_categories;


	// this part should be design time because is information
	// we already have at design time
	for(int i=0;i<Globals::config.num_scale_categories;i++){
		cates_ranges_c1[i][0]=botton_cate_c1;
		cout << "[" <<i<<"]" <<"[0]" << cates_ranges_c1[i][0] <<  endl;
		botton_cate_c1+=slice_c1;
		cates_ranges_c1[i][1]=botton_cate_c1;
		cout << "[" <<i<<"]" <<"[1]" << cates_ranges_c1[i][1] <<  endl;

		cates_ranges_c2[i][0]=botton_cate_c2;
		cout << "[" <<i<<"]" <<"[0]" << cates_ranges_c2[i][0] <<  endl;
		botton_cate_c2+=slice_c2;
		cates_ranges_c2[i][1]=botton_cate_c2;
		cout << "[" <<i<<"]" <<"[1]" << cates_ranges_c2[i][0] <<  endl;


	}


	/*
	 * The NFR with better satisfaction value will have less weight that the NFR with poorer value ;)
	 */

	for(int i=0;i<Globals::config.num_scale_categories;i++){

		if(satisf_c1>=cates_ranges_c1[i][0] && satisf_c1<cates_ranges_c1[i][1]){
			factorc1=Globals::config.factors[i];
		}
		if(satisf_c2>=cates_ranges_c2[i][0] && satisf_c2<cates_ranges_c2[i][1]){
			factorc2=Globals::config.factors[i];
		}
	}

	//json jlog_;
	//Globals::config.jlog_["case_both_poors_f2_f1"]={factorc2,factorc1};


	cout<< "f2 - f1 (both poor) " << factorc2 << " - " << factorc1 << endl;

	//Globals::config.situ_ << jlog_;
	//Globals::config.situ_ << endl;

	return factorc2-factorc1;
}


/*
 * Based on the current POM this method computes the weights of the criteria and store them on the variable updated_weights_criteria
 */
void DESPOT::compute_criteria_weights() const{


	// initializing variable
	for(int i=0;i<Globals::config.nfr_num;i++){
		Globals::config.updated_weights_criteria[i]=0;
	}

	for(int i=0;i<Globals::config.nfr_num;i++){

		double row=0;
		for(int j=0;j<Globals::config.nfr_num;j++){

		// average per row
			row+=Globals::config.pom_criteria[i][j];
		}
		Globals::config.updated_weights_criteria[i]=(row/Globals::config.nfr_num)+Globals::config.kappa;
	}

	// computing tota weights before normalization
	double tota=0;
	for(int i=0;i<Globals::config.nfr_num;i++){
		tota+=Globals::config.updated_weights_criteria[i];
	}


	// normalizing and showing
	cout << "Criteria weights: " << endl;
 	for(int i=0;i<Globals::config.nfr_num;i++){
		Globals::config.updated_weights_criteria[i]=Globals::config.updated_weights_criteria[i]/tota;
		cout << Globals::config.updated_weights_criteria[i] << " ";
	}
	cout << endl;

	//json jlog_;
//	Globals::config.jlog_["weights_criteria"]=Globals::config.updated_weights_criteria;
	//Globals::config.situ_ << jlog_;

}


/*
 * This method is more flexible but it has limitations, updated weights criteria is not with a for, is directly accessed by indexes 0 and 1 because of the arrays mec and mr
 * but is more flexible than previous version
 */
void DESPOT::compute_rewards() const{

	cout << "updated rewards: " << endl;
	for(int i=0;i<Globals::config.action_states_num;i++){
		for(int j=0;j<Globals::config.nfr_states_num;j++){

			//Globals::config.rewards[i][j]=updated_weights_criteria[0]*rewards_c1[i]+updated_weights_criteria[1]*rewards_c2[i]+updated_weights_criteria[2]*rewards_c3[i];
			Globals::config.rewards[i][j]=Globals::config.updated_weights_criteria[0]*Globals::config.rewards_mec[i][j]+
											Globals::config.updated_weights_criteria[1]*Globals::config.rewards_mr[i][j];

			cout << Globals::config.rewards[i][j] << endl;
		}
	}

	/*
	 * Computing maxReward, minReward, and minAction
	 */
	Globals::config.maxReward=0;
	double minReward[2]={1.0,1.0}; //MEC=1,MR=1;
	Globals::config.minReward=1;


	// Max reward
	for(int i=0;i<Globals::config.action_states_num;i++){
 		for(int j=0;j<Globals::config.nfr_states_num;j++){
 			if(Globals::config.rewards[i][j]>Globals::config.maxReward){
 				Globals::config.maxReward=Globals::config.rewards[i][j];
 			}

 		}
	}


	// Min reward for MEC, MR
	for(int i=0;i<Globals::config.action_states_num;i++){
 		for(int j=0;j<Globals::config.nfr_states_num;j++){
 			if(Globals::config.rewards[i][j]<minReward[i]){
 				minReward[i]=Globals::config.rewards[i][j];
 			}

 		}
	}

	// We choose the biggest one
	Globals::config.minReward= 0;
	for(int i=0;i<Globals::config.nfr_num;i++){
		if(minReward[i]>Globals::config.minReward){
				Globals::config.minReward=minReward[i];
				Globals::config.minAction=i;
			}

	}




}






ValuedAction DESPOT::Search() {
	if (logging::level() >= logging::DEBUG) {
		model_->PrintBelief(*belief_);
	}

	if (Globals::config.time_per_move <= 0) // Return a random action if no time is allocated for planning
		return ValuedAction(Random::RANDOM.NextInt(model_->NumActions()),
			Globals::NEG_INFTY);

	double start = get_time_second();


	// 08/08/2016 ARRoW logic for juggling could be located here :)
	// 09/08/2018 we also obtain here the current belief for MEC and MR
	// using the variables Globals::config.current_satisfaction_criteria[0] and ...[1]
	arrow();

	vector<State*> particles = belief_->Sample(Globals::config.num_scenarios);
	logi << "[DESPOT::Search] Time for sampling " << particles.size()
		<< " particles: " << (get_time_second() - start) << "s" << endl;

	statistics_ = SearchStatistics();

	start = get_time_second();
	static RandomStreams streams = RandomStreams(Globals::config.num_scenarios,
		Globals::config.search_depth);

	LookaheadUpperBound* ub = dynamic_cast<LookaheadUpperBound*>(upper_bound_);
	if (ub != NULL) { // Avoid using new streams for LookaheadUpperBound
		static bool initialized = false;
		if (!initialized ) {
			lower_bound_->Init(streams);
			upper_bound_->Init(streams);
			initialized = true;
		}
	} else {
		streams = RandomStreams(Globals::config.num_scenarios,
			Globals::config.search_depth);
		lower_bound_->Init(streams);
		upper_bound_->Init(streams);
	}

	root_ = ConstructTree(particles, streams, lower_bound_, upper_bound_,
		model_, history_, Globals::config.time_per_move, &statistics_);
	logi << "[DESPOT::Search] Time for tree construction: "
		<< (get_time_second() - start) << "s" << endl;

	start = get_time_second();
	root_->Free(*model_);
	logi << "[DESPOT::Search] Time for freeing particles in search tree: "
		<< (get_time_second() - start) << "s" << endl;

	ValuedAction astar = OptimalAction(root_);
	start = get_time_second();
	delete root_;

	logi << "[DESPOT::Search] Time for deleting tree: "
		<< (get_time_second() - start) << "s" << endl;
	logi << "[DESPOT::Search] Search statistics:" << endl << statistics_
		<< endl;

	return astar;
}

double DESPOT::CheckDESPOT(const VNode* vnode, double regularized_value) {
	cout
		<< "--------------------------------------------------------------------------------"
		<< endl;

	const vector<State*>& particles = vnode->particles();
	vector<State*> copy;
	for (int i = 0; i < particles.size(); i ++) {
		copy.push_back(model_->Copy(particles[i]));
	}
	VNode* root = new VNode(copy);

	double pruning_constant = Globals::config.pruning_constant;
	Globals::config.pruning_constant = 0;

	RandomStreams streams = RandomStreams(Globals::config.num_scenarios,
		Globals::config.search_depth);

	streams.position(0);
	InitBounds(root, lower_bound_, upper_bound_, streams, history_);

	double used_time = 0;
	int num_trials = 0, prev_num = 0;
	double pruned_value;
	do {
		double start = clock();
		VNode* cur = Trial(root, streams, lower_bound_, upper_bound_, model_, history_);
		num_trials++;
		used_time += double(clock() - start) / CLOCKS_PER_SEC;

		start = clock();
		Backup(cur);
		used_time += double(clock() - start) / CLOCKS_PER_SEC;

		if (double(num_trials - prev_num) > 0.05 * prev_num) {
			ACT_TYPE pruned_action;
			Globals::config.pruning_constant = pruning_constant;
			VNode* pruned = Prune(root, pruned_action, pruned_value);
			Globals::config.pruning_constant = 0;
			prev_num = num_trials;

			pruned->Free(*model_);
			delete pruned;

			cout << "# trials = " << num_trials << "; target = "
				<< regularized_value << ", current = " << pruned_value
				<< ", l = " << root->lower_bound() << ", u = "
				<< root->upper_bound() << "; time = " << used_time << endl;

			if (pruned_value >= regularized_value) {
				break;
			}
		}
	} while (true);

	cout << "DESPOT: # trials = " << num_trials << "; target = "
		<< regularized_value << ", current = " << pruned_value << ", l = "
		<< root->lower_bound() << ", u = " << root->upper_bound() << "; time = "
		<< used_time << endl;
	Globals::config.pruning_constant = pruning_constant;
	cout
		<< "--------------------------------------------------------------------------------"
		<< endl;

	root->Free(*model_);
	delete root;

	return used_time;
}

double DESPOT::CheckDESPOTSTAR(const VNode* vnode, double regularized_value) {
	cout
		<< "--------------------------------------------------------------------------------"
		<< endl;

	const vector<State*>& particles = vnode->particles();
	vector<State*> copy;
	for (int i = 0; i < particles.size(); i++) {
		copy.push_back(model_->Copy(particles[i]));
	}
	VNode* root = new VNode(copy);

	RandomStreams streams = RandomStreams(Globals::config.num_scenarios,
		Globals::config.search_depth);
	InitBounds(root, lower_bound_, upper_bound_, streams, history_);

	double used_time = 0;
	int num_trials = 0;
	do {
		double start = clock();
		VNode* cur = Trial(root, streams, lower_bound_, upper_bound_, model_, history_);
		num_trials++;
		used_time += double(clock() - start) / CLOCKS_PER_SEC;

		start = clock();
		Backup(cur);
		used_time += double(clock() - start) / CLOCKS_PER_SEC;
	} while (root->lower_bound() < regularized_value);

	cout << "DESPOT: # trials = " << num_trials << "; target = "
		<< regularized_value << ", current = " << root->lower_bound()
		<< ", l = " << root->lower_bound() << ", u = " << root->upper_bound()
		<< "; time = " << used_time << endl;
	cout
		<< "--------------------------------------------------------------------------------"
		<< endl;

	root->Free(*model_);
	delete root;

	return used_time;
}

VNode* DESPOT::Prune(VNode* vnode, ACT_TYPE& pruned_action, double& pruned_value) {
	vector<State*> empty;
	VNode* pruned_v = new VNode(empty, vnode->depth(), NULL,
		vnode->edge());

	vector<QNode*>& children = vnode->children();
	ACT_TYPE astar = -1;
	double nustar = Globals::NEG_INFTY;
	QNode* qstar = NULL;
	for (int i = 0; i < children.size(); i++) {
		QNode* qnode = children[i];
		double nu;
		QNode* pruned_q = Prune(qnode, nu);

		if (nu > nustar) {
			nustar = nu;
			astar = qnode->edge();

			if (qstar != NULL) {
				delete qstar;
			}

			qstar = pruned_q;
		} else {
			delete pruned_q;
		}
	}

	if (nustar < vnode->default_move().value) {
		nustar = vnode->default_move().value;
		astar = vnode->default_move().action;
		delete qstar;
	} else {
		pruned_v->children().push_back(qstar);
		qstar->parent(pruned_v);
	}

	pruned_v->lower_bound(vnode->lower_bound()); // for debugging
	pruned_v->upper_bound(vnode->upper_bound());

	pruned_action = astar;
	pruned_value = nustar;

	return pruned_v;
}

QNode* DESPOT::Prune(QNode* qnode, double& pruned_value) {
	QNode* pruned_q = new QNode((VNode*) NULL, qnode->edge());
	pruned_value = qnode->step_reward - Globals::config.pruning_constant;
	map<OBS_TYPE, VNode*>& children = qnode->children();
	for (map<OBS_TYPE, VNode*>::iterator it = children.begin();
		it != children.end(); it++) {
		ACT_TYPE astar;
		double nu;
		VNode* pruned_v = Prune(it->second, astar, nu);
		if (nu == it->second->default_move().value) {
			delete pruned_v;
		} else {
			pruned_q->children()[it->first] = pruned_v;
			pruned_v->parent(pruned_q);
		}
		pruned_value += nu;
	}

	pruned_q->lower_bound(qnode->lower_bound()); // for debugging
	pruned_q->upper_bound(qnode->upper_bound()); // for debugging

	return pruned_q;
}

ValuedAction DESPOT::OptimalAction(VNode* vnode) {
	ValuedAction astar(-1, Globals::NEG_INFTY);
	for (ACT_TYPE action = 0; action < vnode->children().size(); action++) {
		QNode* qnode = vnode->Child(action);

		cout<< "action.value " << qnode->lower_bound() << endl;
		if(action==0){
			Globals::config.jlog_["ev.mst"]=qnode->lower_bound();
		}else{
			Globals::config.jlog_["ev.rt"]=qnode->lower_bound();
		}

		if (qnode->lower_bound() > astar.value) {
			astar = ValuedAction(action, qnode->lower_bound());
		}
	}

	if (vnode->default_move().value > astar.value) {
		astar = vnode->default_move();
	}

	return astar;
}

double DESPOT::Gap(VNode* vnode) {
	return (vnode->upper_bound() - vnode->lower_bound());
}

double DESPOT::WEU(VNode* vnode) {
	return WEU(vnode, Globals::config.xi);
}

// Can pass root as an argument, but will not affect performance much
double DESPOT::WEU(VNode* vnode, double xi) {
	VNode* root = vnode;
	while (root->parent() != NULL) {
		root = root->parent()->parent();
	}
	return Gap(vnode) - xi * vnode->Weight() * Gap(root);
}

VNode* DESPOT::SelectBestWEUNode(QNode* qnode) {
	double weustar = Globals::NEG_INFTY;
	VNode* vstar = NULL;
	map<OBS_TYPE, VNode*>& children = qnode->children();
	for (map<OBS_TYPE, VNode*>::iterator it = children.begin();
		it != children.end(); it++) {
		VNode* vnode = it->second;

		double weu = WEU(vnode);
		if (weu >= weustar) {
			weustar = weu;
			vstar = vnode->vstar;
		}
	}
	return vstar;
}

QNode* DESPOT::SelectBestUpperBoundNode(VNode* vnode) {
	int astar = -1;
	double upperstar = Globals::NEG_INFTY;
	for (ACT_TYPE action = 0; action < vnode->children().size(); action++) {
		QNode* qnode = vnode->Child(action);

		if (qnode->upper_bound() > upperstar) {
			upperstar = qnode->upper_bound();
			astar = action;
		}
	}
	assert(astar >= 0);
	return vnode->Child(astar);
}

void DESPOT::Update(VNode* vnode) {
	if (vnode->IsLeaf()) {
		return;
	}

	double lower = vnode->default_move().value;
	double upper = vnode->default_move().value;
	double utility_upper = Globals::NEG_INFTY;

	for (ACT_TYPE action = 0; action < vnode->children().size(); action++) {
		QNode* qnode = vnode->Child(action);

		lower = max(lower, qnode->lower_bound());
		upper = max(upper, qnode->upper_bound());
		utility_upper = max(utility_upper, qnode->utility_upper_bound);
	}

	if (lower > vnode->lower_bound()) {
		vnode->lower_bound(lower);
	}
	if (upper < vnode->upper_bound()) {
		vnode->upper_bound(upper);
	}
	if (utility_upper < vnode->utility_upper_bound) {
		vnode->utility_upper_bound = utility_upper;
	}
}

void DESPOT::Update(QNode* qnode) {
	double lower = qnode->step_reward;
	double upper = qnode->step_reward;
	double utility_upper = qnode->step_reward
		+ Globals::config.pruning_constant;

	map<OBS_TYPE, VNode*>& children = qnode->children();
	for (map<OBS_TYPE, VNode*>::iterator it = children.begin();
		it != children.end(); it++) {
		VNode* vnode = it->second;

		lower += vnode->lower_bound();
		upper += vnode->upper_bound();
		utility_upper += vnode->utility_upper_bound;
	}

	if (lower > qnode->lower_bound()) {
		qnode->lower_bound(lower);
	}
	if (upper < qnode->upper_bound()) {
		qnode->upper_bound(upper);
	}
	if (utility_upper < qnode->utility_upper_bound) {
		qnode->utility_upper_bound = utility_upper;
	}
}

void DESPOT::Backup(VNode* vnode) {
	int iter = 0;
	logd << "- Backup " << vnode << " at depth " << vnode->depth() << endl;
	while (true) {
		logd << " Iter " << iter << " " << vnode << endl;

		Update(vnode);

		QNode* parentq = vnode->parent();
		if (parentq == NULL) {
			break;
		}

		Update(parentq);
		logd << " Updated Q-node to (" << parentq->lower_bound() << ", "
			<< parentq->upper_bound() << ")" << endl;

		vnode = parentq->parent();
		iter++;
	}
	logd << "* Backup complete!" << endl;
}

VNode* DESPOT::FindBlocker(VNode* vnode) {
	VNode* cur = vnode;
	int count = 1;
	while (cur != NULL) {
		if (cur->utility_upper_bound - count * Globals::config.pruning_constant
			<= cur->default_move().value) {
			break;
		}
		count++;
		if (cur->parent() == NULL) {
			cur = NULL;
		} else {
			cur = cur->parent()->parent();
		}
	}
	return cur;
}

void DESPOT::Expand(VNode* vnode,
	ScenarioLowerBound* lower_bound, ScenarioUpperBound* upper_bound,
	const DSPOMDP* model, RandomStreams& streams,
	History& history) {
	vector<QNode*>& children = vnode->children();
	logd << "- Expanding vnode " << vnode << endl;
	for (ACT_TYPE action = 0; action < model->NumActions(); action++) {
		logd << " Action " << action << endl;
		QNode* qnode = new QNode(vnode, action);
		children.push_back(qnode);

		Expand(qnode, lower_bound, upper_bound, model, streams, history);
	}
	logd << "* Expansion complete!" << endl;
}

void DESPOT::Expand(QNode* qnode, ScenarioLowerBound* lb,
	ScenarioUpperBound* ub, const DSPOMDP* model,
	RandomStreams& streams,
	History& history) {
	VNode* parent = qnode->parent();
	streams.position(parent->depth());
	map<OBS_TYPE, VNode*>& children = qnode->children();

	const vector<State*>& particles = parent->particles();

	double step_reward = 0;

	// Partition particles by observation
	map<OBS_TYPE, vector<State*> > partitions;
	OBS_TYPE obs;
	double reward;
	for (int i = 0; i < particles.size(); i++) {
		State* particle = particles[i];
		logd << " Original: " << *particle << endl;

		State* copy = model->Copy(particle);

		logd << " Before step: " << *copy << endl;

		bool terminal = model->Step(*copy, streams.Entry(copy->scenario_id),
			qnode->edge(), reward, obs);

		step_reward += reward * copy->weight;

		logd << " After step: " << *copy << " " << (reward * copy->weight)
			<< " " << reward << " " << copy->weight << endl;

		if (!terminal) {
			partitions[obs].push_back(copy);
		} else {
			model->Free(copy);
		}
	}
	step_reward = Globals::Discount(parent->depth()) * step_reward
		- Globals::config.pruning_constant;//pruning_constant is used for regularization

	double lower_bound = step_reward;
	double upper_bound = step_reward;

	// Create new belief nodes
	for (map<OBS_TYPE, vector<State*> >::iterator it = partitions.begin();
		it != partitions.end(); it++) {
		OBS_TYPE obs = it->first;
		logd << " Creating node for obs " << obs << endl;
		VNode* vnode = new VNode(partitions[obs], parent->depth() + 1,
			qnode, obs);
		logd << " New node created!" << endl;
		children[obs] = vnode;

		history.Add(qnode->edge(), obs);
		InitBounds(vnode, lb, ub, streams, history);
		history.RemoveLast();
		logd << " New node's bounds: (" << vnode->lower_bound() << ", "
			<< vnode->upper_bound() << ")" << endl;

		lower_bound += vnode->lower_bound();
		upper_bound += vnode->upper_bound();
	}

	qnode->step_reward = step_reward;
	qnode->lower_bound(lower_bound);
	qnode->upper_bound(upper_bound);
	qnode->utility_upper_bound = upper_bound + Globals::config.pruning_constant;

	qnode->default_value = lower_bound; // for debugging
}

ValuedAction DESPOT::Evaluate(VNode* root, vector<State*>& particles,
	RandomStreams& streams, POMCPPrior* prior, const DSPOMDP* model) {
	double value = 0;

	for (int i = 0; i < particles.size(); i++) {
		particles[i]->scenario_id = i;
	}

	for (int i = 0; i < particles.size(); i++) {
		State* particle = particles[i];
		VNode* cur = root;
		State* copy = model->Copy(particle);
		double discount = 1.0;
		double val = 0;
		int steps = 0;

		while (!streams.Exhausted()) {
			ACT_TYPE action =
				(cur != NULL) ?
					OptimalAction(cur).action : prior->GetAction(*copy);

			assert(action != -1);

			double reward;
			OBS_TYPE obs;
			bool terminal = model->Step(*copy, streams.Entry(copy->scenario_id),
				action, reward, obs);

			val += discount * reward;
			discount *= Globals::Discount();

			if (!terminal) {
				prior->Add(action, obs);
				streams.Advance();
				steps++;

				if (cur != NULL && !cur->IsLeaf()) {
					QNode* qnode = cur->Child(action);
					map<OBS_TYPE, VNode*>& vnodes = qnode->children();
					cur = vnodes.find(obs) != vnodes.end() ? vnodes[obs] : NULL;
				}
			} else {
				break;
			}
		}

		for (int i = 0; i < steps; i++) {
			streams.Back();
			prior->PopLast();
		}

		model->Free(copy);

		value += val;
	}

	return ValuedAction(OptimalAction(root).action, value / particles.size());
}

void DESPOT::belief(Belief* b) {
	logi << "[DESPOT::belief] Start: Set initial belief." << endl;
	belief_ = b;
	history_.Truncate(0);

	//lower_bound_->belief(b); // needed for POMCPScenarioLowerBound
	logi << "[DESPOT::belief] End: Set initial belief." << endl;
}

void DESPOT::BeliefUpdate(ACT_TYPE action, OBS_TYPE obs) {
	double start = get_time_second();

	model_->GetBelief(*belief_);
	cout << "Previous belief (the action shown was selected with this belief)" << endl;
	cout <<  "ARRoW: Mec=true " << Globals::config.current_satisfaction_criteria[0] <<endl;
	cout <<  "ARRoW: Mr=true " << Globals::config.current_satisfaction_criteria[1] <<endl;
	Globals::config.jlog_["current_belief_mec_true_(s)"]=Globals::config.current_satisfaction_criteria[0];
	Globals::config.jlog_["current_belief_mr_true_(s)"]=Globals::config.current_satisfaction_criteria[1];


	belief_->Update(action, obs);
	history_.Add(action, obs);

	model_->GetBelief(*belief_);
	cout << "Current belief (based on the new states and obtained observations after performing the action)" << endl;
	int poorvalue=0;
	cout <<  "ARRoW: Mec=true " << Globals::config.current_satisfaction_criteria[0] <<endl;
	cout <<  "ARRoW: Mr=true " << Globals::config.current_satisfaction_criteria[1] <<endl;

	Globals::config.jlog_["updated_belief_mec_true_(s')"]=Globals::config.current_satisfaction_criteria[0];
	Globals::config.jlog_["updated_belief_mr_true_(s')"]=Globals::config.current_satisfaction_criteria[1];

	cout << "current action : "<< action << endl;

	int situation=0;


	//lower_bound_->belief(belief_);

	logi << "[Solver::Update] Updated belief, history and root with action "
		<< action << ", observation " << obs
		<< " in " << (get_time_second() - start) << "s" << endl;
}

} // namespace despot
