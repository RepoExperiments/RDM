#ifndef DESPOT_H
#define DESPOT_H

#include <despot/core/solver.h>
#include <despot/interface/pomdp.h>
#include <despot/interface/belief.h>
#include <despot/core/node.h>
#include <despot/core/globals.h>
#include <despot/core/history.h>
#include <despot/random_streams.h>

namespace despot {

class DESPOT: public Solver {
friend class VNode;

protected:
	VNode* root_;
	SearchStatistics statistics_;

	ScenarioLowerBound* lower_bound_;
	ScenarioUpperBound* upper_bound_;

public:
	DESPOT(const DSPOMDP* model, ScenarioLowerBound* lb, ScenarioUpperBound* ub, Belief* belief = NULL);
	virtual ~DESPOT();

	ValuedAction Search();

	void belief(Belief* b);
	void BeliefUpdate(ACT_TYPE action, OBS_TYPE obs);

	ScenarioLowerBound* lower_bound() const;
	ScenarioUpperBound* upper_bound() const;

	static VNode* ConstructTree(std::vector<State*>& particles, RandomStreams& streams,
		ScenarioLowerBound* lower_bound, ScenarioUpperBound* upper_bound,
		const DSPOMDP* model, History& history, double timeout,
		SearchStatistics* statistics = NULL);


	void arrow() const;
		void showPOM() const;
		void updatePOM() const;
		int isPoor(int posi_criterion) const;
		int isSomebodyPoor() const;

		double computing_factor(double satisf_c1, double satisf_c2, double c1_poor_high, double c2_poor_high) const;
		double computing_factor_no_poors(double satisf_c1, double satisf_c2, double c1_poor_high, double c2_poor_high) const;
		double computing_factor_poors(double satisf_c1, double satisf_c2, double c1_poor_high, double c2_poor_high) const;
		void compute_criteria_weights() const;
		void compute_rewards() const;


		// automatic juggling

		double computing_factor2(double weight_mec, double weight_mr) const; // 10/11/2017
		void final_RDM_Weights(double weight_mec, double weight_mr) const;// 22/10/2017
		double* comparisonMEC_MR(double factor,double kappa) const;// 22/10/2017



protected:
	static VNode* Trial(VNode* root, RandomStreams& streams,
		ScenarioLowerBound* lower_bound, ScenarioUpperBound* upper_bound,
		const DSPOMDP* model, History& history, SearchStatistics* statistics =
			NULL);
	static void InitLowerBound(VNode* vnode, ScenarioLowerBound* lower_bound,
		RandomStreams& streams, History& history);
	static void InitUpperBound(VNode* vnode, ScenarioUpperBound* upper_bound,
		RandomStreams& streams, History& history);
	static void InitBounds(VNode* vnode, ScenarioLowerBound* lower_bound,
		ScenarioUpperBound* upper_bound, RandomStreams& streams, History& history);

	static void Expand(VNode* vnode,
		ScenarioLowerBound* lower_bound, ScenarioUpperBound* upper_bound,
		const DSPOMDP* model, RandomStreams& streams, History& history);
	static void Backup(VNode* vnode);

	static double Gap(VNode* vnode);

	double CheckDESPOT(const VNode* vnode, double regularized_value);
	double CheckDESPOTSTAR(const VNode* vnode, double regularized_value);
	void Compare();

	static void ExploitBlockers(VNode* vnode);
	static VNode* FindBlocker(VNode* vnode);
	static void Expand(QNode* qnode, ScenarioLowerBound* lower_bound,
		ScenarioUpperBound* upper_bound, const DSPOMDP* model,
		RandomStreams& streams, History& history);
	static void Update(VNode* vnode);
	static void Update(QNode* qnode);
	static VNode* Prune(VNode* vnode, ACT_TYPE& pruned_action, double& pruned_value);
	static QNode* Prune(QNode* qnode, double& pruned_value);
	static double WEU(VNode* vnode);
	static double WEU(VNode* vnode, double epsilon);
	static VNode* SelectBestWEUNode(QNode* qnode);
	static QNode* SelectBestUpperBoundNode(VNode* vnode);
	static ValuedAction OptimalAction(VNode* vnode);

	static ValuedAction Evaluate(VNode* root, std::vector<State*>& particles,
		RandomStreams& streams, POMCPPrior* prior, const DSPOMDP* model);
};

} // namespace despot

#endif
