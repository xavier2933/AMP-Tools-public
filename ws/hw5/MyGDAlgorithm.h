#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW5.h"

class MyGDAlgorithm : public amp::GDAlgorithm {
	public:
		// Consider defining a class constructor to easily tune parameters, for example: 
		MyGDAlgorithm(double d_star, double zetta, double Q_star, double eta) :
			d_star(d_star),
			zetta(zetta),
			Q_star(Q_star),
			eta(eta) {}

		// Override this method to solve a given problem.
		virtual amp::Path2D plan(const amp::Problem2D& problem) override;
		Eigen::Vector2d GetNextStep(amp::Path2D& path, const amp::Problem2D& problem);
		Eigen::Vector2d GetUAtt(Eigen::Vector2d gradient, amp::Path2D& path, const amp::Problem2D& problem);
		Eigen::Vector2d GetURep(Eigen::Vector2d gradient, amp::Path2D& path, const amp::Problem2D& problem);
		bool is_point_inside_polygon(const amp::Problem2D& environment, const Eigen::Vector2d& point) const;

		amp::Path2D path_;
		amp::Problem2D problem_;


	private:
		double d_star, zetta, Q_star, eta;

		// Add additional member variables here...
};

class MyPotentialFunction : public amp::PotentialFunction2D {
    public:
		MyPotentialFunction(MyGDAlgorithm* algo) : algo_(algo) {}
		// Returns the potential function value (height) for a given 2D point. 
        virtual double operator()(const Eigen::Vector2d& q) const override {
            return q[0] * q[0] + q[1] * q[1];
        }

		virtual Eigen::Vector2d getGradient(const Eigen::Vector2d& q) const override {
            return algo_->GetUAtt(q,algo_->path_, algo_->problem_);
        }

	private: 
    MyGDAlgorithm* algo_;  // Pointer to MyGDAlgorithm
};