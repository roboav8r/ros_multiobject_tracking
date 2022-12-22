#ifndef GAUSSIAN_DATATYPES_H_
#define GAUSSIAN_DATATYPES_H_

#include <Eigen/Dense>
#include <math.h>

/*
Namespace for Gaussian model classes and data types
*/
namespace GaussianDataTypes {

    /* 
    GAUSSIAN MODEL
    Primary components are a mean, covariance, and weight.
    Also includes operators and helper methods.
    */
    
    template <size_t D>
    struct GaussianModel
    {
        GaussianModel()
        {
            clear();
            //m_isFalseTarget = false;
        }

        GaussianModel &operator=(const GaussianModel &rhs)
        {
            if (this != &rhs)
            {
                Mean = rhs.Mean;
                Cov = rhs.Cov;
                Weight = rhs.Weight;
                //m_isFalseTarget = rhs.m_isFalseTarget;
            }

            return *this;
        }

        GaussianModel &operator==(const GaussianModel &rhs)
        {
            if (this->Weight == &rhs->Weight &&  
                this->Mean == &rhs->Mean && 
                this->Cov == &rhs->Cov) 
            { return true; }
            else {return false;}
        }

        void clear()
        {
            Mean.setZero();
            Cov.setIdentity();
            Weight = 0.f;
        }

        float Weight;
        Eigen::Matrix<float, D, 1> Mean;
        Eigen::Matrix<float, D, D> Cov;
        //bool m_isFalseTarget;
    }; // GaussianModel


    /* 
    GAUSSIAN MIXTURE
    Primary component is a vector of Gaussian Models.
    Also includes operators and helper methods.
    */

    template <size_t D>
    class GaussianMixture
    {
    public:
        // Constructors
        GaussianMixture()
        {
            Gaussians.clear();
        }
        GaussianMixture(GaussianMixture const &source)
        {
            Gaussians = source.Gaussians;
        }

        GaussianMixture(std::vector<GaussianModel<D>> const &source)
        {
            Gaussians = source;
        }


        // Operator overloading
        GaussianMixture operator=(const GaussianMixture &source)
        {
            // Skip assignment if same object
            if (this == &source)
                return *this;

            // Else, use vectors & Eigen "=" operator
            Gaussians = source.Gaussians;
            return *this;
        }


        // Member variables
        std::vector<GaussianModel<D>> Gaussians;


        // Member functions
        void sort()
        {
            std::sort(Gaussians.begin(), Gaussians.end(), [](const auto &lhs, const auto &rhs) {
                return lhs.Weight > rhs.Weight;
            });
        }

        int selectBestGaussian()
        {
            float best_weight = 0.f;
            int best_index = -1;
            int i = 0;

            std::for_each(Gaussians.begin(), Gaussians.end(), [&](GaussianModel<D> const &gaussian) {
                if (gaussian.Weight > best_weight)
                {
                    best_weight = gaussian.Weight;
                    best_index = i;
                }
                ++i;
            });

            return best_index;
        } // selectBestGaussian

        void selectCloseGaussians(int i_ref, float threshold, std::vector<int> &close_gaussians)
        {
            close_gaussians.clear();

            float gauss_distance;

            Eigen::Matrix<float, D, 1> diff_vec;
            Eigen::Matrix<float, D, D> cov_inverse;

            // We only take positions into account there
            int i = 0;
            for (auto const &gaussian : Gaussians)
            {
                if (i != i_ref)
                {
                    // Compute distance
                    diff_vec = Gaussians[i_ref].Mean.head(D) -
                               gaussian.Mean.head(D);

                    cov_inverse = (Gaussians[i_ref].Cov.topLeftCorner(D, D)).inverse();

                    gauss_distance = diff_vec.transpose() *
                                     cov_inverse.topLeftCorner(D, D) *
                                     diff_vec;

                    // Add to the set of close gaussians, if below threshold
                    if ((gauss_distance < threshold) && (gaussian.Weight != 0.f))
                    {
                        close_gaussians.push_back(i);
                    }
                }
                ++i;
            }
        } // selectCloseGaussians

        GaussianModel<D> mergeGaussians(std::vector<int> &i_gaussians_to_merge, bool b_remove_from_mixture)
        {

            GaussianModel<D> merged_model;
            Eigen::Matrix<float, D, 1> diff;

            if (i_gaussians_to_merge.size() > 1)
            {
                // Reset the destination
                merged_model.clear();

                // Build merged gaussian :
                // - weight is the sum of all weights
                for (auto const &i_g : i_gaussians_to_merge)
                {
                    merged_model.Weight += Gaussians[i_g].Weight;
                }

                // - gaussian center is the weighted m_mean of all centers
                for (auto const &i_g : i_gaussians_to_merge)
                {
                    merged_model.Mean += Gaussians[i_g].Mean * Gaussians[i_g].Weight;
                }

                if (merged_model.Weight != 0.f)
                {
                    merged_model.Mean /= merged_model.Weight;
                }

                // - covariance is related to initial gaussian model cov and the discrepancy
                // from merged m_mean position and every merged gaussian pose
                merged_model.Cov.setZero();
                for (auto const &i_g : i_gaussians_to_merge)
                {
                    diff = merged_model.Mean - Gaussians[i_g].Mean;

                    merged_model.Cov += Gaussians[i_g].Weight * (Gaussians[i_g].Cov + diff * diff.transpose());
                }

                if (merged_model.Weight != 0.f)
                {
                    merged_model.Cov /= merged_model.Weight;
                }
            }
            else
            {
                // Just return the initial single gaussian model :
                merged_model = Gaussians[i_gaussians_to_merge[0]];
            }

            if (b_remove_from_mixture)
            {
                // Remove input gaussians from the mixture
                // - sort the index vector
                std::sort(i_gaussians_to_merge.begin(),
                          i_gaussians_to_merge.end());

                // - pop out the corresponding gaussians, in reverse
                auto it = Gaussians.begin();

                for (int i = i_gaussians_to_merge.size() - 1; i > -1; ++i)
                {
                    Gaussians.erase(it + i);
                }
            }

            return merged_model;
        } // mergeGaussians


        void prune(float& trunc_threshold, float& merge_threshold, uint& max_gaussians)
        {
            // Sort the gaussians mixture, ascending order
            sort();

            int index, i_best;

            std::vector<int> i_close_to_best;
            GaussianMixture<D> pruned_targets;
            GaussianModel<D> merged_gaussian;

            merged_gaussian.clear();
            pruned_targets.Gaussians.clear();

            while (!Gaussians.empty() && pruned_targets.Gaussians.size() < max_gaussians)
            {
                // - Pick the biggest gaussian (based on weight)
                i_best = selectBestGaussian();

                if (i_best == -1 || Gaussians[i_best].Weight < trunc_threshold)
                {
                    break;
                }
                else
                {
                    // - Select all the gaussians close enough, to merge if needed
                    i_close_to_best.clear();
                    selectCloseGaussians(i_best, merge_threshold, i_close_to_best);

                    // - Build a new merged gaussian
                    i_close_to_best.push_back(i_best); // Add the initial gaussian

                    if (i_close_to_best.size() > 1)
                    {
                        merged_gaussian = mergeGaussians(i_close_to_best, false);
                    }
                    else
                    {
                        merged_gaussian = Gaussians[i_close_to_best[0]];
                    }

                    // - Append merged gaussian to the pruned_targets gaussian mixture
                    pruned_targets.Gaussians.push_back(merged_gaussian);

                    // - Remove all the merged gaussians from current_targets :
                    // -- Sort the indexes
                    std::sort(i_close_to_best.begin(), i_close_to_best.end());

                    // -- Remove from the last one (to keep previous indexes unchanged)
                    while (!i_close_to_best.empty())
                    {
                        index = i_close_to_best.back();
                        i_close_to_best.pop_back();

                        Gaussians.erase(Gaussians.begin() + index);
                    }
                }
            }

            Gaussians = pruned_targets.Gaussians;
        } // prune

    }; // Gaussian Mixture Class

    /*
    PROBABILITY DENSITY
    Helper function used by the measurement update to evaluate a point's probability within a gaussian distribution
    */
    template <size_t N>
    float MultiVarGaussPdf(const Eigen::Matrix<float, N, 1>& point, const Eigen::Matrix<float, N, 1>& mean, const Eigen::Matrix<float, N, N>& cov)
    {
        return (1/sqrt((2*M_PI*cov).determinant()))*exp((point-mean).transpose()*cov.inverse()*(point-mean));
        // IMPROVEMENT: use cholesky factorization (https://stackoverflow.com/questions/27385477/how-to-efficiently-use-inverse-and-determinant-in-eigen)
    }

}

#endif // GAUSSIAN_DATATYPES_H_