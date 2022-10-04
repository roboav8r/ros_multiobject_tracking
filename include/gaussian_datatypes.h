#ifndef GAUSSIAN_DATATYPES_H_
#define GAUSSIAN_DATATYPES_H_

#include <Eigen/Dense>

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

        void clear()
        {
            Mean.setZero();
            Cov.setIdentity();
            Weight = 0.f;
        }

        float Weight;
        Eigen::Matrix<double, D, 1> Mean;
        Eigen::Matrix<double, D, D> Cov;
        //bool m_isFalseTarget;
    };



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
                return lhs.m_weight > rhs.m_weight;
            });
        }

    };

}

        // GaussianModel<D> mergeGaussians(vector<int> &i_gaussians_to_merge, bool b_remove_from_mixture)
        // {
        //     // TODO: Ben - rewrite this crap, could be half way long

        //     GaussianModel<D> merged_model;
        //     Matrix<float, D, 1> diff;

        //     if (i_gaussians_to_merge.size() > 1)
        //     {
        //         // Reset the destination
        //         merged_model.clear();

        //         // Build merged gaussian :
        //         // - weight is the sum of all weights
        //         for (auto const &i_g : i_gaussians_to_merge)
        //         {
        //             merged_model.m_weight += m_gaussians[i_g].m_weight;
        //         }

        //         // - gaussian center is the weighted m_mean of all centers
        //         for (auto const &i_g : i_gaussians_to_merge)
        //         {
        //             merged_model.m_mean += m_gaussians[i_g].m_mean * m_gaussians[i_g].m_weight;
        //         }

        //         if (merged_model.m_weight != 0.f)
        //         {
        //             merged_model.m_mean /= merged_model.m_weight;
        //         }

        //         // - covariance is related to initial gaussian model cov and the discrepancy
        //         // from merged m_mean position and every merged gaussian pose
        //         merged_model.m_cov.setZero();
        //         for (auto const &i_g : i_gaussians_to_merge)
        //         {
        //             diff = merged_model.m_mean - m_gaussians[i_g].m_mean;

        //             merged_model.m_cov += m_gaussians[i_g].m_weight * (m_gaussians[i_g].m_cov + diff * diff.transpose());
        //         }

        //         if (merged_model.m_weight != 0.f)
        //         {
        //             merged_model.m_cov /= merged_model.m_weight;
        //         }
        //     }
        //     else
        //     {
        //         // Just return the initial single gaussian model :
        //         merged_model = m_gaussians[i_gaussians_to_merge[0]];
        //     }

        //     if (b_remove_from_mixture)
        //     {
        //         // Remove input gaussians from the mixture
        //         // - sort the index vector
        //         std::sort(i_gaussians_to_merge.begin(),
        //                   i_gaussians_to_merge.end());

        //         // - pop out the corresponding gaussians, in reverse
        //         auto it = m_gaussians.begin();

        //         for (int i = i_gaussians_to_merge.size() - 1; i > -1; ++i)
        //         {
        //             m_gaussians.erase(it + i);
        //         }
        //     }

        //     return merged_model;
        // }

        // void normalize(float linear_offset)
        // {
        //     const float sum = std::accumulate(m_gaussians.begin(), m_gaussians.end(), 0.f, [](const GaussianModel<D> &g1, const GaussianModel<D> &g2) { return g1.m_weight + g2.m_weight; });

        //     if ((linear_offset + sum) != 0.f)
        //     {
        //         for (auto &gaussian : m_gaussians)
        //         {
        //             gaussian.m_weight /= (linear_offset + sum);
        //         }
        //     }
        // }

        // void normalize(float linear_offset, int start_pos, int stop_pos, int step)
        // {
        //     float sum = 0.f;
        //     for (int i = start_pos; i < stop_pos; ++i)
        //     {
        //         sum += m_gaussians[i * step].m_weight;
        //     }

        //     if ((linear_offset + sum) != 0.f)
        //     {
        //         for (int i = start_pos; i < stop_pos; ++i)
        //         {
        //             m_gaussians[i * step].m_weight /= (linear_offset + sum);
        //         }
        //     }
        // }

        // void prune(float trunc_threshold, float merge_threshold, uint max_gaussians)
        // {
        //     // Sort the gaussians mixture, ascending order
        //     sort();

        //     int index, i_best;

        //     vector<int> i_close_to_best;
        //     GaussianMixture<D> pruned_targets;
        //     GaussianModel<D> merged_gaussian;

        //     merged_gaussian.clear();
        //     pruned_targets.m_gaussians.clear();

        //     while (!m_gaussians.empty() && pruned_targets.m_gaussians.size() < max_gaussians)
        //     {
        //         // - Pick the biggest gaussian (based on weight)
        //         i_best = selectBestGaussian();

        //         if (i_best == -1 || m_gaussians[i_best].m_weight < trunc_threshold)
        //         {
        //             break;
        //         }
        //         else
        //         {
        //             // - Select all the gaussians close enough, to merge if needed
        //             i_close_to_best.clear();
        //             selectCloseGaussians(i_best, merge_threshold, i_close_to_best);

        //             // - Build a new merged gaussian
        //             i_close_to_best.push_back(i_best); // Add the initial gaussian

        //             if (i_close_to_best.size() > 1)
        //             {
        //                 merged_gaussian = mergeGaussians(i_close_to_best, false);
        //             }
        //             else
        //             {
        //                 merged_gaussian = m_gaussians[i_close_to_best[0]];
        //             }

        //             // - Append merged gaussian to the pruned_targets gaussian mixture
        //             pruned_targets.m_gaussians.push_back(merged_gaussian);

        //             // - Remove all the merged gaussians from current_targets :
        //             // -- Sort the indexes
        //             std::sort(i_close_to_best.begin(), i_close_to_best.end());

        //             // -- Remove from the last one (to keep previous indexes unchanged)
        //             while (!i_close_to_best.empty())
        //             {
        //                 index = i_close_to_best.back();
        //                 i_close_to_best.pop_back();

        //                 m_gaussians.erase(m_gaussians.begin() + index);
        //             }
        //         }
        //     }

        //     m_gaussians = pruned_targets.m_gaussians;
        // }


        // void selectCloseGaussians(int i_ref, float threshold, vector<int> &close_gaussians)
        // {
        //     close_gaussians.clear();

        //     float gauss_distance;

        //     Matrix<float, D, 1> diff_vec;
        //     Matrix<float, D, D> cov_inverse;

        //     // We only take positions into account there
        //     int i = 0;
        //     for (auto const &gaussian : m_gaussians)
        //     {
        //         if (i != i_ref)
        //         {
        //             // Compute distance
        //             diff_vec = m_gaussians[i_ref].m_mean.head(D) -
        //                        gaussian.m_mean.head(D);

        //             cov_inverse = (m_gaussians[i_ref].m_cov.topLeftCorner(D, D)).inverse();

        //             gauss_distance = diff_vec.transpose() *
        //                              cov_inverse.topLeftCorner(D, D) *
        //                              diff_vec;

        //             // Add to the set of close gaussians, if below threshold
        //             if ((gauss_distance < threshold) && (gaussian.m_weight != 0.f))
        //             {
        //                 close_gaussians.push_back(i);
        //             }
        //         }
        //         ++i;
        //     }
        // }

        // int selectBestGaussian()
        // {
        //     float best_weight = 0.f;
        //     int best_index = -1;
        //     int i = 0;

        //     std::for_each(m_gaussians.begin(), m_gaussians.end(), [&](GaussianModel<D> const &gaussian) {
        //         if (gaussian.m_weight > best_weight)
        //         {
        //             best_weight = gaussian.m_weight;
        //             best_index = i;
        //         }
        //         ++i;
        //     });

        //     return best_index;
        // }


#endif // GAUSSIAN_DATATYPES_H_