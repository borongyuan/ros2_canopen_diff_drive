#ifndef DIFF_DRIVE__ROLLING_MEAN_ACCUMULATOR_HPP_
#define DIFF_DRIVE__ROLLING_MEAN_ACCUMULATOR_HPP_

#include <cassert>
#include <vector>

namespace diff_drive
{
template <typename T>
class RollingMeanAccumulator
{
public:
  explicit RollingMeanAccumulator(size_t rolling_window_size)
    : buffer_(rolling_window_size, 0.0), next_insert_(0), sum_(0.0), buffer_filled_(false)
  {
  }

  void accumulate(T val)
  {
    sum_ -= buffer_[next_insert_];
    sum_ += val;
    buffer_[next_insert_] = val;
    next_insert_++;
    buffer_filled_ |= next_insert_ >= buffer_.size();
    next_insert_ = next_insert_ % buffer_.size();
  }

  T getRollingMean() const
  {
    size_t valid_data_count = buffer_filled_ * buffer_.size() + !buffer_filled_ * next_insert_;
    assert(valid_data_count > 0);
    return sum_ / valid_data_count;
  }

private:
  std::vector<T> buffer_;
  size_t next_insert_;
  T sum_;
  bool buffer_filled_;
};
}  // namespace diff_drive
#endif  // DIFF_DRIVE__ROLLING_MEAN_ACCUMULATOR_HPP_
