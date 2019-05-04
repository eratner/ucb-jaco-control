#ifndef UCB_JACO_CONTROL_BUFFER_H
#define UCB_JACO_CONTROL_BUFFER_H

#include <vector>
#include <iostream>
#include <stdexcept>

namespace ucb_jaco_control
{

template <typename DataType>
class Buffer
{
public:
  Buffer(unsigned int size)
    : size_(size), count_(0), front_(nullptr), back_(nullptr)
  {
  }

  ~Buffer()
  {
    clear();
  }

  void add(const DataType& data)
  {
    if (count_ == size_)
    {
      // Buffer is full, so remove the oldest item.
      DataNode *to_delete = front_;
      front_->next->prev = nullptr;
      front_ = front_->next;

      delete to_delete;
      to_delete = nullptr;
      --count_;
    }

    // Add the new data to the back.
    DataNode *node = new DataNode(data, nullptr, back_);
    if (back_)
      back_->next = node;
    else
    {
      // First node, need to set front pointer.
      front_ = node;
    }

    back_ = node;
    ++count_;
  }

  std::vector<DataType> getData() const
  {
    // Insert the data into a vector ordered with front at data[0].
    std::vector<DataType> data(count_);

    DataNode *node = front_;
    for (unsigned int i = 0; i < count_; ++i)
    {
      data[i] = node->data;
      node = node->next;
    }

    return data;
  }

  const DataType& back() const
  {
    if (count_ == 0)
      throw std::out_of_range("Buffer is empty!");

    return back_->data;
  }

  const DataType& front() const
  {
    if (count_ == 0)
      throw std::out_of_range("Buffer is empty!");

    return front_->data;
  }

  int count() const
  {
    return count_;
  }

  void clear()
  {
    count_ = 0;
    DataNode* node = front_;
    while (node)
    {
      DataNode* next = node->next;
      delete node;
      node = next;
    }
  }

private:
  struct DataNode
  {
    DataNode(const DataType& d, DataNode* n = nullptr, DataNode* p = nullptr)
      : data(d), next(n), prev(p)
    {
    }

    DataType  data;

    DataNode* next;
    DataNode* prev;
  };

  DataNode*    front_;
  DataNode*    back_;

  unsigned int size_;   // Maximum size of the buffer.
  unsigned int count_;  // Current size of the buffer. 

};

} // namespace ucb_jaco_control

#endif // UCB_JACO_CONTROL_BUFFER_H
