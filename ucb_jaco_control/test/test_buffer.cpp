#include <ucb_jaco_control/buffer.h>
#include <gtest/gtest.h>
#include <iostream>
#include <stdexcept>

using ucb_jaco_control::Buffer;

template <typename T>
void print(const std::vector<T> &vec)
{
  for (int i = 0; i < vec.size(); ++i)
  {
    std::cout << vec[i];
    if (i < vec.size() - 1)
      std::cout << ", ";
  }
  std::cout << std::endl;
}

TEST(Buffer, testBuffer)
{
  Buffer<double> buff(3);

  ASSERT_EQ(buff.getData().size(), 0);
  ASSERT_THROW(buff.back(), std::out_of_range);
  ASSERT_THROW(buff.front(), std::out_of_range);

  buff.add(1);
  ASSERT_EQ(buff.count(), 1);
  ASSERT_EQ(buff.getData().size(), 1);
  ASSERT_EQ(buff.back(), 1);
  ASSERT_EQ(buff.front(), 1);

  buff.add(2);
  ASSERT_EQ(buff.count(), 2);
  ASSERT_EQ(buff.getData().size(), 2);
  ASSERT_EQ(buff.front(), 1);
  ASSERT_EQ(buff.back(), 2);

  buff.add(3);
  ASSERT_EQ(buff.count(), 3);
  ASSERT_EQ(buff.getData().size(), 3);
  ASSERT_EQ(buff.front(), 1);
  ASSERT_EQ(buff.back(), 3);

  buff.add(4);
  ASSERT_EQ(buff.count(), 3);
  ASSERT_EQ(buff.getData().size(), 3);
  ASSERT_EQ(buff.front(), 2);
  ASSERT_EQ(buff.back(), 4);

  std::vector<double> data = buff.getData();
  std::vector<double> expected_data = {2, 3, 4};
  for (int i = 0; i < data.size(); ++i)
    ASSERT_FLOAT_EQ(expected_data[i], data[i]);

  print(buff.getData());
}

int main(int argc, char *argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

