#include <iostream>
#include <memory>
#include <vector>
std::vector<std::shared_ptr<int>> test_ptr_returns()
// std::shared_ptr<int> test_ptr_returns()

{
  std::vector<std::shared_ptr<int>> result;
  std::shared_ptr<int> intptr1 = std::make_unique<int>(10);
//   *intptr1 = 5;
  result.push_back(intptr1);

  std::shared_ptr<int> intptr2= std::make_unique<int>(10);
// //   *intptr2 = 10;
  result.push_back(intptr2);
  return result;
// return intptr1;
}
int main()
{
  std::vector<std::shared_ptr<int>> ptr_array = test_ptr_returns();
//   //   std::unique_ptr<int> ptr;
//   for (int i = 0; i<ptr_array.size(); ++i)
//   {
//     std::cout << *(ptr_array[i]) << ", " << std::endl;
//   }
// std::unique_ptr<int> ptr = std::make_unique<int>(5);
// std::unique_ptr<int> ptr = test_ptr_returns();

// std::cout<<*ptr;

std::cout<<"Hello World!";
}