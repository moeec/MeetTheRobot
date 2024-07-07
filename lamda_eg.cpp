#include <iostream>
#include <vector>
#include <algorithm>

/* In this example:

A lambda print is defined to print a number.
The std::for_each function is used to apply this lambda to each element in the numbers vector.
Another lambda is used inline within std::for_each to increment each element in the numbers vector by 1.
Lambdas can be particularly powerful when used with algorithms in the Standard Library, as they allow you 
to define custom operations directly in place, leading to more concise and readable code.

The std::for_each function in C++ is an algorithm provided by the Standard Library that applies a given 
function to a range of elements in a container. It is defined in the <algorithm> header and is used to 
iterate over a range, applying the function to each element.

template<class InputIt, class UnaryFunction>
UnaryFunction for_each(InputIt first, InputIt last, UnaryFunction f);

InputIt first: The iterator to the first element in the range.
InputIt last: The iterator to one past the last element in the range.
UnaryFunction f: The function or function object to apply to each element in the range.
std::for_each applies the function f to each element in the range [first, last). The function f can 
be a function pointer, a function object, or a lambda expression.

*/

int main() {
    std::vector<int> numbers = {1, 2, 3, 4, 5};

    // Print each number in the vector
    std::for_each(numbers.begin(), numbers.end(), [](int n) {
        std::cout << n << ' ';
    });
    std::cout << std::endl;

    // Add 1 to each element in the vector
    std::for_each(numbers.begin(), numbers.end(), [](int &n) {
        n += 1;
    });

    // Print each number in the vector again to see the changes
    std::for_each(numbers.begin(), numbers.end(), [](int n) {
        std::cout << n << ' ';
    });
    std::cout << std::endl;

    return 0;
}
