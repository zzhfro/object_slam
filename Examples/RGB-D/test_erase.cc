#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include <set>
int main()
{
 double a=1.5;
 double* p1=&a;
 double a2=2.5;
 double* p2=&a2;
 double a3=3.5;
 double* p3=&a3;
 double a4=4.5;
 double* p4=&a4;

 std::vector<double*> v;
 v.push_back(p1);
 v.push_back(p2);
 v.push_back(p3);
 v.push_back(p4);


 auto it = std::find(v.begin(), v.end(), p1);
    if (it != v.end()) {
        v.erase(it);
    }

for (double* ptr : v) {
        std::cout << *ptr << std::endl;
    }
std::cout<<*p1<<std::endl;

}