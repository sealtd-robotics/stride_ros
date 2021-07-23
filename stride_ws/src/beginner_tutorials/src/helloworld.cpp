#include <iostream>
#include <vector>
#include <string>

using namespace std;

int main()
{
    int a[5];
    unsigned char *cp = reinterpret_cast<unsigned char *>(a);
    float *fp = reinterpret_cast<float*>(a);


    cout << a << endl;
    cout << (void *)cp << endl;
    cout << fp << endl;
    cout << "end" << endl;
}