#include <iostream>
#include <vector>
#include <string>
#include <fstream>

// #include <bits/stdc++.h>
#include <string>
#include <sys/stat.h>
#include <ctime>
// #include <sys/types.h>

// using namespace std;

int main()
{
    // current date/time based on current system
   time_t now = time(0);

   std::cout << "Number of sec since January 1,1970 is:: " << now << std::endl;

   tm *ltm = localtime(&now);

   std::string a = std::to_string(ltm->tm_year) + "hi";

   // print various components of tm structure.
   std::cout << "Year:" << 1900 + ltm->tm_year<<std::endl;
   std::cout << "Month: "<< 1 + ltm->tm_mon<< std::endl;
   std::cout << "Day: "<< ltm->tm_mday << std::endl;
   std::cout << "Time: "<< ltm->tm_hour << ":";
   std::cout << ltm->tm_min << ":";
   std::cout << ltm->tm_sec << std::endl;


    // std::string dir_name = "delete_me";
    // mkdir(dir_name.c_str(), 0777);

    // std::ofstream file(dir_name + "/file1.txt");
    // file << "hello worudo";
    // file.close();


    // int a[5];
    // unsigned char *cp = reinterpret_cast<unsigned char *>(a);
    // float *fp = reinterpret_cast<float*>(a);


    // cout << a << endl;
    // cout << (void *)cp << endl;
    // cout << fp << endl;
    // cout << "end" << endl;
}