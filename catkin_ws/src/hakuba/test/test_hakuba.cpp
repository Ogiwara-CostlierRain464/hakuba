#include <gtest/gtest.h>
#include <ecl/threads.hpp>
#include <functional>

using namespace std;

struct Unit: public ::testing::Test{};

TEST_F(Unit, hello){
    EXPECT_EQ(1+1, 2);
}

struct A{
    void a() const{
        cout << "w" << endl;
    }
};

struct B {
public:
    explicit B(const std::function<void(B&)> &code_)
            : code(code_),
              codeThread(ecl::Thread(ecl::generateFunctionObject(&B::onCodeThread, *this))) {}


    void onCodeThread(){
        code(*this);
    }

    const std::function<void(B&)> &code;
    ecl::Thread codeThread;
};

TEST_F(Unit, thread){
    A a;
    B b([&a](B&){
        a.a();
    });

    sleep(1);
}

int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}