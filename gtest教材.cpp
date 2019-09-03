

1. 安装Google test

	git clone https://github.com/google/googletest.git
cd googletest
mkdir build
cmake ..
make
sudo make install
以上命令会将gtest编译好，并将动态链接库放在/usr/local/lib 目录下：

huangyang@ubuntu:/usr/local/lib$ ls
libgmock.a       libgtest.a       pkgconfig  python3.5
libgmock_main.a  libgtest_main.a  python2.7
如上所示，四个.a文件是gtest的动态链接库，在编写gtest的时候只需要链接这些文件就可以使用gtest。

将头文件放在/usr/local/include目录下：

huangyang@huangyang-desktop:~/Notebook$ cd /usr/local/include/
huangyang@huangyang-desktop:/usr/local/include$ ls
gmock  gtest




2.使用gtest
add程序

add.cc

#include <iostream>

int add(int a, int b)
{
    return a + b;
}

//int main()
//{
//    std::cout << add(3, 10);
//}
为add程序编写单元测试用例

testAdd.cc

#include <gtest/gtest.h>

extern int add(int a, int b);

TEST(testCase, test0)
{
    EXPECT_EQ(add(2, 3), 5);
}

int main(int argc, char **argv)
{
 testing::InitGoogleTest(&argc, argv);
 return RUN_ALL_TESTS();
}

编译和链接程序
 g++ add.cc testAdd.cc -lgtest -lpthread -std=c++11

















// 断言方法
	断言方法的丰富程度，决定了写测试代码时，需不需要做很多hack。
	这里直接把各种方法贴出来：

	基本断言

	出错会停止测试的断言	出错不会停止测试的断言	意思
	ASSERT_EQ(expected, actual)	EXPECT_EQ(expected, actual)	val1 == val2
	ASSERT_NE(val1, val2)	EXPECT_NE(val1, val2)	val1 != val2
	ASSERT_LT(val1, val2)	EXPECT_LT(val1, val2)	val1 < val2
	ASSERT_LE(val1, val2)	EXPECT_LE(val1, val2)	val1 <= val2
	ASSERT_GT(val1, val2)	EXPECT_GT(val1, val2)	val1 > val2
	ASSERT_GE(val1, val2)	EXPECT_GE(val1, val2)	val1 >= val2
	ASSERT_TRUE(condition)	EXPECT_TRUE(condition)	condition为true
	ASSERT_FALSE(condition)	EXPECT_FALSE(condition)	condition为false
	适用于C Style字符串的断言

	出错会停止测试的断言	出错不会停止测试的断言	意思
	ASSERT_STREQ(expected_str, actual_str)	EXPECT_STREQ(expected_str, actual_str)	两个字符串相等
	ASSERT_STRNE(str1, str2)	EXPECT_STRNE(str1, str2)	两个字符串不等
	ASSERT_STRCASEEQ(expected_str,actual_str)	EXPECT_STRCASEEQ(expected_str,actual_str)	两个字符串相等，忽略大小写
	ASSERT_STRCASENE(str1, str2)	EXPECT_STRCASENE(str1, str2)	两个字符串不等，忽略大小写
	适用于浮点数比较的断言

	出错会停止测试的断言	出错不会停止测试的断言	意思
	ASSERT_FLOAT_EQ(expected, actual)	EXPECT_FLOAT_EQ(expected, actual)	float类型近似相等
	ASSERT_DOUBLE_EQ(expected, actual)	EXPECT_DOUBLE_EQ(expected, actual)	double类型近似相等
	近似相等的断言

	出错会停止测试的断言	出错不会停止测试的断言	意思
	ASSERT_NEAR(val1, val2, abs_error)	EXPECT_NEAR(val1, val2, abs_error)	val1和val2的差的绝对值小于abs_error
	是否抛出异常的断言

	出错会停止测试的断言	出错不会停止测试的断言	意思
	ASSERT_THROW(statement, exception_type)	EXPECT_THROW(statement, exception_type)	代码块会抛出指定异常
	ASSERT_ANY_THROW(statement)	EXPECT_ANY_THROW(statement)	代码块会抛出异常
	ASSERT_NO_THROW(statement)	EXPECT_NO_THROW(statement)	代码块不会抛出异常
