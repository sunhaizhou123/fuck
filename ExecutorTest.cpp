#include <gtest/gtest.h>
#include <memory>
//#include <tuple>
#include "Executor.hpp"
#include "PoseEq.hpp"

namespace adas
{
    // 重载Pose的==，用的全局函数，用于比较二个姿态对象是不是相对
    // bool operator==(const Pose &lhs, const Pose &rhs)
    // {
    //     return std::tie(lhs.x, lhs.y, lhs.heading) == std::tie(rhs.x, rhs.y, rhs.heading);
    // }
    // 下面二个测试用例，测试静态方法Executor::NewExecutor
    // 测试用例1
    TEST(ExecutorTest, should_return_init_pose_when_without_command)
    {
        // given 给定测试条件
        // 测试条件就是调用Executor的静态方法NewExecutor，返回一个指向Executor对象的智能指针executor，这样我们就不需要去delete了
        std::unique_ptr<Executor> executor(Executor::NewExecutor({0, 0, 'E'})); // 给了初始姿态

        // when

        // then
        const Pose target({0, 0, 'E'}); // 构造一个姿势对象target，其内容为{0, 0, 'E'}
        // 既然构造Executor对象时的初始姿势是{0, 0, 'E'}，那么executor->Query()返回的Executor对象的姿势必须等于target
        // 所以这里用了断言：executor->Query()返回的Executor对象的姿势必须等于target，否则测试失败，说明Executor的实现有问题
        ASSERT_EQ(target, executor->Query()); // ASSERT_EQ内部调用了重载的Pose的==
    }
    // 测试用例2
    TEST(ExecutorTest, should_return_default_pose_when_without_init_and_command)
    {
        // given
        std::unique_ptr<Executor> executor(Executor::NewExecutor()); // 不给初始姿态

        // when

        // then
        const Pose target({0, 0, 'N'});
        ASSERT_EQ(target, executor->Query());
    }
    // 测试用例3，测试Executor方法，在朝向为E，起点为(0,0)时去执行M指令是否正确
    TEST(ExecutorTest, should_return_x_plus_1_given_command_is_M_and_facing_is_E)
    {
        // given 给定一个executor
        std::unique_ptr<Executor> executor(Executor::NewExecutor({0, 0, 'E'})); // 给初始姿势

        // when 调用executor的Execute方法执行M指令
        executor->Execute("M");

        // then
        const Pose target({1, 0, 'E'});       // 正确结果应该是target
        ASSERT_EQ(target, executor->Query()); // 断言executor->Query()返回的Executor对象的姿势必须等于target
    }
    // 测试用例4，测试Executor方法，在朝向为W，起点为(0,0)时去执行M指令是否正确
    TEST(ExecutorTest, should_return_x_minus_1_given_command_is_M_and_facing_is_W)
    {
        // given 给定一个executor
        std::unique_ptr<Executor> executor(Executor::NewExecutor({0, 0, 'W'})); // 给初始姿势

        // when 调用executor的Execute方法执行M指令
        executor->Execute("M");

        // then
        const Pose target({-1, 0, 'W'});      // 正确结果应该是target
        ASSERT_EQ(target, executor->Query()); // 断言executor->Query()返回的Executor对象的姿势必须等于target
    }
    // 测试用例5，测试Executor方法，在朝向为N，起点为(0,0)时去执行M指令是否正确
    TEST(ExecutorTest, should_return_y_plus_1_given_command_is_M_and_facing_is_N)
    {
        // given 给定一个executor
        std::unique_ptr<Executor> executor(Executor::NewExecutor({0, 0, 'N'})); // 给初始姿势

        // when 调用executor的Execute方法执行M指令
        executor->Execute("M");

        // then
        const Pose target({0, 1, 'N'});       // 正确结果应该是target
        ASSERT_EQ(target, executor->Query()); // 断言executor->Query()返回的Executor对象的姿势必须等于target
    }
    // 测试用例6，测试Executor方法，在朝向为S，起点为(0,0)时去执行M指令是否正确
    TEST(ExecutorTest, should_return_y_minus_1_given_command_is_M_and_facing_is_S)
    {
        // given 给定一个executor
        std::unique_ptr<Executor> executor(Executor::NewExecutor({0, 0, 'S'})); // 给初始姿势

        // when 调用executor的Execute方法执行M指令
        executor->Execute("M");

        // then
        const Pose target({0, -1, 'S'});      // 正确结果应该是target
        ASSERT_EQ(target, executor->Query()); // 断言executor->Query()返回的Executor对象的姿势必须等于target
    }
    // 测试用例7，测试Executor方法，在朝向为E，起点为(0,0)时去执行L指令是否正确
    TEST(ExecutorTest, should_return_facing_N_given_command_is_L_and_facing_is_E)
    {
        // given 给定一个executor
        std::unique_ptr<Executor> executor(Executor::NewExecutor({0, 0, 'E'})); // 给初始姿势

        // when 调用executor的Execute方法执行L指令
        executor->Execute("L");

        // then
        const Pose target({0, 0, 'N'});       // 正确结果应该是target
        ASSERT_EQ(target, executor->Query()); // 断言executor->Query()返回的Executor对象的姿势必须等于target
    }
    // 测试用例8，测试Executor方法，在朝向为W，起点为(0,0)时去执行L指令是否正确
    TEST(ExecutorTest, should_return_facing_S_given_command_is_L_and_facing_is_W)
    {
        // given 给定一个executor
        std::unique_ptr<Executor> executor(Executor::NewExecutor({0, 0, 'W'})); // 给初始姿势

        // when 调用executor的Execute方法执行L指令
        executor->Execute("L");

        // then
        const Pose target({0, 0, 'S'});       // 正确结果应该是target
        ASSERT_EQ(target, executor->Query()); // 断言executor->Query()返回的Executor对象的姿势必须等于target
    }
    // 测试用例9，测试Executor方法，在朝向为N，起点为(0,0)时去执行L指令是否正确
    TEST(ExecutorTest, should_return_facing_W_given_command_is_L_and_facing_is_N)
    {
        // given 给定一个executor
        std::unique_ptr<Executor> executor(Executor::NewExecutor({0, 0, 'N'})); // 给初始姿势

        // when 调用executor的Execute方法执行L指令
        executor->Execute("L");

        // then
        const Pose target({0, 0, 'W'});       // 正确结果应该是target
        ASSERT_EQ(target, executor->Query()); // 断言executor->Query()返回的Executor对象的姿势必须等于target
    }
    // 测试用例10，测试Executor方法，在朝向为S，起点为(0,0)时去执行L指令是否正确
    TEST(ExecutorTest, should_return_facing_E_given_command_is_L_and_facing_is_S)
    {
        // given 给定一个executor
        std::unique_ptr<Executor> executor(Executor::NewExecutor({0, 0, 'S'})); // 给初始姿势

        // when 调用executor的Execute方法执行L指令
        executor->Execute("L");

        // then
        const Pose target({0, 0, 'E'});       // 正确结果应该是target
        ASSERT_EQ(target, executor->Query()); // 断言executor->Query()返回的Executor对象的姿势必须等于target
    }
    // 测试用例11，测试Executor方法，在朝向为E，起点为(0,0)时去执行R指令是否正确
    TEST(ExecutorTest, should_return_facing_S_given_command_is_R_and_facing_is_E)
    {
        // given 给定一个executor
        std::unique_ptr<Executor> executor(Executor::NewExecutor({0, 0, 'E'})); // 给初始姿势

        // when 调用executor的Execute方法执行L指令
        executor->Execute("R");

        // then
        const Pose target({0, 0, 'S'});       // 正确结果应该是target
        ASSERT_EQ(target, executor->Query()); // 断言executor->Query()返回的Executor对象的姿势必须等于target
    }
    // 测试用例12，测试Executor方法，在朝向为W，起点为(0,0)时去执行R指令是否正确
    TEST(ExecutorTest, should_return_facing_N_given_command_is_R_and_facing_is_W)
    {
        // given 给定一个executor
        std::unique_ptr<Executor> executor(Executor::NewExecutor({0, 0, 'W'})); // 给初始姿势

        // when 调用executor的Execute方法执行L指令
        executor->Execute("R");

        // then
        const Pose target({0, 0, 'N'});       // 正确结果应该是target
        ASSERT_EQ(target, executor->Query()); // 断言executor->Query()返回的Executor对象的姿势必须等于target
    }
    // 测试用例13，测试Executor方法，在朝向为N，起点为(0,0)时去执行R指令是否正确
    TEST(ExecutorTest, should_return_facing_E_given_command_is_R_and_facing_is_N)
    {
        // given 给定一个executor
        std::unique_ptr<Executor> executor(Executor::NewExecutor({0, 0, 'N'})); // 给初始姿势

        // when 调用executor的Execute方法执行L指令
        executor->Execute("R");

        // then
        const Pose target({0, 0, 'E'});       // 正确结果应该是target
        ASSERT_EQ(target, executor->Query()); // 断言executor->Query()返回的Executor对象的姿势必须等于target
    }
    // 测试用例14，测试Executor方法，在朝向为S，起点为(0,0)时去执行R指令是否正确
    TEST(ExecutorTest, should_return_facing_W_given_command_is_R_and_facing_is_S)
    {
        // given 给定一个executor
        std::unique_ptr<Executor> executor(Executor::NewExecutor({0, 0, 'S'})); // 给初始姿势

        // when 调用executor的Execute方法执行L指令
        executor->Execute("R");

        // then
        const Pose target({0, 0, 'W'});       // 正确结果应该是target
        ASSERT_EQ(target, executor->Query()); // 断言executor->Query()返回的Executor对象的姿势必须等于target
    }
}