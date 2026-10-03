#include "../User/Module/Gimbal/GimbalHightStallDetector.hpp"
#include <cassert>

int main()
{
    GimbalHightStallDetector detector;
    assert(!detector.Update(true, 3.0f, 0.0f, 800.0f, 100));
    assert(!detector.Update(true, 3.0f, 0.0f, 800.0f, 599));
    assert(detector.Update(true, 3.0f, 0.0f, 800.0f, 600));
    // 停止或恢复运动均不能自动解除故障。
    assert(detector.Update(false, 0.0f, 0.0f, 0.0f, 601));
    assert(detector.Update(true, 3.0f, 3.0f, 800.0f, 602));
    detector.Reset();
    assert(!detector.HasFault());
    assert(!detector.Update(true, 3.0f, 0.0f, 800.0f, 0));
    assert(!detector.Update(true, 3.0f, 0.2f, 800.0f, 400));
    assert(!detector.Update(true, 3.0f, 0.0f, 800.0f, 500));
    assert(!detector.Update(true, 3.0f, 0.0f, 800.0f, 999));
    // 换向、切断输出、停止和低速目标中断计时。
    assert(!detector.Update(true, -3.0f, 0.0f, -800.0f, 1000));
    assert(!detector.Update(true, -3.0f, 0.0f, -800.0f, 1499));
    assert(!detector.Update(true, -3.0f, 0.0f, 0.0f, 1500));
    assert(!detector.Update(true, -3.0f, 0.0f, -800.0f, 1600));
    assert(!detector.Update(false, -3.0f, 0.0f, -800.0f, 2100));
    assert(!detector.Update(true, 0.2f, 0.0f, 800.0f, 3000));
    assert(!detector.Update(true, 0.2f, 0.0f, 800.0f, 4000));
    detector.Reset();
    assert(!detector.Update(true, -1.5f, 0.0f, -800.0f, UINT32_MAX - 200));
    assert(!detector.Update(true, -1.5f, 0.0f, -800.0f, 298));
    assert(detector.Update(true, -1.5f, 0.0f, -800.0f, 299));
}
