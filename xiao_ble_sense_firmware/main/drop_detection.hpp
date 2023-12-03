#pragma once

namespace VCNL
{
    void init();
    void deinit();
    void startDetection();
    void stopDetection();
    int get_drop_count();
    void set_drop_count(const int count);
}