#pragma once

namespace DropDetection
{
    void init();
    void deinit();
    void start();
    void stop();
    int get_drop_count();
    void set_drop_count(const int count);
}