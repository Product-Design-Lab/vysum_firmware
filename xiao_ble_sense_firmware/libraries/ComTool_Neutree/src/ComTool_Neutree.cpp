#include "ComTool_Neutree.h"


#include <cassert>

void ComToolPack(uint8_t *buff, int buff_len,
                               const uint8_t *header, int header_len,
                               const char *name,
                               double x, double y)
{
    uint8_t len = (uint8_t)strlen(name);
    int actual_len = header_len + 1 + len + 8 + 8 + 1;
    assert(actual_len <= buff_len);

    memcpy(buff, header, header_len);
    buff[header_len] = len;
    memcpy(buff + 5, name, len);
    memcpy(buff + 5 + len, &x, 8);
    memcpy(buff + 5 + len + 8, &y, 8);
    int sum = 0;
    for (int i = 0; i < header_len + 1 + len + 8 + 8; i++)
    {
        sum += buff[i];
    }
    buff[header_len + 1 + len + 8 + 8] = (uint8_t)(sum & 0xff);
}