#include <Arduino.h>

union vcell
{
    float f;
    uint8_t b[4];
};

const uint8_t n_cells = 4;

class Battery
{
public:
    Battery();
    void buffer2vcells(uint8_t buffer_array[]);
    void print_vcells();
    vcell vcell_1;
    vcell vcell_2;
    vcell vcell_3;
    vcell vcell_4;
    vcell *vcells[n_cells];
private:
};