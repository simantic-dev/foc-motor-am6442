#include <stdlib.h>
#include "ti_drivers_config.h"
#include "ti_board_config.h"

void foc_main(void);

int main(void)
{
    System_init();
    Board_init();
    foc_main();
    Board_deinit();
    System_deinit();
    return 0;
}
