#ifndef D1A942CD_8799_44BA_919D_944D3E28E376
#define D1A942CD_8799_44BA_919D_944D3E28E376

#include "lvgl.h"
#include "sdkconfig.h"
#include "img_bitmap.h"

#ifdef __cplusplus
extern "C" {
#endif

// Your code here
bool _lvgl_lock(int timeout_ms);
void _lvgl_unlock(void);

#ifdef __cplusplus
}
#endif

#endif /* D1A942CD_8799_44BA_919D_944D3E28E376 */
