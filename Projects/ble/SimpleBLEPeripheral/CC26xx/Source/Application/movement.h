#ifndef MOVEMENT_H
#define MOVEMENT_H

#ifdef __cplusplus
extern "C"
{
#endif

/* -----------------------------------------------------------------------------
*                                          Constants
* ------------------------------------------------------------------------------
*/
#define MOVEMENT_DATA_LEN              18

/*
 * Task creation function for the led blinking.
 */
extern void Movement_createTask(void);
//extern void get_movementData(float *mangX, float *mangY);
extern void get_movementData(uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif
