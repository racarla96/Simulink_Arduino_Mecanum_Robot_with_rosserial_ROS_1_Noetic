//WattmeterINA219Driver.h
#ifndef _WATTMETERINA219DRIVER_H_
#define _WATTMETERINA219DRIVER_H_
#ifdef __cplusplus
extern "C" {
#endif
    #if (defined(MATLAB_MEX_FILE) || defined(RSIM_PARAMETER_LOADING) ||  defined(RSIM_WITH_SL_SOLVER))
        /* This will be run in Rapid Accelerator Mode */
        #define wINA219Driver_Init(a, b)    (0)
        #define wINA219Driver_Step(a, b, c) (0)
        #define wINA219Driver_Terminate()   (0)
    #else
        #include <stdint.h>

        void wINA219Driver_Init(int8_t, int8_t);
        //void wINA219Driver_Init(int8_t, int8_t, int8_t);
        void wINA219Driver_Step(int8_t, int32_t*, int32_t*);
        void wINA219Driver_Terminate(void);
#endif
#ifdef __cplusplus
}
#endif
#endif 