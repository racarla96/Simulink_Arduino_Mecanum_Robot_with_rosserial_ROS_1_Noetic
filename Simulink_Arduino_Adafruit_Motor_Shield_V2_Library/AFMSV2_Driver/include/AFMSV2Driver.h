//AFMSV2Driver.h
#ifndef _AFMSV2DRIVER_H_
#define _AFMSV2DRIVER_H_
#ifdef __cplusplus
extern "C" {
#endif
    #if (defined(MATLAB_MEX_FILE) || defined(RSIM_PARAMETER_LOADING) ||  defined(RSIM_WITH_SL_SOLVER))
        /* This will be run in Rapid Accelerator Mode */
        #define AFMSV2Driver_Init()         (0)
        #define AFMSV2Driver_Step(a,b,c,d)    (0)
        #define AFMSV2Driver_Terminate()    (0)
    #else
        void AFMSV2Driver_Init(void);
        void AFMSV2Driver_Step(int16_t,int16_t,int16_t,int16_t);
        void AFMSV2Driver_Terminate(void);
#endif
#ifdef __cplusplus
}
#endif
#endif 