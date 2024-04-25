//Robot_Mecanum_rosserial_Driver.h
#ifndef _Robot_Mecanum_rosserial_Driver_H_
#define _Robot_Mecanum_rosserial_Driver_H_
#ifdef __cplusplus
extern "C" {
#endif
    #if (defined(MATLAB_MEX_FILE) || defined(RSIM_PARAMETER_LOADING) ||  defined(RSIM_WITH_SL_SOLVER))
        /* This will be run in Rapid Accelerator Mode */
        #define Robot_Mecanum_rosserial_Driver_Init()           (0)
        #define Robot_Mecanum_rosserial_Driver_Step()           (0)
        #define Robot_Mecanum_rosserial_Driver_Terminate()      (0)
    #else
        void Robot_Mecanum_rosserial_Driver_Init();
        void Robot_Mecanum_rosserial_Driver_Step(float*, float*, float*, float*, float, float, float, float, float, float, float, float, float, float);
        void Robot_Mecanum_rosserial_Driver_Terminate(void);
#endif
#ifdef __cplusplus
}
#endif
#endif 