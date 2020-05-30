#if !defined(__PARAMETERS_H__)
#define __PARAMETERS_H__
    
    //////////////////////////////////////////////////////////////////////////
    // COSTANTI
    //////////////////////////////////////////////////////////////////////////
    #define DEG2RAD 0.0174533f  // pi/180
    #define RAD2DEG 57.2957795f //= 180/PI
    
    // parametri meccanici-------------------
    #define ROBOT_WHEEL_DISTANCE 0.374f  // distanza in m tra le due ruote
    #define ROBOT_WHEEL_DIAMETER 0.1178f //DIAMETRO RUOTE con cinghia
    
    
    // Encoders --------------------------------
    #define METER2ENC_L_DEFAULT 292.0f /* 1/610 */
    #define METER2ENC_R_DEFAULT 414.0f /*1.0/250 */
    
    // cinematica --------------------------------
    #define ROBOT_ROTATION_CIRCUMPHERENCE_CM = 109.955f // circonferenza = pi*ROBOT_WEEL_DISTANCE=109,956cm
    
    
    #define  MOTOR_MINIMUM_ANGULAR_VELOCITY 0.10
    #define  MOTOR_MAXIMUM_ANGULAR_VELOCITY 10.0
    
    //Nuove costanti
    #define ROBOT_M2STEPS 3744.82219040  // was 3536.7765f	 //   1m/s=ROBOT_MOTOR_STEPS_PER_M/s
    #define ROBOT_STEPS2M 0.000267035376 // reciproco di  ROBOT_M2STEPS  3744.82219040
    
    
    
    //posizione home (sala dietro la mia sedia)
    #define GPSHOME_LAT 45.471633
    #define GPSHOME_LNG 9.124051
    // CHIAVARI  LAT 44.326953  LNG 9.289679
#endif
