
#ifndef _CCP5_H
#define _CCP5_H

/**
  Section: Included Files
 */

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif

    /** Data Type Definition
     @Summary
       Defines the values to convert from 16bit to two 8 bit and viceversa

     @Description
       This routine used to get two 8 bit values from 16bit also
       two 8 bit value are combine to get 16bit.

     Remarks:
       None
     */

    typedef union CCPR5Reg_tag {

        struct {
            uint8_t ccpr5l;
            uint8_t ccpr5h;
        };

        struct {
            uint16_t ccpr5_16Bit;
        };
    } CCP_PERIOD_REG_T;

    /**
      Section: Capture Module APIs
     */

    /**
      @Summary
        Initializes the CCP5

      @Description
        This routine initializes the CCP5_Initialize.
        This routine must be called before any other CCP5 routine is called.
        This routine should only be called once during system initialization.

      @Preconditions
        None

      @Param
        None

      @Returns
        None

      @Comment
    

     @Example
        <code>
        CCP5_Initialize();
        </code>
     */
    void CCP5_Initialize(void);

    /**
      @Summary
        Implements ISR

      @Description
        This routine is used to implement the ISR for the interrupt-driven
        implementations.

      @Returns
        None

      @Param
        None
     */
    void CCP5_CaptureISR(void);

    int16_t getRperiod(void);
    int16_t getRspeed(void);
    int16_t getRspeedINT(void);
    int16_t getRticks(void);
    void setRticks(int16_t value);
    
    volatile uint8_t isValidR;
    volatile uint8_t stoppedR;

#ifdef __cplusplus  // Provide C++ Compatibility

}

#endif

#endif  //_CCP5_H
/**
 End of File
 */

