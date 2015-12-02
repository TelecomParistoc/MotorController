
#ifndef _CCP4_H
#define _CCP4_H

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
       Defines the values to convert from 16bit to two 8 bit and vice versa

     @Description
       This routine used to get two 8 bit values from 16bit also
       two 8 bit value are combine to get 16bit.

     Remarks:
       None
     */

    typedef union CCPR4Reg_tag {
        struct {
            uint8_t ccpr4l;
            uint8_t ccpr4h;
        };

        struct {
            uint16_t ccpr4_16Bit;
        };
    } CCP_PERIOD_REG_T;

    /**
      Section: Capture Module APIs
     */

    /**
      @Summary
        Initializes the CCP4

      @Description
        This routine initializes the CCP4_Initialize.
        This routine must be called before any other CCP4 routine is called.
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
        CCP4_Initialize();
        </code>
     */
    void CCP4_Initialize(void);

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
    void CCP4_CaptureISR(void);
    
    int16_t getLperiod(void);
    int16_t getLspeed(void);
    int16_t getLspeedINT(void);
    int16_t getLticks(void);
    void setLticks(int16_t value);
    
    volatile uint8_t isValidL;
    volatile uint8_t stoppedL;

#ifdef __cplusplus  // Provide C++ Compatibility

}

#endif

#endif  //_CCP4_H
/**
 End of File
 */

