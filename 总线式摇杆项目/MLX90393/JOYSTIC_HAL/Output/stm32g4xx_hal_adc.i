# 1 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Src\\stm32g4xx_hal_adc.c"













































































































































































































































































































 

 
# 1 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal.h"


















 

 







 
# 1 "..\\..\\User\\stm32g4xx_hal_conf.h"

















 

 







 
 

 


 










 
 
 



 
 
 
 
 
 
 
 
 
 
 
 
 
# 75 "..\\..\\User\\stm32g4xx_hal_conf.h"

 


 
# 109 "..\\..\\User\\stm32g4xx_hal_conf.h"

 




 












 










 







 


 





 












 





 

 


 

# 188 "..\\..\\User\\stm32g4xx_hal_conf.h"

 



 
 

 




 



 


 

# 1 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

















 

 







 
# 1 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_def.h"


















 

 







 
# 1 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g4xx.h"



























 



 



 










 



 






 

# 74 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g4xx.h"



 
# 86 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g4xx.h"



 
# 98 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g4xx.h"



 



 

# 1 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"























 



 



 










 



 








 



 




 
typedef enum
{
 
  NonMaskableInt_IRQn         = -14,     
  HardFault_IRQn              = -13,     
  MemoryManagement_IRQn       = -12,     
  BusFault_IRQn               = -11,     
  UsageFault_IRQn             = -10,     
  SVCall_IRQn                 = -5,      
  DebugMonitor_IRQn           = -4,      
  PendSV_IRQn                 = -2,      
  SysTick_IRQn                = -1,      
 
  WWDG_IRQn                   = 0,       
  PVD_PVM_IRQn                = 1,       
  RTC_TAMP_LSECSS_IRQn        = 2,       
  RTC_WKUP_IRQn               = 3,       
  FLASH_IRQn                  = 4,       
  RCC_IRQn                    = 5,       
  EXTI0_IRQn                  = 6,       
  EXTI1_IRQn                  = 7,       
  EXTI2_IRQn                  = 8,       
  EXTI3_IRQn                  = 9,       
  EXTI4_IRQn                  = 10,      
  DMA1_Channel1_IRQn          = 11,      
  DMA1_Channel2_IRQn          = 12,      
  DMA1_Channel3_IRQn          = 13,      
  DMA1_Channel4_IRQn          = 14,      
  DMA1_Channel5_IRQn          = 15,      
  DMA1_Channel6_IRQn          = 16,      
  ADC1_2_IRQn                 = 18,      
  USB_HP_IRQn                 = 19,      
  USB_LP_IRQn                 = 20,      
  FDCAN1_IT0_IRQn             = 21,      
  FDCAN1_IT1_IRQn             = 22,      
  EXTI9_5_IRQn                = 23,      
  TIM1_BRK_TIM15_IRQn         = 24,      
  TIM1_UP_TIM16_IRQn          = 25,      
  TIM1_TRG_COM_TIM17_IRQn     = 26,      
  TIM1_CC_IRQn                = 27,      
  TIM2_IRQn                   = 28,      
  TIM3_IRQn                   = 29,      
  TIM4_IRQn                   = 30,      
  I2C1_EV_IRQn                = 31,      
  I2C1_ER_IRQn                = 32,      
  I2C2_EV_IRQn                = 33,      
  I2C2_ER_IRQn                = 34,      
  SPI1_IRQn                   = 35,      
  SPI2_IRQn                   = 36,      
  USART1_IRQn                 = 37,      
  USART2_IRQn                 = 38,      
  USART3_IRQn                 = 39,      
  EXTI15_10_IRQn              = 40,      
  RTC_Alarm_IRQn              = 41,      
  USBWakeUp_IRQn              = 42,      
  TIM8_BRK_IRQn               = 43,      
  TIM8_UP_IRQn                = 44,      
  TIM8_TRG_COM_IRQn           = 45,      
  TIM8_CC_IRQn                = 46,      
  LPTIM1_IRQn                 = 49,      
  SPI3_IRQn                   = 51,      
  UART4_IRQn                  = 52,      
  TIM6_DAC_IRQn               = 54,      
  TIM7_IRQn                   = 55,      
  DMA2_Channel1_IRQn          = 56,      
  DMA2_Channel2_IRQn          = 57,      
  DMA2_Channel3_IRQn          = 58,      
  DMA2_Channel4_IRQn          = 59,      
  DMA2_Channel5_IRQn          = 60,      
  UCPD1_IRQn                  = 63,      
  COMP1_2_3_IRQn              = 64,      
  COMP4_IRQn                  = 65,      
  CRS_IRQn                    = 75,      
  SAI1_IRQn                   = 76,      
  FPU_IRQn                    = 81,      
  RNG_IRQn                    = 90,      
  LPUART1_IRQn                = 91,      
  I2C3_EV_IRQn                = 92,      
  I2C3_ER_IRQn                = 93,      
  DMAMUX_OVR_IRQn             = 94,      
  DMA2_Channel6_IRQn          = 97,      
  CORDIC_IRQn                 = 100,     
  FMAC_IRQn                   = 101      
} IRQn_Type;



 

# 1 "..\\..\\Drivers\\CMSIS\\Include\\core_cm4.h"
 




 
















 










# 1 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
 
 





 









     
# 27 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
     











# 46 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"





 

     

     
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

     

     
     
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef   signed           int int_least32_t;
typedef   signed       __int64 int_least64_t;

     
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef unsigned           int uint_least32_t;
typedef unsigned       __int64 uint_least64_t;

     

     
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef   signed       __int64 int_fast64_t;

     
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef unsigned       __int64 uint_fast64_t;

     




typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;


     
typedef   signed     long long intmax_t;
typedef unsigned     long long uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     






     






     






     

     


     


     


     

     
# 216 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     



     






     
    
 



# 241 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     







     










     











# 305 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"






 
# 35 "..\\..\\Drivers\\CMSIS\\Include\\core_cm4.h"

















 




 



 

# 1 "..\\..\\Drivers\\CMSIS\\Include\\cmsis_version.h"
 




 
















 










 
# 64 "..\\..\\Drivers\\CMSIS\\Include\\core_cm4.h"

 









 
# 87 "..\\..\\Drivers\\CMSIS\\Include\\core_cm4.h"

# 161 "..\\..\\Drivers\\CMSIS\\Include\\core_cm4.h"

# 1 "..\\..\\Drivers\\CMSIS\\Include\\cmsis_compiler.h"
 




 
















 




# 29 "..\\..\\Drivers\\CMSIS\\Include\\cmsis_compiler.h"



 
# 1 "..\\..\\Drivers\\CMSIS\\Include\\cmsis_armcc.h"
 




 
















 









 













   
   

 




 
# 110 "..\\..\\Drivers\\CMSIS\\Include\\cmsis_armcc.h"

 





















 



 





 
 






 
 





 
static __inline uint32_t __get_CONTROL(void)
{
  register uint32_t __regControl         __asm("control");
  return(__regControl);
}






 
static __inline void __set_CONTROL(uint32_t control)
{
  register uint32_t __regControl         __asm("control");
  __regControl = control;
}






 
static __inline uint32_t __get_IPSR(void)
{
  register uint32_t __regIPSR          __asm("ipsr");
  return(__regIPSR);
}






 
static __inline uint32_t __get_APSR(void)
{
  register uint32_t __regAPSR          __asm("apsr");
  return(__regAPSR);
}






 
static __inline uint32_t __get_xPSR(void)
{
  register uint32_t __regXPSR          __asm("xpsr");
  return(__regXPSR);
}






 
static __inline uint32_t __get_PSP(void)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  return(__regProcessStackPointer);
}






 
static __inline void __set_PSP(uint32_t topOfProcStack)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  __regProcessStackPointer = topOfProcStack;
}






 
static __inline uint32_t __get_MSP(void)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  return(__regMainStackPointer);
}






 
static __inline void __set_MSP(uint32_t topOfMainStack)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  __regMainStackPointer = topOfMainStack;
}






 
static __inline uint32_t __get_PRIMASK(void)
{
  register uint32_t __regPriMask         __asm("primask");
  return(__regPriMask);
}






 
static __inline void __set_PRIMASK(uint32_t priMask)
{
  register uint32_t __regPriMask         __asm("primask");
  __regPriMask = (priMask);
}









 







 







 
static __inline uint32_t  __get_BASEPRI(void)
{
  register uint32_t __regBasePri         __asm("basepri");
  return(__regBasePri);
}






 
static __inline void __set_BASEPRI(uint32_t basePri)
{
  register uint32_t __regBasePri         __asm("basepri");
  __regBasePri = (basePri & 0xFFU);
}







 
static __inline void __set_BASEPRI_MAX(uint32_t basePri)
{
  register uint32_t __regBasePriMax      __asm("basepri_max");
  __regBasePriMax = (basePri & 0xFFU);
}






 
static __inline uint32_t __get_FAULTMASK(void)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  return(__regFaultMask);
}






 
static __inline void __set_FAULTMASK(uint32_t faultMask)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  __regFaultMask = (faultMask & (uint32_t)1U);
}









 
static __inline uint32_t __get_FPSCR(void)
{


  register uint32_t __regfpscr         __asm("fpscr");
  return(__regfpscr);



}






 
static __inline void __set_FPSCR(uint32_t fpscr)
{


  register uint32_t __regfpscr         __asm("fpscr");
  __regfpscr = (fpscr);



}


 


 



 




 






 







 






 








 










 










 






                  





 








 

__attribute__((section(".rev16_text"))) static __inline __asm uint32_t __REV16(uint32_t value)
{
  rev16 r0, r0
  bx lr
}








 

__attribute__((section(".revsh_text"))) static __inline __asm int16_t __REVSH(int16_t value)
{
  revsh r0, r0
  bx lr
}









 









 








 
# 561 "..\\..\\Drivers\\CMSIS\\Include\\cmsis_armcc.h"







 











 












 












 














 














 














 










 









 









 









 

__attribute__((section(".rrx_text"))) static __inline __asm uint32_t __RRX(uint32_t value)
{
  rrx r0, r0
  bx lr
}








 








 








 








 








 








 


# 809 "..\\..\\Drivers\\CMSIS\\Include\\cmsis_armcc.h"

   


 



 



# 880 "..\\..\\Drivers\\CMSIS\\Include\\cmsis_armcc.h"











 


# 35 "..\\..\\Drivers\\CMSIS\\Include\\cmsis_compiler.h"




 
# 280 "..\\..\\Drivers\\CMSIS\\Include\\cmsis_compiler.h"




# 163 "..\\..\\Drivers\\CMSIS\\Include\\core_cm4.h"

















 
# 207 "..\\..\\Drivers\\CMSIS\\Include\\core_cm4.h"

 






 
# 223 "..\\..\\Drivers\\CMSIS\\Include\\core_cm4.h"

 




 













 



 






 



 
typedef union
{
  struct
  {
    uint32_t _reserved0:16;               
    uint32_t GE:4;                        
    uint32_t _reserved1:7;                
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} APSR_Type;

 





















 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:23;               
  } b;                                    
  uint32_t w;                             
} IPSR_Type;

 






 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:1;                
    uint32_t ICI_IT_1:6;                  
    uint32_t GE:4;                        
    uint32_t _reserved1:4;                
    uint32_t T:1;                         
    uint32_t ICI_IT_2:2;                  
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} xPSR_Type;

 

































 
typedef union
{
  struct
  {
    uint32_t nPRIV:1;                     
    uint32_t SPSEL:1;                     
    uint32_t FPCA:1;                      
    uint32_t _reserved0:29;               
  } b;                                    
  uint32_t w;                             
} CONTROL_Type;

 









 







 



 
typedef struct
{
  volatile uint32_t ISER[8U];                
        uint32_t RESERVED0[24U];
  volatile uint32_t ICER[8U];                
        uint32_t RESERVED1[24U];
  volatile uint32_t ISPR[8U];                
        uint32_t RESERVED2[24U];
  volatile uint32_t ICPR[8U];                
        uint32_t RESERVED3[24U];
  volatile uint32_t IABR[8U];                
        uint32_t RESERVED4[56U];
  volatile uint8_t  IP[240U];                
        uint32_t RESERVED5[644U];
  volatile  uint32_t STIR;                    
}  NVIC_Type;

 



 







 



 
typedef struct
{
  volatile const  uint32_t CPUID;                   
  volatile uint32_t ICSR;                    
  volatile uint32_t VTOR;                    
  volatile uint32_t AIRCR;                   
  volatile uint32_t SCR;                     
  volatile uint32_t CCR;                     
  volatile uint8_t  SHP[12U];                
  volatile uint32_t SHCSR;                   
  volatile uint32_t CFSR;                    
  volatile uint32_t HFSR;                    
  volatile uint32_t DFSR;                    
  volatile uint32_t MMFAR;                   
  volatile uint32_t BFAR;                    
  volatile uint32_t AFSR;                    
  volatile const  uint32_t PFR[2U];                 
  volatile const  uint32_t DFR;                     
  volatile const  uint32_t ADR;                     
  volatile const  uint32_t MMFR[4U];                
  volatile const  uint32_t ISAR[5U];                
        uint32_t RESERVED0[5U];
  volatile uint32_t CPACR;                   
} SCB_Type;

 















 






























 



 





















 









 


















 










































 









 


















 





















 


















 









 















 







 



 
typedef struct
{
        uint32_t RESERVED0[1U];
  volatile const  uint32_t ICTR;                    
  volatile uint32_t ACTLR;                   
} SCnSCB_Type;

 



 















 







 



 
typedef struct
{
  volatile uint32_t CTRL;                    
  volatile uint32_t LOAD;                    
  volatile uint32_t VAL;                     
  volatile const  uint32_t CALIB;                   
} SysTick_Type;

 












 



 



 









 







 



 
typedef struct
{
  volatile  union
  {
    volatile  uint8_t    u8;                  
    volatile  uint16_t   u16;                 
    volatile  uint32_t   u32;                 
  }  PORT [32U];                          
        uint32_t RESERVED0[864U];
  volatile uint32_t TER;                     
        uint32_t RESERVED1[15U];
  volatile uint32_t TPR;                     
        uint32_t RESERVED2[15U];
  volatile uint32_t TCR;                     
        uint32_t RESERVED3[32U];
        uint32_t RESERVED4[43U];
  volatile  uint32_t LAR;                     
  volatile const  uint32_t LSR;                     
        uint32_t RESERVED5[6U];
  volatile const  uint32_t PID4;                    
  volatile const  uint32_t PID5;                    
  volatile const  uint32_t PID6;                    
  volatile const  uint32_t PID7;                    
  volatile const  uint32_t PID0;                    
  volatile const  uint32_t PID1;                    
  volatile const  uint32_t PID2;                    
  volatile const  uint32_t PID3;                    
  volatile const  uint32_t CID0;                    
  volatile const  uint32_t CID1;                    
  volatile const  uint32_t CID2;                    
  volatile const  uint32_t CID3;                    
} ITM_Type;

 



 



























 









   







 



 
typedef struct
{
  volatile uint32_t CTRL;                    
  volatile uint32_t CYCCNT;                  
  volatile uint32_t CPICNT;                  
  volatile uint32_t EXCCNT;                  
  volatile uint32_t SLEEPCNT;                
  volatile uint32_t LSUCNT;                  
  volatile uint32_t FOLDCNT;                 
  volatile const  uint32_t PCSR;                    
  volatile uint32_t COMP0;                   
  volatile uint32_t MASK0;                   
  volatile uint32_t FUNCTION0;               
        uint32_t RESERVED0[1U];
  volatile uint32_t COMP1;                   
  volatile uint32_t MASK1;                   
  volatile uint32_t FUNCTION1;               
        uint32_t RESERVED1[1U];
  volatile uint32_t COMP2;                   
  volatile uint32_t MASK2;                   
  volatile uint32_t FUNCTION2;               
        uint32_t RESERVED2[1U];
  volatile uint32_t COMP3;                   
  volatile uint32_t MASK3;                   
  volatile uint32_t FUNCTION3;               
} DWT_Type;

 






















































 



 



 



 



 



 



 



























   







 



 
typedef struct
{
  volatile const  uint32_t SSPSR;                   
  volatile uint32_t CSPSR;                   
        uint32_t RESERVED0[2U];
  volatile uint32_t ACPR;                    
        uint32_t RESERVED1[55U];
  volatile uint32_t SPPR;                    
        uint32_t RESERVED2[131U];
  volatile const  uint32_t FFSR;                    
  volatile uint32_t FFCR;                    
  volatile const  uint32_t FSCR;                    
        uint32_t RESERVED3[759U];
  volatile const  uint32_t TRIGGER;                 
  volatile const  uint32_t FIFO0;                   
  volatile const  uint32_t ITATBCTR2;               
        uint32_t RESERVED4[1U];
  volatile const  uint32_t ITATBCTR0;               
  volatile const  uint32_t FIFO1;                   
  volatile uint32_t ITCTRL;                  
        uint32_t RESERVED5[39U];
  volatile uint32_t CLAIMSET;                
  volatile uint32_t CLAIMCLR;                
        uint32_t RESERVED7[8U];
  volatile const  uint32_t DEVID;                   
  volatile const  uint32_t DEVTYPE;                 
} TPI_Type;

 



 



 












 






 



 





















 






 





















 






 



 


















 






   








 



 
typedef struct
{
  volatile const  uint32_t TYPE;                    
  volatile uint32_t CTRL;                    
  volatile uint32_t RNR;                     
  volatile uint32_t RBAR;                    
  volatile uint32_t RASR;                    
  volatile uint32_t RBAR_A1;                 
  volatile uint32_t RASR_A1;                 
  volatile uint32_t RBAR_A2;                 
  volatile uint32_t RASR_A2;                 
  volatile uint32_t RBAR_A3;                 
  volatile uint32_t RASR_A3;                 
} MPU_Type;



 









 









 



 









 






























 








 



 
typedef struct
{
        uint32_t RESERVED0[1U];
  volatile uint32_t FPCCR;                   
  volatile uint32_t FPCAR;                   
  volatile uint32_t FPDSCR;                  
  volatile const  uint32_t MVFR0;                   
  volatile const  uint32_t MVFR1;                   
  volatile const  uint32_t MVFR2;                   
} FPU_Type;

 



























 



 












 
























 












 




 







 



 
typedef struct
{
  volatile uint32_t DHCSR;                   
  volatile  uint32_t DCRSR;                   
  volatile uint32_t DCRDR;                   
  volatile uint32_t DEMCR;                   
} CoreDebug_Type;

 




































 






 







































 







 






 







 


 







 

 
# 1553 "..\\..\\Drivers\\CMSIS\\Include\\core_cm4.h"

# 1562 "..\\..\\Drivers\\CMSIS\\Include\\core_cm4.h"









 










 


 



 





 

# 1616 "..\\..\\Drivers\\CMSIS\\Include\\core_cm4.h"

# 1626 "..\\..\\Drivers\\CMSIS\\Include\\core_cm4.h"




 
# 1637 "..\\..\\Drivers\\CMSIS\\Include\\core_cm4.h"










 
static __inline void __NVIC_SetPriorityGrouping(uint32_t PriorityGroup)
{
  uint32_t reg_value;
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);              

  reg_value  =  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR;                                                    
  reg_value &= ~((uint32_t)((0xFFFFUL << 16U) | (7UL << 8U)));  
  reg_value  =  (reg_value                                   |
                ((uint32_t)0x5FAUL << 16U) |
                (PriorityGroupTmp << 8U)  );               
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR =  reg_value;
}






 
static __inline uint32_t __NVIC_GetPriorityGrouping(void)
{
  return ((uint32_t)((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8U)) >> 8U));
}







 
static __inline void __NVIC_EnableIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    __memory_changed();
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
    __memory_changed();
  }
}









 
static __inline uint32_t __NVIC_GetEnableIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[(((uint32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return(0U);
  }
}







 
static __inline void __NVIC_DisableIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
    do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);
    do { __schedule_barrier(); __isb(0xF); __schedule_barrier(); } while (0U);
  }
}









 
static __inline uint32_t __NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[(((uint32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return(0U);
  }
}







 
static __inline void __NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
  }
}







 
static __inline void __NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICPR[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
  }
}









 
static __inline uint32_t __NVIC_GetActive(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IABR[(((uint32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return(0U);
  }
}










 
static __inline void __NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if ((int32_t)(IRQn) >= 0)
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[((uint32_t)IRQn)]               = (uint8_t)((priority << (8U - 4)) & (uint32_t)0xFFUL);
  }
  else
  {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[(((uint32_t)IRQn) & 0xFUL)-4UL] = (uint8_t)((priority << (8U - 4)) & (uint32_t)0xFFUL);
  }
}










 
static __inline uint32_t __NVIC_GetPriority(IRQn_Type IRQn)
{

  if ((int32_t)(IRQn) >= 0)
  {
    return(((uint32_t)((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[((uint32_t)IRQn)]               >> (8U - 4)));
  }
  else
  {
    return(((uint32_t)((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[(((uint32_t)IRQn) & 0xFUL)-4UL] >> (8U - 4)));
  }
}












 
static __inline uint32_t NVIC_EncodePriority (uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);    
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7UL - PriorityGroupTmp) > (uint32_t)(4)) ? (uint32_t)(4) : (uint32_t)(7UL - PriorityGroupTmp);
  SubPriorityBits     = ((PriorityGroupTmp + (uint32_t)(4)) < (uint32_t)7UL) ? (uint32_t)0UL : (uint32_t)((PriorityGroupTmp - 7UL) + (uint32_t)(4));

  return (
           ((PreemptPriority & (uint32_t)((1UL << (PreemptPriorityBits)) - 1UL)) << SubPriorityBits) |
           ((SubPriority     & (uint32_t)((1UL << (SubPriorityBits    )) - 1UL)))
         );
}












 
static __inline void NVIC_DecodePriority (uint32_t Priority, uint32_t PriorityGroup, uint32_t* const pPreemptPriority, uint32_t* const pSubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);    
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7UL - PriorityGroupTmp) > (uint32_t)(4)) ? (uint32_t)(4) : (uint32_t)(7UL - PriorityGroupTmp);
  SubPriorityBits     = ((PriorityGroupTmp + (uint32_t)(4)) < (uint32_t)7UL) ? (uint32_t)0UL : (uint32_t)((PriorityGroupTmp - 7UL) + (uint32_t)(4));

  *pPreemptPriority = (Priority >> SubPriorityBits) & (uint32_t)((1UL << (PreemptPriorityBits)) - 1UL);
  *pSubPriority     = (Priority                   ) & (uint32_t)((1UL << (SubPriorityBits    )) - 1UL);
}










 
static __inline void __NVIC_SetVector(IRQn_Type IRQn, uint32_t vector)
{
  uint32_t vectors = (uint32_t )((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->VTOR;
  (* (int *) (vectors + ((int32_t)IRQn + 16) * 4)) = vector;
   
}









 
static __inline uint32_t __NVIC_GetVector(IRQn_Type IRQn)
{
  uint32_t vectors = (uint32_t )((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->VTOR;
  return (uint32_t)(* (int *) (vectors + ((int32_t)IRQn + 16) * 4));
}





 
__declspec(noreturn) static __inline void __NVIC_SystemReset(void)
{
  do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);                                                          
 
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR  = (uint32_t)((0x5FAUL << 16U)    |
                           (((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8U)) |
                            (1UL << 2U)    );          
  do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);                                                           

  for(;;)                                                            
  {
    __nop();
  }
}

 


 



# 1 "..\\..\\Drivers\\CMSIS\\Include\\mpu_armv7.h"





 
















 
 





 



# 62 "..\\..\\Drivers\\CMSIS\\Include\\mpu_armv7.h"

# 69 "..\\..\\Drivers\\CMSIS\\Include\\mpu_armv7.h"





 












   














 
# 110 "..\\..\\Drivers\\CMSIS\\Include\\mpu_armv7.h"












                          









  










  












  




 




 




 




 





 
typedef struct {
  uint32_t RBAR; 
  uint32_t RASR; 
} ARM_MPU_Region_t;
    


 
static __inline void ARM_MPU_Enable(uint32_t MPU_Control)
{
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->CTRL = MPU_Control | (1UL );

  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHCSR |= (1UL << 16U);

  do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);
  do { __schedule_barrier(); __isb(0xF); __schedule_barrier(); } while (0U);
}


 
static __inline void ARM_MPU_Disable(void)
{
  do { __schedule_barrier(); __dmb(0xF); __schedule_barrier(); } while (0U);

  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHCSR &= ~(1UL << 16U);

  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->CTRL  &= ~(1UL );
}



 
static __inline void ARM_MPU_ClrRegion(uint32_t rnr)
{
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RNR = rnr;
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RASR = 0U;
}




    
static __inline void ARM_MPU_SetRegion(uint32_t rbar, uint32_t rasr)
{
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RBAR = rbar;
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RASR = rasr;
}





    
static __inline void ARM_MPU_SetRegionEx(uint32_t rnr, uint32_t rbar, uint32_t rasr)
{
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RNR = rnr;
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RBAR = rbar;
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RASR = rasr;
}





 
static __inline void ARM_MPU_OrderedMemcpy(volatile uint32_t* dst, const uint32_t* __restrict src, uint32_t len)
{
  uint32_t i;
  for (i = 0U; i < len; ++i) 
  {
    dst[i] = src[i];
  }
}




 
static __inline void ARM_MPU_Load(ARM_MPU_Region_t const* table, uint32_t cnt) 
{
  const uint32_t rowWordSize = sizeof(ARM_MPU_Region_t)/4U;
  while (cnt > 4U) {
    ARM_MPU_OrderedMemcpy(&(((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RBAR), &(table->RBAR), 4U*rowWordSize);
    table += 4U;
    cnt -= 4U;
  }
  ARM_MPU_OrderedMemcpy(&(((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RBAR), &(table->RBAR), cnt*rowWordSize);
}

# 1956 "..\\..\\Drivers\\CMSIS\\Include\\core_cm4.h"




 





 








 
static __inline uint32_t SCB_GetFPUType(void)
{
  uint32_t mvfr0;

  mvfr0 = ((FPU_Type *) ((0xE000E000UL) + 0x0F30UL) )->MVFR0;
  if      ((mvfr0 & ((0xFUL << 4U) | (0xFUL << 8U))) == 0x020U)
  {
    return 1U;            
  }
  else
  {
    return 0U;            
  }
}


 



 





 













 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{
  if ((ticks - 1UL) > (0xFFFFFFUL ))
  {
    return (1UL);                                                    
  }

  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD  = (uint32_t)(ticks - 1UL);                          
  __NVIC_SetPriority (SysTick_IRQn, (1UL << 4) - 1UL);  
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL   = 0UL;                                              
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL  = (1UL << 2U) |
                   (1UL << 1U)   |
                   (1UL );                          
  return (0UL);                                                      
}



 



 





 

extern volatile int32_t ITM_RxBuffer;                               










 
static __inline uint32_t ITM_SendChar (uint32_t ch)
{
  if (((((ITM_Type *) (0xE0000000UL) )->TCR & (1UL )) != 0UL) &&       
      ((((ITM_Type *) (0xE0000000UL) )->TER & 1UL               ) != 0UL)   )      
  {
    while (((ITM_Type *) (0xE0000000UL) )->PORT[0U].u32 == 0UL)
    {
      __nop();
    }
    ((ITM_Type *) (0xE0000000UL) )->PORT[0U].u8 = (uint8_t)ch;
  }
  return (ch);
}







 
static __inline int32_t ITM_ReceiveChar (void)
{
  int32_t ch = -1;                            

  if (ITM_RxBuffer != ((int32_t)0x5AA55AA5U))
  {
    ch = ITM_RxBuffer;
    ITM_RxBuffer = ((int32_t)0x5AA55AA5U);        
  }

  return (ch);
}







 
static __inline int32_t ITM_CheckChar (void)
{

  if (ITM_RxBuffer == ((int32_t)0x5AA55AA5U))
  {
    return (0);                               
  }
  else
  {
    return (1);                               
  }
}

 










# 156 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"
# 1 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\system_stm32g4xx.h"

















 



 



 



 









 



 




 
  






 
extern uint32_t SystemCoreClock;             

extern const uint8_t  AHBPrescTable[16];     
extern const uint8_t  APBPrescTable[8];      



 



 



 



 



 



 

extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);


 









 



 
 
# 157 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"
# 158 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"



 



 

typedef struct
{
  volatile uint32_t ISR;           
  volatile uint32_t IER;           
  volatile uint32_t CR;            
  volatile uint32_t CFGR;          
  volatile uint32_t CFGR2;         
  volatile uint32_t SMPR1;         
  volatile uint32_t SMPR2;         
       uint32_t RESERVED1;     
  volatile uint32_t TR1;           
  volatile uint32_t TR2;           
  volatile uint32_t TR3;           
       uint32_t RESERVED2;     
  volatile uint32_t SQR1;          
  volatile uint32_t SQR2;          
  volatile uint32_t SQR3;          
  volatile uint32_t SQR4;          
  volatile uint32_t DR;            
       uint32_t RESERVED3;     
       uint32_t RESERVED4;     
  volatile uint32_t JSQR;          
       uint32_t RESERVED5[4];  
  volatile uint32_t OFR1;          
  volatile uint32_t OFR2;          
  volatile uint32_t OFR3;          
  volatile uint32_t OFR4;          
       uint32_t RESERVED6[4];  
  volatile uint32_t JDR1;          
  volatile uint32_t JDR2;          
  volatile uint32_t JDR3;          
  volatile uint32_t JDR4;          
       uint32_t RESERVED7[4];  
  volatile uint32_t AWD2CR;        
  volatile uint32_t AWD3CR;        
       uint32_t RESERVED8;     
       uint32_t RESERVED9;     
  volatile uint32_t DIFSEL;        
  volatile uint32_t CALFACT;       
       uint32_t RESERVED10[2]; 
  volatile uint32_t GCOMP;         
} ADC_TypeDef;

typedef struct
{
  volatile uint32_t CSR;           
  uint32_t      RESERVED1;     
  volatile uint32_t CCR;           
  volatile uint32_t CDR;           
} ADC_Common_TypeDef;



 

typedef struct
{
  volatile uint32_t CREL;          
  volatile uint32_t ENDN;          
       uint32_t RESERVED1;     
  volatile uint32_t DBTP;          
  volatile uint32_t TEST;          
  volatile uint32_t RWD;           
  volatile uint32_t CCCR;          
  volatile uint32_t NBTP;          
  volatile uint32_t TSCC;          
  volatile uint32_t TSCV;          
  volatile uint32_t TOCC;          
  volatile uint32_t TOCV;          
       uint32_t RESERVED2[4];  
  volatile uint32_t ECR;           
  volatile uint32_t PSR;           
  volatile uint32_t TDCR;          
       uint32_t RESERVED3;     
  volatile uint32_t IR;            
  volatile uint32_t IE;            
  volatile uint32_t ILS;           
  volatile uint32_t ILE;           
       uint32_t RESERVED4[8];  
  volatile uint32_t RXGFC;         
  volatile uint32_t XIDAM;         
  volatile uint32_t HPMS;          
       uint32_t RESERVED5;     
  volatile uint32_t RXF0S;         
  volatile uint32_t RXF0A;         
  volatile uint32_t RXF1S;         
  volatile uint32_t RXF1A;         
       uint32_t RESERVED6[8];  
  volatile uint32_t TXBC;          
  volatile uint32_t TXFQS;         
  volatile uint32_t TXBRP;         
  volatile uint32_t TXBAR;         
  volatile uint32_t TXBCR;         
  volatile uint32_t TXBTO;         
  volatile uint32_t TXBCF;         
  volatile uint32_t TXBTIE;        
  volatile uint32_t TXBCIE;        
  volatile uint32_t TXEFS;         
  volatile uint32_t TXEFA;         
} FDCAN_GlobalTypeDef;



 

typedef struct
{
  volatile uint32_t CKDIV;         
} FDCAN_Config_TypeDef;



 

typedef struct
{
  volatile uint32_t CSR;          
} COMP_TypeDef;



 

typedef struct
{
  volatile uint32_t DR;           
  volatile uint32_t IDR;          
  volatile uint32_t CR;           
  uint32_t      RESERVED0;    
  volatile uint32_t INIT;         
  volatile uint32_t POL;          
} CRC_TypeDef;



 
typedef struct
{
  volatile uint32_t CR;           
  volatile uint32_t CFGR;         
  volatile uint32_t ISR;          
  volatile uint32_t ICR;          
} CRS_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;           
  volatile uint32_t SWTRIGR;      
  volatile uint32_t DHR12R1;      
  volatile uint32_t DHR12L1;      
  volatile uint32_t DHR8R1;       
  volatile uint32_t DHR12R2;      
  volatile uint32_t DHR12L2;      
  volatile uint32_t DHR8R2;       
  volatile uint32_t DHR12RD;      
  volatile uint32_t DHR12LD;      
  volatile uint32_t DHR8RD;       
  volatile uint32_t DOR1;         
  volatile uint32_t DOR2;         
  volatile uint32_t SR;           
  volatile uint32_t CCR;          
  volatile uint32_t MCR;          
  volatile uint32_t SHSR1;        
  volatile uint32_t SHSR2;        
  volatile uint32_t SHHR;         
  volatile uint32_t SHRR;         
  volatile uint32_t RESERVED[2];
  volatile uint32_t STR1;         
  volatile uint32_t STR2;         
  volatile uint32_t STMODR;       
} DAC_TypeDef;



 

typedef struct
{
  volatile uint32_t IDCODE;       
  volatile uint32_t CR;           
  volatile uint32_t APB1FZR1;     
  volatile uint32_t APB1FZR2;     
  volatile uint32_t APB2FZ;       
} DBGMCU_TypeDef;



 

typedef struct
{
  volatile uint32_t CCR;          
  volatile uint32_t CNDTR;        
  volatile uint32_t CPAR;         
  volatile uint32_t CMAR;         
} DMA_Channel_TypeDef;

typedef struct
{
  volatile uint32_t ISR;          
  volatile uint32_t IFCR;         
} DMA_TypeDef;



 

typedef struct
{
  volatile uint32_t   CCR;        
}DMAMUX_Channel_TypeDef;

typedef struct
{
  volatile uint32_t   CSR;       
  volatile uint32_t   CFR;       
}DMAMUX_ChannelStatus_TypeDef;

typedef struct
{
  volatile uint32_t   RGCR;         
}DMAMUX_RequestGen_TypeDef;

typedef struct
{
  volatile uint32_t   RGSR;         
  volatile uint32_t   RGCFR;         
}DMAMUX_RequestGenStatus_TypeDef;



 

typedef struct
{
  volatile uint32_t IMR1;         
  volatile uint32_t EMR1;         
  volatile uint32_t RTSR1;        
  volatile uint32_t FTSR1;        
  volatile uint32_t SWIER1;       
  volatile uint32_t PR1;          
  uint32_t      RESERVED1;    
  uint32_t      RESERVED2;    
  volatile uint32_t IMR2;         
  volatile uint32_t EMR2;         
  volatile uint32_t RTSR2;        
  volatile uint32_t FTSR2;        
  volatile uint32_t SWIER2;       
  volatile uint32_t PR2;          
} EXTI_TypeDef;



 

typedef struct
{
  volatile uint32_t ACR;               
  volatile uint32_t PDKEYR;            
  volatile uint32_t KEYR;              
  volatile uint32_t OPTKEYR;           
  volatile uint32_t SR;                
  volatile uint32_t CR;                
  volatile uint32_t ECCR;              
       uint32_t RESERVED1;         
  volatile uint32_t OPTR;              
  volatile uint32_t PCROP1SR;          
  volatile uint32_t PCROP1ER;          
  volatile uint32_t WRP1AR;            
  volatile uint32_t WRP1BR;            
       uint32_t RESERVED2[15];     
  volatile uint32_t SEC1R;             
} FLASH_TypeDef;



 
typedef struct
{
  volatile uint32_t X1BUFCFG;         
  volatile uint32_t X2BUFCFG;         
  volatile uint32_t YBUFCFG;          
  volatile uint32_t PARAM;            
  volatile uint32_t CR;               
  volatile uint32_t SR;               
  volatile uint32_t WDATA;            
  volatile uint32_t RDATA;            
} FMAC_TypeDef;




 

typedef struct
{
  volatile uint32_t MODER;        
  volatile uint32_t OTYPER;       
  volatile uint32_t OSPEEDR;      
  volatile uint32_t PUPDR;        
  volatile uint32_t IDR;          
  volatile uint32_t ODR;          
  volatile uint32_t BSRR;         
  volatile uint32_t LCKR;         
  volatile uint32_t AFR[2];       
  volatile uint32_t BRR;          
} GPIO_TypeDef;



 

typedef struct
{
  volatile uint32_t CR1;          
  volatile uint32_t CR2;          
  volatile uint32_t OAR1;         
  volatile uint32_t OAR2;         
  volatile uint32_t TIMINGR;      
  volatile uint32_t TIMEOUTR;     
  volatile uint32_t ISR;          
  volatile uint32_t ICR;          
  volatile uint32_t PECR;         
  volatile uint32_t RXDR;         
  volatile uint32_t TXDR;         
} I2C_TypeDef;



 

typedef struct
{
  volatile uint32_t KR;           
  volatile uint32_t PR;           
  volatile uint32_t RLR;          
  volatile uint32_t SR;           
  volatile uint32_t WINR;         
} IWDG_TypeDef;



 

typedef struct
{
  volatile uint32_t ISR;               
  volatile uint32_t ICR;               
  volatile uint32_t IER;               
  volatile uint32_t CFGR;              
  volatile uint32_t CR;                
  volatile uint32_t CMP;               
  volatile uint32_t ARR;               
  volatile uint32_t CNT;               
  volatile uint32_t OR;                
} LPTIM_TypeDef;



 

typedef struct
{
  volatile uint32_t CSR;            
  volatile uint32_t RESERVED[5];    
  volatile uint32_t TCMR;           
} OPAMP_TypeDef;



 

typedef struct
{
  volatile uint32_t CR1;       
  volatile uint32_t CR2;       
  volatile uint32_t CR3;       
  volatile uint32_t CR4;       
  volatile uint32_t SR1;       
  volatile uint32_t SR2;       
  volatile uint32_t SCR;       
  uint32_t RESERVED;       
  volatile uint32_t PUCRA;     
  volatile uint32_t PDCRA;     
  volatile uint32_t PUCRB;     
  volatile uint32_t PDCRB;     
  volatile uint32_t PUCRC;     
  volatile uint32_t PDCRC;     
  volatile uint32_t PUCRD;     
  volatile uint32_t PDCRD;     
  volatile uint32_t PUCRE;     
  volatile uint32_t PDCRE;     
  volatile uint32_t PUCRF;     
  volatile uint32_t PDCRF;     
  volatile uint32_t PUCRG;     
  volatile uint32_t PDCRG;     
  uint32_t RESERVED1[10];  
  volatile uint32_t CR5;       
} PWR_TypeDef;




 

typedef struct
{
  volatile uint32_t CR;           
  volatile uint32_t ICSCR;        
  volatile uint32_t CFGR;         
  volatile uint32_t PLLCFGR;      
  uint32_t      RESERVED0;    
  uint32_t      RESERVED1;    
  volatile uint32_t CIER;         
  volatile uint32_t CIFR;         
  volatile uint32_t CICR;         
  uint32_t      RESERVED2;    
  volatile uint32_t AHB1RSTR;     
  volatile uint32_t AHB2RSTR;     
  volatile uint32_t AHB3RSTR;     
  uint32_t      RESERVED3;    
  volatile uint32_t APB1RSTR1;    
  volatile uint32_t APB1RSTR2;    
  volatile uint32_t APB2RSTR;     
  uint32_t      RESERVED4;    
  volatile uint32_t AHB1ENR;      
  volatile uint32_t AHB2ENR;      
  volatile uint32_t AHB3ENR;      
  uint32_t      RESERVED5;    
  volatile uint32_t APB1ENR1;     
  volatile uint32_t APB1ENR2;     
  volatile uint32_t APB2ENR;      
  uint32_t      RESERVED6;    
  volatile uint32_t AHB1SMENR;    
  volatile uint32_t AHB2SMENR;    
  volatile uint32_t AHB3SMENR;    
  uint32_t      RESERVED7;    
  volatile uint32_t APB1SMENR1;   
  volatile uint32_t APB1SMENR2;   
  volatile uint32_t APB2SMENR;    
  uint32_t      RESERVED8;    
  volatile uint32_t CCIPR;        
  uint32_t      RESERVED9;    
  volatile uint32_t BDCR;         
  volatile uint32_t CSR;          
  volatile uint32_t CRRCR;        
  volatile uint32_t CCIPR2;       
} RCC_TypeDef;



 


 







typedef struct
{
  volatile uint32_t TR;           
  volatile uint32_t DR;           
  volatile uint32_t SSR;          
  volatile uint32_t ICSR;         
  volatile uint32_t PRER;         
  volatile uint32_t WUTR;         
  volatile uint32_t CR;           
       uint32_t RESERVED0;    
       uint32_t RESERVED1;    
  volatile uint32_t WPR;          
  volatile uint32_t CALR;         
  volatile uint32_t SHIFTR;       
  volatile uint32_t TSTR;         
  volatile uint32_t TSDR;         
  volatile uint32_t TSSSR;        
       uint32_t RESERVED2;    
  volatile uint32_t ALRMAR;       
  volatile uint32_t ALRMASSR;     
  volatile uint32_t ALRMBR;       
  volatile uint32_t ALRMBSSR;     
  volatile uint32_t SR;           
  volatile uint32_t MISR;         
       uint32_t RESERVED3;    
  volatile uint32_t SCR;          
} RTC_TypeDef;



 

typedef struct
{
  volatile uint32_t CR1;                      
  volatile uint32_t CR2;                      
       uint32_t RESERVED0;                
  volatile uint32_t FLTCR;                    
       uint32_t RESERVED1[6];             
       uint32_t RESERVED2;                
  volatile uint32_t IER;                      
  volatile uint32_t SR;                       
  volatile uint32_t MISR;                     
       uint32_t RESERVED3;                
  volatile uint32_t SCR;                      
       uint32_t RESERVED4[48];            
  volatile uint32_t BKP0R;                    
  volatile uint32_t BKP1R;                    
  volatile uint32_t BKP2R;                    
  volatile uint32_t BKP3R;                    
  volatile uint32_t BKP4R;                    
  volatile uint32_t BKP5R;                    
  volatile uint32_t BKP6R;                    
  volatile uint32_t BKP7R;                    
  volatile uint32_t BKP8R;                    
  volatile uint32_t BKP9R;                    
  volatile uint32_t BKP10R;                   
  volatile uint32_t BKP11R;                   
  volatile uint32_t BKP12R;                   
  volatile uint32_t BKP13R;                   
  volatile uint32_t BKP14R;                   
  volatile uint32_t BKP15R;                   
} TAMP_TypeDef;



 

typedef struct
{
  volatile uint32_t GCR;           
  uint32_t      RESERVED[16];  
  volatile uint32_t PDMCR;         
  volatile uint32_t PDMDLY;        
} SAI_TypeDef;

typedef struct
{
  volatile uint32_t CR1;          
  volatile uint32_t CR2;          
  volatile uint32_t FRCR;         
  volatile uint32_t SLOTR;        
  volatile uint32_t IMR;          
  volatile uint32_t SR;           
  volatile uint32_t CLRFR;        
  volatile uint32_t DR;           
} SAI_Block_TypeDef;



 

typedef struct
{
  volatile uint32_t CR1;          
  volatile uint32_t CR2;          
  volatile uint32_t SR;           
  volatile uint32_t DR;           
  volatile uint32_t CRCPR;        
  volatile uint32_t RXCRCR;       
  volatile uint32_t TXCRCR;       
  volatile uint32_t I2SCFGR;      
  volatile uint32_t I2SPR;        
} SPI_TypeDef;



 

typedef struct
{
  volatile uint32_t MEMRMP;       
  volatile uint32_t CFGR1;        
  volatile uint32_t EXTICR[4];    
  volatile uint32_t SCSR;         
  volatile uint32_t CFGR2;        
  volatile uint32_t SWPR;         
  volatile uint32_t SKR;          
} SYSCFG_TypeDef;



 

typedef struct
{
  volatile uint32_t CR1;          
  volatile uint32_t CR2;          
  volatile uint32_t SMCR;         
  volatile uint32_t DIER;         
  volatile uint32_t SR;           
  volatile uint32_t EGR;          
  volatile uint32_t CCMR1;        
  volatile uint32_t CCMR2;        
  volatile uint32_t CCER;         
  volatile uint32_t CNT;          
  volatile uint32_t PSC;          
  volatile uint32_t ARR;          
  volatile uint32_t RCR;          
  volatile uint32_t CCR1;         
  volatile uint32_t CCR2;         
  volatile uint32_t CCR3;         
  volatile uint32_t CCR4;         
  volatile uint32_t BDTR;         
  volatile uint32_t CCR5;         
  volatile uint32_t CCR6;         
  volatile uint32_t CCMR3;        
  volatile uint32_t DTR2;         
  volatile uint32_t ECR;          
  volatile uint32_t TISEL;        
  volatile uint32_t AF1;          
  volatile uint32_t AF2;          
  volatile uint32_t OR ;          
       uint32_t RESERVED0[220]; 
  volatile uint32_t DCR;          
  volatile uint32_t DMAR;         
} TIM_TypeDef;



 
typedef struct
{
  volatile uint32_t CR1;          
  volatile uint32_t CR2;          
  volatile uint32_t CR3;          
  volatile uint32_t BRR;          
  volatile uint32_t GTPR;         
  volatile uint32_t RTOR;         
  volatile uint32_t RQR;          
  volatile uint32_t ISR;          
  volatile uint32_t ICR;          
  volatile uint32_t RDR;          
  volatile uint32_t TDR;          
  volatile uint32_t PRESC;        
} USART_TypeDef;



 

typedef struct
{
  volatile uint16_t EP0R;             
  volatile uint16_t RESERVED0;        
  volatile uint16_t EP1R;             
  volatile uint16_t RESERVED1;        
  volatile uint16_t EP2R;             
  volatile uint16_t RESERVED2;        
  volatile uint16_t EP3R;             
  volatile uint16_t RESERVED3;        
  volatile uint16_t EP4R;             
  volatile uint16_t RESERVED4;        
  volatile uint16_t EP5R;             
  volatile uint16_t RESERVED5;        
  volatile uint16_t EP6R;             
  volatile uint16_t RESERVED6;        
  volatile uint16_t EP7R;             
  volatile uint16_t RESERVED7[17];    
  volatile uint16_t CNTR;             
  volatile uint16_t RESERVED8;        
  volatile uint16_t ISTR;             
  volatile uint16_t RESERVED9;        
  volatile uint16_t FNR;              
  volatile uint16_t RESERVEDA;        
  volatile uint16_t DADDR;            
  volatile uint16_t RESERVEDB;        
  volatile uint16_t BTABLE;           
  volatile uint16_t RESERVEDC;        
  volatile uint16_t LPMCSR;           
  volatile uint16_t RESERVEDD;        
  volatile uint16_t BCDR;             
  volatile uint16_t RESERVEDE;        
} USB_TypeDef;



 

typedef struct
{
  volatile uint32_t CSR;          
  volatile uint32_t CCR;          
} VREFBUF_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;           
  volatile uint32_t CFR;          
  volatile uint32_t SR;           
} WWDG_TypeDef;




 
typedef struct
{
  volatile uint32_t CR;   
  volatile uint32_t SR;   
  volatile uint32_t DR;   
} RNG_TypeDef;



 

typedef struct
{
  volatile uint32_t CSR;           
  volatile uint32_t WDATA;         
  volatile uint32_t RDATA;         
} CORDIC_TypeDef;



 

typedef struct
{
  volatile uint32_t CFG1;           
  volatile uint32_t CFG2;           
  volatile uint32_t RESERVED0;      
  volatile uint32_t CR;             
  volatile uint32_t IMR;            
  volatile uint32_t SR;             
  volatile uint32_t ICR;            
  volatile uint32_t TX_ORDSET;      
  volatile uint32_t TX_PAYSZ;       
  volatile uint32_t TXDR;           
  volatile uint32_t RX_ORDSET;      
  volatile uint32_t RX_PAYSZ;       
  volatile uint32_t RXDR;           
  volatile uint32_t RX_ORDEXT1;     
  volatile uint32_t RX_ORDEXT2;     
} UCPD_TypeDef;




 











 







 






 
# 972 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 984 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 996 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 1006 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 1013 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 1020 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 1037 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"




 
# 1049 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"










 







 



 
# 1097 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 1104 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"






# 1129 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 1144 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 1151 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 1158 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 1171 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"















 



 



 

 
 
 

 
 
 
 
 



 


 
# 1245 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 1280 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 1312 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 1320 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"







# 1335 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"







# 1357 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 1364 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 1383 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 1392 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"





 
# 1404 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 1411 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 1419 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 1426 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"





# 1440 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 1448 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 1455 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 1462 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 1469 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 1476 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 1483 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 1490 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 1497 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 1504 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 1511 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"





 
# 1523 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 1530 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 1537 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 1544 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 1551 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 1558 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 1565 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 1572 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 1579 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 




# 1591 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"





 








 








 
# 1622 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 1631 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 1640 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 1649 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 1658 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 1668 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 1677 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 1686 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 1695 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 1704 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 1714 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 1723 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 1732 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 1741 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 1750 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 1760 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 1769 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 




 






# 1790 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"







# 1805 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 1814 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 1823 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 1832 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 




# 1844 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 1853 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"





 




# 1869 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 1878 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"





 




# 1894 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 1903 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"





 




# 1919 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 1928 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"





 




 




 




 




 
# 1976 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 2000 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 2024 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 2036 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 2047 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 




 
 
# 2088 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 2122 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 2132 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 2140 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

















# 2164 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 2174 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 









 
 
 
 
 
 




# 2202 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"









# 2217 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 2224 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

















 
 
 
 
 
 
# 2291 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 




 




 
 
 
 
 
 




 




 
# 2334 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 




 




 
 
 
 

 
# 2375 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 2383 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 2390 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"











 
# 2429 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 2443 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
 
 
 
 


 


 
# 2461 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 2469 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"







# 2483 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 2493 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"





# 2504 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 2512 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"







# 2526 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 2536 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 2550 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 2558 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 2566 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 2574 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 2582 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 2590 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 2598 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 2606 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 2614 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 2622 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 2630 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 2638 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 2655 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 2671 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 2679 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 2687 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"















# 2708 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"









 




 




 
# 2734 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 2742 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 2750 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"





 
# 2762 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"





 
# 2775 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 2783 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 2791 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 2799 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
 
 
 
 
 
# 2812 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 2826 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"







 
# 2870 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"


 
# 2888 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
 
 
 
 

 
# 2968 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 3042 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 3068 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"























 




 




 




 
 
 
 
 

 
# 3124 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"



















# 3151 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 3160 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 3198 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 3236 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 3246 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"















# 3269 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 3283 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 3297 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 3323 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 3421 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"


 
 
 
 
 
 
# 3525 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 3620 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 3694 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 3768 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 3842 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 3916 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 3942 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 3968 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 3982 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 3996 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 4010 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 4024 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
 
 
 
 
 
 
# 4050 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 




 
# 4072 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 4083 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 4091 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 4135 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 4149 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 4157 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 




 
# 4173 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 




 
# 4192 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 4227 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 4235 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 4309 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 4383 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 4423 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 4431 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 4457 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 




 
# 4476 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 4493 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 




 
# 4515 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 




 




 
# 4539 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 




 




 




 




 




 




 




 
# 4591 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 





 
 




 
 
 
 
 
 
# 4653 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 4691 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 4735 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 4752 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 4809 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 




 
# 4822 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 4830 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 4838 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"


 
# 4847 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"


 
 
 
 
 
 
# 4864 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"
 
# 4871 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"
 
# 4881 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"
 
# 4904 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"
 
# 4932 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"
 
# 4948 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"
 



 





 
 
 
 
 
 
# 5044 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 5094 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 5144 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 5162 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 5244 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 5294 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 5376 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 5426 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 5476 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 5494 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 5512 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 5562 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 5580 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 5598 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 5696 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 5730 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 5783 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 5841 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 5851 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 5909 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 5919 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 5969 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 5987 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"


 
 
 
 
 
 
# 6058 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 6093 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 6104 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 6137 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 6154 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 6171 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 6224 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 6253 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 




 




 




 
 
 
 
 
 




 
# 6286 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 




 
# 6302 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 




 
 
 
 
 
 
# 6367 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 

# 6390 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"


 
 
 
 
 

 

# 6424 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"


 

 
# 6444 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 6474 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 6509 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 6532 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 6558 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 6584 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 6607 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 6654 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 6698 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 

# 6749 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 6796 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 6846 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 6896 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 6946 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 6996 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 7046 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 7096 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 7146 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 7163 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 




 
# 7203 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 





 
 
 
 
 


 




 
# 7232 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 7245 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 7252 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
 
# 7266 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 7278 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
 










 










 
# 7310 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 7320 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 7328 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"







 
# 7342 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"







 
# 7357 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 7364 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"







 
# 7378 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 






# 7392 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 7400 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 7411 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 7421 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"







# 7436 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 7445 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 7468 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 7494 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 7520 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 7543 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 7578 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 

 
# 7636 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 7644 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 7673 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 7696 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 7731 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 

 
# 7795 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 7803 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 7832 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 7858 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 7899 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 

 
# 7963 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 7971 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 8000 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 






















































































 
# 8098 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"







# 8111 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"







# 8130 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 8138 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 8163 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 8171 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 8185 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 


 
 
 
 
 
 
# 8204 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 8221 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
 
 
 
 

 
# 8270 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 8314 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 




 
# 8348 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 8356 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 




 
# 8446 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 




 
# 8474 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 8482 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 8525 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 8555 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 




 
# 8630 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 8642 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 8712 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 8724 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 8744 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 8764 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 8784 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
 
 
 
 
 
# 8812 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 8841 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 8862 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 8885 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 8908 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 8931 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 8954 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 





 
 
 
 
 
 












 












# 9073 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 9080 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"







# 9102 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 9112 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"









 
# 9128 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 9141 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"


# 9152 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 9161 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"


 
# 9175 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 9186 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 9196 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 9206 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"







# 9220 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"





 
# 9247 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 9270 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 9277 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 9300 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 




 










# 9329 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 9337 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 9344 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 9351 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 9358 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 9365 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 9372 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 9379 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 9386 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"


 
 
 
 
 


 


 
# 9408 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 9415 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 9446 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 9488 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 9527 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 




 




 




 




 
# 9582 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 9593 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
 
 
 
 
 
# 9606 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"





 
# 9645 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 9659 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"



 
# 9670 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"



 
# 9681 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"



 
# 9692 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"



 
# 9703 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 9717 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"



 
# 9728 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"



 
# 9739 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"



 
# 9750 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"



 
# 9761 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 9775 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"



 
# 9786 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"



 
# 9797 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"



 
# 9807 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"



 
# 9817 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 9831 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"



 
# 9841 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"



 
# 9851 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"



 
# 9861 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"



 
# 9871 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 9879 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 9896 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 9928 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 




 
 
 
 
 
 
# 9955 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

























 
# 9990 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 9998 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 10032 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 10040 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 10049 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"





# 10062 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"





# 10074 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"







# 10087 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"









 
# 10154 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 10216 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 10245 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"


 






# 10260 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 10268 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"











# 10285 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 10293 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"





 






# 10312 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"







# 10326 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 






# 10340 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 10348 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"











# 10365 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 10373 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"





 






# 10392 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"







# 10406 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 10414 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 10422 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"





# 10433 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 10441 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"





 
# 10507 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 10515 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 




 




 




 




 




 




 




 
# 10564 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 




 
# 10582 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"







# 10607 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 10614 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 10621 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 10628 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 10635 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 10645 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 10654 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 10693 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 10731 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 




 
# 10745 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 10753 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 10761 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 10769 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 10782 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 10789 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 




















# 10822 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 10829 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 




 
 
 
 
 
 
# 10862 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 10885 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 10908 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 






















# 10938 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 10946 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"







# 10971 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 10988 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 




 




 




 
# 11011 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 11018 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"
 
 
 
 
 
 
# 11115 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 11181 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 11266 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 11274 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 11282 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 11290 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 11307 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 11399 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 11446 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 




 




 
# 11465 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
 
 
 
 
 
# 11486 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 




 
 
 
 
 
# 11505 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 11517 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 

                                                                          
# 11527 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"


                                                                          
# 11537 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"
                                                                          
# 11545 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
 
 
 
 
# 11558 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 11575 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 11588 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 11598 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 






 
# 11615 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"



 


 
# 11631 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 





 
 




 




 




 




 




 




 




 




 

 




 




 




 




 




 




 




 




 

 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 

 




 




 




 




 




 




 




 




 

 




# 11828 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"





 




# 11846 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"





 




# 11864 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"





 




# 11882 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"





 




# 11900 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"





 




# 11918 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"





 




# 11936 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"





 




# 11954 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"





 

 


# 11970 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"



 


# 11982 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"



 


# 11994 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"



 


# 12006 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"



 


# 12018 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"



 


# 12030 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"



 


# 12042 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"



 


# 12054 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"



 


# 12066 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"



 


# 12078 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"



 


# 12090 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"



 


# 12102 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"



 


# 12114 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"



 


# 12126 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"



 


# 12138 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"



 


# 12150 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"



 
 
 
 
 
 
# 12211 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 12225 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 12275 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 12322 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 12382 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 12423 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 




 




 




 
# 12452 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 




 




 




 




 
 
 
 
 
 
# 12489 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"





 
# 12505 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

# 12512 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"





 






 



 



 

 








 



 





 


 


 




 
# 12577 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"






 


 
# 12594 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 


 


 




 


 




 



 


 




 




 


 




 



 


 


 


 
# 12663 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 



 






 






 



 
# 12695 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 12703 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 






 






 



 



 






 
# 12744 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 12754 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 12764 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 12810 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 12833 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 12843 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 






 






 
# 12865 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 12873 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 



 






 






 






 
# 12906 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 






 






 
# 12930 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 12938 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 12948 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 12957 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 






 






 



 
# 12983 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 
# 12993 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g431xx.h"

 




 




 




 





 






 




 





 






 






 






 





 






 





 




 


 


 


 


 




 


 
 
 
 
 
 
 

 



 











 

  

 

 
# 109 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g4xx.h"
# 130 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g4xx.h"



 



 
typedef enum
{
  RESET = 0,
  SET = !RESET
} FlagStatus, ITStatus;

typedef enum
{
  DISABLE = 0,
  ENABLE = !DISABLE
} FunctionalState;


typedef enum
{
  SUCCESS = 0,
  ERROR = !SUCCESS
} ErrorStatus;



 




 



















 

# 1 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal.h"


















 

 
# 625 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal.h"

 
# 188 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32G4xx\\Include\\stm32g4xx.h"









 



 




 
# 31 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_def.h"
# 1 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


















 

 







 
 
 



 







 



 
# 88 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

# 96 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"






 



 





 



 
# 134 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

# 201 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 



 



 






 



 

# 237 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"













 



 
# 269 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"





# 313 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

# 323 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

# 385 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 



 

# 482 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 



 

# 499 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 



 

# 518 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"








 




 
# 543 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 





 



 


















# 593 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"





# 604 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

# 611 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"










 



 
# 635 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

# 644 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

# 655 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

# 767 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

# 784 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 
# 807 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 





 



 






 



 















 
 






 



 














 



 










 



 







































 



 


# 950 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"






 



 

 
# 972 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

 












 



 




























# 1028 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 




 















 




 
# 1069 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 









# 1099 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 



 



# 1137 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

# 1147 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

# 1166 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"










# 1193 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 



 




 



 

























 




 








 



 




 



 
# 1273 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 



 

# 1290 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

# 1302 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

# 1333 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 



 











 

# 1381 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 



 

 



 



 



 
# 1409 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

 












# 1446 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 
# 1476 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 



 
# 1491 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 



 








# 1519 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

# 1530 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 

# 1560 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

# 1568 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"






# 1584 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"





 



 





 



 



 



 
# 1624 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 



 



 






 




 



 

 



 





 



 
# 1685 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"









 




 
# 1713 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

# 1734 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

# 1745 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

# 1754 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

# 1767 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

# 1776 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 



 







 



 
# 1812 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

# 1827 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


# 1860 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 



 
# 2027 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



# 2037 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 

# 2051 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 







 



 

# 2074 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 



 

# 2102 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 










 



 














 




 




 




 







 




 
# 2180 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 




 
# 2224 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

# 2238 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 




 








# 2512 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

# 2526 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

# 2743 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

# 2757 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

# 2764 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

# 2785 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

# 2933 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

 



# 2958 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

# 2979 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

# 3096 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

# 3105 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

# 3122 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

# 3137 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"






# 3166 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

















# 3192 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"





# 3219 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"







# 3234 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

# 3267 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

# 3285 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"












# 3303 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

# 3324 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

# 3332 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 



 




 



 
# 3355 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

# 3383 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

# 3398 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"






 



 




# 3438 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

# 3464 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

# 3471 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

# 3483 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 

# 3497 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"








 



 
# 3518 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 







 



 













 




 









# 3572 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 












# 3598 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

# 3607 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

# 3616 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"








 



 








# 3649 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"




 



 

# 3666 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"






 



 




 



 
# 3700 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 







 



 
# 3727 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 





 



 



 







 

# 32 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_def.h"
# 1 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"
 






 

 
 
 





 





# 34 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"




  typedef signed int ptrdiff_t;



  



    typedef unsigned int size_t;    
# 57 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"



   



      typedef unsigned short wchar_t;  
# 82 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"



    




   




  typedef long double max_align_t;









# 114 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"



 

# 33 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_def.h"

 



 
typedef enum
{
  HAL_OK       = 0x00U,
  HAL_ERROR    = 0x01U,
  HAL_BUSY     = 0x02U,
  HAL_TIMEOUT  = 0x03U
} HAL_StatusTypeDef;



 
typedef enum
{
  HAL_UNLOCKED = 0x00U,
  HAL_LOCKED   = 0x01U
} HAL_LockTypeDef;

 




























 


# 103 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_def.h"







# 125 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_def.h"


 
# 154 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_def.h"



 









 


# 186 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_def.h"



 



 


# 203 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_def.h"








 
# 30 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"



 



 

 


 



 
typedef struct
{
  uint32_t PLLState;   
 

  uint32_t PLLSource;  
 

  uint32_t PLLM;       
 

  uint32_t PLLN;       
 

  uint32_t PLLP;       
 

  uint32_t PLLQ;       
 

  uint32_t PLLR;       

 

}RCC_PLLInitTypeDef;



 
typedef struct
{
  uint32_t OscillatorType;       
 

  uint32_t HSEState;             
 

  uint32_t LSEState;             
 

  uint32_t HSIState;             
 

  uint32_t HSICalibrationValue;  
 

  uint32_t LSIState;             
 

  uint32_t HSI48State;             
 

  RCC_PLLInitTypeDef PLL;         

}RCC_OscInitTypeDef;



 
typedef struct
{
  uint32_t ClockType;             
 

  uint32_t SYSCLKSource;          
 

  uint32_t AHBCLKDivider;         
 

  uint32_t APB1CLKDivider;        
 

  uint32_t APB2CLKDivider;        
 

}RCC_ClkInitTypeDef;



 

 


 



 




 



 
# 152 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"


 



 





 



 





 



 





 



 




 



 




 



 





 



 
# 233 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"


 



 
# 270 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"


 



 






 



 






 



 





 



 





 



 






 



 





 



 





 



 
# 359 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"


 



 







 



 






 



 




 



 
# 406 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"


 



 







 



 
# 433 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"


 










 
 




 



 
# 465 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

 



 



 






 



 

 



 







 

# 508 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 516 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 524 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 532 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 540 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 548 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 556 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

















 







 

# 590 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 598 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 606 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 614 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 622 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 630 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 638 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 646 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 656 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 664 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 674 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 682 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 692 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 702 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 710 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"










































 







 

# 771 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 781 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"











 







 

# 809 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 817 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 825 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 835 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 843 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 851 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 859 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 867 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 875 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 883 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 891 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 899 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 907 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 917 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 927 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 935 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 943 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 951 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 961 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 969 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 977 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 985 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 993 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 1003 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 1011 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"































































 







 

# 1091 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 1099 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 1107 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 1115 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 1123 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 1133 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 1141 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 1149 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 1157 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 1167 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 1175 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

# 1185 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

































 







 































 







 
















































































 







 



















 







 





























































































































 







 
































































 




 



































 




 



















































































 




 






















 




 






















































































































 




 



































































 








 




































 








 
























































































 








 



















 








 
































































































































 








 
































































 








 




































 








 
























































































 








 



















 








 




























































































































 








 

































































 



 






 






 



 








 






 
















 











 











 












 


























 
# 2841 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"




















 
# 2879 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"







 



























 










 









 













 












 






































 
# 3012 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"







 














 













 










 









 















 





















 






 













 














 















 














 






 




















 
# 3209 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"



 



 

 


 
 







 




 

 


 

# 3248 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

























































# 3313 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"












 

 
# 1 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc_ex.h"

















 

 







 
# 30 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc_ex.h"



 



 

 



 



 
typedef struct
{
  uint32_t PeriphClockSelection;   
 

  uint32_t Usart1ClockSelection;   
 

  uint32_t Usart2ClockSelection;   
 

  uint32_t Usart3ClockSelection;   
 


  uint32_t Uart4ClockSelection;    
 








  uint32_t Lpuart1ClockSelection;  
 

  uint32_t I2c1ClockSelection;     
 

  uint32_t I2c2ClockSelection;     
 

  uint32_t I2c3ClockSelection;     
 







  uint32_t Lptim1ClockSelection;   
 

  uint32_t Sai1ClockSelection;     
 

  uint32_t I2sClockSelection;     
 


  uint32_t FdcanClockSelection;     
 



  uint32_t UsbClockSelection;      
 


  uint32_t RngClockSelection;      
 

  uint32_t Adc12ClockSelection;    
 











  uint32_t RTCClockSelection;      
 
}RCC_PeriphCLKInitTypeDef;



 
typedef struct
{
  uint32_t Prescaler;             
 

  uint32_t Source;                
 

  uint32_t Polarity;              
 

  uint32_t ReloadValue;           

 

  uint32_t ErrorLimitValue;       
 

  uint32_t HSI48CalibrationValue; 
 

}RCC_CRSInitTypeDef;



 
typedef struct
{
  uint32_t ReloadValue;           
 

  uint32_t HSI48CalibrationValue; 
 

  uint32_t FreqErrorCapture;      

 

  uint32_t FreqErrorDirection;    


 

}RCC_CRSSynchroInfoTypeDef;



 

 


 



 




 



 
# 231 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc_ex.h"


 




 






 



 






 



 






 




 






 


# 294 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc_ex.h"



 






 



 





 



 





 



 





 



 






 



 






 



 






 



 





 




 




 



 




 



 





 

# 419 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc_ex.h"

# 431 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc_ex.h"

# 443 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc_ex.h"



 



 



 
# 462 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc_ex.h"


 



 





 



 
# 487 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc_ex.h"


 



 




 



 




 



 



 



 





 



 




 



 
# 546 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc_ex.h"



 



 
# 561 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc_ex.h"



 



 

 


 










 









 











 









 











 









 












 









 



# 690 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc_ex.h"










 









 










 








 











 








 










 








 


# 797 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc_ex.h"










 









 













 










 













 










 













 









 












 







 













 







 











 








 


# 978 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc_ex.h"

# 1001 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc_ex.h"




 




 





 





 





 






 






 






 





 





 









 









 





 





 












 











 










 












 
 


# 1156 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc_ex.h"













 















 

 


# 1200 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc_ex.h"




 



 




 





 






 





 











 




 



 

 


 



 

HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit);
void              HAL_RCCEx_GetPeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit);
uint32_t          HAL_RCCEx_GetPeriphCLKFreq(uint32_t PeriphClk);



 



 

void              HAL_RCCEx_EnableLSECSS(void);
void              HAL_RCCEx_DisableLSECSS(void);
void              HAL_RCCEx_EnableLSECSS_IT(void);
void              HAL_RCCEx_LSECSS_IRQHandler(void);
void              HAL_RCCEx_LSECSS_Callback(void);
void              HAL_RCCEx_EnableLSCO(uint32_t LSCOSource);
void              HAL_RCCEx_DisableLSCO(void);



 



 

void              HAL_RCCEx_CRSConfig(RCC_CRSInitTypeDef *pInit);
void              HAL_RCCEx_CRSSoftwareSynchronizationGenerate(void);
void              HAL_RCCEx_CRSGetSynchronizationInfo(RCC_CRSSynchroInfoTypeDef *pSynchroInfo);
uint32_t          HAL_RCCEx_CRSWaitSynchronization(uint32_t Timeout);
void              HAL_RCCEx_CRS_IRQHandler(void);
void              HAL_RCCEx_CRS_SyncOkCallback(void);
void              HAL_RCCEx_CRS_SyncWarnCallback(void);
void              HAL_RCCEx_CRS_ExpectedSyncCallback(void);
void              HAL_RCCEx_CRS_ErrorCallback(uint32_t Error);



 



 

 


 




# 1409 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc_ex.h"

# 1427 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc_ex.h"

# 1447 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc_ex.h"



















# 1473 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc_ex.h"

# 1482 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc_ex.h"






















# 1512 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc_ex.h"










































# 1560 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc_ex.h"

# 1569 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc_ex.h"
























 



 



 







 
# 3329 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_rcc.h"

 


 




 

 
HAL_StatusTypeDef HAL_RCC_DeInit(void);
HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef *RCC_OscInitStruct);
HAL_StatusTypeDef HAL_RCC_ClockConfig(RCC_ClkInitTypeDef *RCC_ClkInitStruct, uint32_t FLatency);



 



 

 
void              HAL_RCC_MCOConfig(uint32_t RCC_MCOx, uint32_t RCC_MCOSource, uint32_t RCC_MCODiv);
void              HAL_RCC_EnableCSS(void);
void              HAL_RCC_EnableLSECSS(void);
void              HAL_RCC_DisableLSECSS(void);
uint32_t          HAL_RCC_GetSysClockFreq(void);
uint32_t          HAL_RCC_GetHCLKFreq(void);
uint32_t          HAL_RCC_GetPCLK1Freq(void);
uint32_t          HAL_RCC_GetPCLK2Freq(void);
void              HAL_RCC_GetOscConfig(RCC_OscInitTypeDef *RCC_OscInitStruct);
void              HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef *RCC_ClkInitStruct, uint32_t *pFLatency);
 
void              HAL_RCC_NMI_IRQHandler(void);
 
void              HAL_RCC_CSSCallback(void);



 



 



 



 







 
# 212 "..\\..\\User\\stm32g4xx_hal_conf.h"


# 1 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_gpio.h"

















 

 







 
# 30 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_gpio.h"



 




 

 



 


 
typedef struct
{
  uint32_t Pin;        
 

  uint32_t Mode;       
 

  uint32_t Pull;       
 

  uint32_t Speed;      
 

  uint32_t Alternate;  
 
} GPIO_InitTypeDef;



 
typedef enum
{
  GPIO_PIN_RESET = 0U,
  GPIO_PIN_SET
} GPIO_PinState;


 

 


 


 
# 102 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_gpio.h"




 










 
# 130 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_gpio.h"


 




 






 




 





 



 

 


 






 







 







 







 







 




 

 


 





# 231 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_gpio.h"











 

 
# 1 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_gpio_ex.h"

















 


 







 
# 31 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_gpio_ex.h"



 




 

 
 


 



 



 







 
# 72 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_gpio_ex.h"



 
# 98 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_gpio_ex.h"



 
# 120 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_gpio_ex.h"



 
# 135 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_gpio_ex.h"



 
# 152 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_gpio_ex.h"



 
# 171 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_gpio_ex.h"



 
# 187 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_gpio_ex.h"



 
# 213 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_gpio_ex.h"



 
# 227 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_gpio_ex.h"



 
# 241 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_gpio_ex.h"



 
# 253 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_gpio_ex.h"



 
# 268 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_gpio_ex.h"



 







 
# 288 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_gpio_ex.h"



 






 



 

 


 



 
# 318 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_gpio_ex.h"



 



 

 


 



 







 
# 246 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_gpio.h"

 



 




 

 
void              HAL_GPIO_Init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init);
void              HAL_GPIO_DeInit(GPIO_TypeDef  *GPIOx, uint32_t GPIO_Pin);



 




 

 
GPIO_PinState     HAL_GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void              HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
void              HAL_GPIO_TogglePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
HAL_StatusTypeDef HAL_GPIO_LockPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void              HAL_GPIO_EXTI_IRQHandler(uint16_t GPIO_Pin);
void              HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);



 



 



 



 







 
# 216 "..\\..\\User\\stm32g4xx_hal_conf.h"


# 1 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_dma.h"

















 

 







 
# 30 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_dma.h"



 



 

 


 



 
typedef struct
{
  uint32_t Request;                   
 

  uint32_t Direction;                 

 

  uint32_t PeriphInc;                 
 

  uint32_t MemInc;                    
 

  uint32_t PeriphDataAlignment;       
 

  uint32_t MemDataAlignment;          
 

  uint32_t Mode;                      


 

  uint32_t Priority;                  
 
} DMA_InitTypeDef;



 
typedef enum
{
  HAL_DMA_STATE_RESET             = 0x00U,   
  HAL_DMA_STATE_READY             = 0x01U,   
  HAL_DMA_STATE_BUSY              = 0x02U,   
  HAL_DMA_STATE_TIMEOUT           = 0x03U,   
} HAL_DMA_StateTypeDef;



 
typedef enum
{
  HAL_DMA_FULL_TRANSFER      = 0x00U,     
  HAL_DMA_HALF_TRANSFER      = 0x01U      
} HAL_DMA_LevelCompleteTypeDef;




 
typedef enum
{
  HAL_DMA_XFER_CPLT_CB_ID          = 0x00U,     
  HAL_DMA_XFER_HALFCPLT_CB_ID      = 0x01U,     
  HAL_DMA_XFER_ERROR_CB_ID         = 0x02U,     
  HAL_DMA_XFER_ABORT_CB_ID         = 0x03U,     
  HAL_DMA_XFER_ALL_CB_ID           = 0x04U      

} HAL_DMA_CallbackIDTypeDef;



 
typedef struct __DMA_HandleTypeDef
{
  DMA_Channel_TypeDef    *Instance;                                                   

  DMA_InitTypeDef       Init;                                                         

  HAL_LockTypeDef       Lock;                                                         

  volatile HAL_DMA_StateTypeDef  State;                                                   

  void                  *Parent;                                                      

  void (* XferCpltCallback)(struct __DMA_HandleTypeDef *hdma);                        

  void (* XferHalfCpltCallback)(struct __DMA_HandleTypeDef *hdma);                    

  void (* XferErrorCallback)(struct __DMA_HandleTypeDef *hdma);                       

  void (* XferAbortCallback)(struct __DMA_HandleTypeDef *hdma);                       

  volatile uint32_t          ErrorCode;                                                   

  DMA_TypeDef            *DmaBaseAddress;                                             

  uint32_t               ChannelIndex;                                                

  DMAMUX_Channel_TypeDef           *DMAmuxChannel;                                    

  DMAMUX_ChannelStatus_TypeDef     *DMAmuxChannelStatus;                              

  uint32_t                         DMAmuxChannelStatusMask;                           

  DMAMUX_RequestGen_TypeDef        *DMAmuxRequestGen;                                 

  DMAMUX_RequestGenStatus_TypeDef  *DMAmuxRequestGenStatus;                           

  uint32_t                         DMAmuxRequestGenStatusMask;                        

} DMA_HandleTypeDef;


 

 



 



 
# 173 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_dma.h"


 



 















# 201 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_dma.h"

# 212 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_dma.h"

# 219 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_dma.h"

# 226 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_dma.h"




# 240 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_dma.h"









# 256 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_dma.h"

# 264 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_dma.h"







# 277 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_dma.h"







# 292 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_dma.h"











# 310 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_dma.h"









# 328 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_dma.h"

# 335 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_dma.h"




















 



 





 



 




 



 




 



 





 



 





 



 




 



 






 




 





 



 
# 475 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_dma.h"


 



 

 


 




 






 






 



 





 

# 548 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_dma.h"





 
# 585 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_dma.h"





 
# 622 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_dma.h"





 
# 659 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_dma.h"












 
# 679 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_dma.h"












 
# 699 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_dma.h"










 











 











 






 




 

 
# 1 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_dma_ex.h"

















 

 







 
# 30 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_dma_ex.h"



 



 

 


 



 




 
typedef struct
{
  uint32_t SyncSignalID;  
 

  uint32_t SyncPolarity;  
 

  FunctionalState SyncEnable;  
 


  FunctionalState EventEnable;    
 

  uint32_t RequestNumber; 
 


} HAL_DMA_MuxSyncConfigTypeDef;




 
typedef struct
{
  uint32_t SignalID;      
 

  uint32_t Polarity;       
 

  uint32_t RequestNumber;  
 

} HAL_DMA_MuxRequestGeneratorConfigTypeDef;



 

 


 



 
# 123 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_dma_ex.h"



 



 







 



 
# 164 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_dma_ex.h"



 



 







 



 

 

 


 

 


 

 
HAL_StatusTypeDef HAL_DMAEx_ConfigMuxRequestGenerator(DMA_HandleTypeDef *hdma,
                                                      HAL_DMA_MuxRequestGeneratorConfigTypeDef *pRequestGeneratorConfig);
HAL_StatusTypeDef HAL_DMAEx_EnableMuxRequestGenerator(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMAEx_DisableMuxRequestGenerator(DMA_HandleTypeDef *hdma);
 

 
HAL_StatusTypeDef HAL_DMAEx_ConfigMuxSync(DMA_HandleTypeDef *hdma, HAL_DMA_MuxSyncConfigTypeDef *pSyncConfig);
 

void HAL_DMAEx_MUX_IRQHandler(DMA_HandleTypeDef *hdma);



 



 

 



 


























 




 



 







 
# 749 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_dma.h"

 



 



 
 
HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_DeInit(DMA_HandleTypeDef *hdma);


 



 
 
HAL_StatusTypeDef HAL_DMA_Start(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
HAL_StatusTypeDef HAL_DMA_Start_IT(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress,
                                   uint32_t DataLength);
HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_Abort_IT(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_PollForTransfer(DMA_HandleTypeDef *hdma, HAL_DMA_LevelCompleteTypeDef CompleteLevel,
                                          uint32_t Timeout);
void HAL_DMA_IRQHandler(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_RegisterCallback(DMA_HandleTypeDef *hdma, HAL_DMA_CallbackIDTypeDef CallbackID, void (* pCallback)(DMA_HandleTypeDef *_hdma));
HAL_StatusTypeDef HAL_DMA_UnRegisterCallback(DMA_HandleTypeDef *hdma, HAL_DMA_CallbackIDTypeDef CallbackID);



 



 
 
HAL_DMA_StateTypeDef HAL_DMA_GetState(DMA_HandleTypeDef *hdma);
uint32_t             HAL_DMA_GetError(DMA_HandleTypeDef *hdma);


 



 

 


 

































 

 



 



 







 
# 220 "..\\..\\User\\stm32g4xx_hal_conf.h"


# 1 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_cortex.h"

















 
 







 
# 29 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_cortex.h"



 




 

 


 





 
typedef struct
{
  uint8_t                Enable;                
 
  uint8_t                Number;                
 
  uint32_t               BaseAddress;            
  uint8_t                Size;                  
 
  uint8_t                SubRegionDisable;      
 
  uint8_t                TypeExtField;          
 
  uint8_t                AccessPermission;      
 
  uint8_t                DisableExec;           
 
  uint8_t                IsShareable;           
 
  uint8_t                IsCacheable;           
 
  uint8_t                IsBufferable;          
 
}MPU_Region_InitTypeDef;


 




 

 



 



 
# 101 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_cortex.h"


 



 





 




 






 



 




 



 




 



 




 



 




 



 




 



 






 



 
# 214 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_cortex.h"


 



 
# 227 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_cortex.h"


 



 
# 242 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_cortex.h"


 




 

 


 



 

 


 




 
 
void HAL_NVIC_SetPriorityGrouping(uint32_t PriorityGroup);
void HAL_NVIC_SetPriority(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority);
void HAL_NVIC_EnableIRQ(IRQn_Type IRQn);
void HAL_NVIC_DisableIRQ(IRQn_Type IRQn);
void HAL_NVIC_SystemReset(void);
uint32_t HAL_SYSTICK_Config(uint32_t TicksNumb);



 




 
 
uint32_t HAL_NVIC_GetPriorityGrouping(void);
void HAL_NVIC_GetPriority(IRQn_Type IRQn, uint32_t PriorityGroup, uint32_t* pPreemptPriority, uint32_t* pSubPriority);
uint32_t HAL_NVIC_GetPendingIRQ(IRQn_Type IRQn);
void HAL_NVIC_SetPendingIRQ(IRQn_Type IRQn);
void HAL_NVIC_ClearPendingIRQ(IRQn_Type IRQn);
uint32_t HAL_NVIC_GetActive(IRQn_Type IRQn);
void HAL_SYSTICK_CLKSourceConfig(uint32_t CLKSource);
void HAL_SYSTICK_IRQHandler(void);
void HAL_SYSTICK_Callback(void);


void HAL_MPU_Enable(uint32_t MPU_Control);
void HAL_MPU_Disable(void);
void HAL_MPU_ConfigRegion(MPU_Region_InitTypeDef *MPU_Init);



 



 

  
 
 
 


 




































# 358 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_cortex.h"

# 367 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_cortex.h"

# 396 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_cortex.h"






 

 



 



 








 
# 224 "..\\..\\User\\stm32g4xx_hal_conf.h"


# 1 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_adc.h"

















 

 







 
# 30 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_adc.h"

 
# 1 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_ll_adc.h"

















 

 







 
# 30 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_ll_adc.h"



 





 

 
 

 


 

 
 
 
 

 
 










 
 
# 84 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_ll_adc.h"



 
 
 
 

 
 










 
 







 
 
 
 


 
 
 





 
 
 





 





 
 
 
 


 
 
 





 
 
 





 








 
 
 
 
 
 
 
 





 


 




 
 








 
 
# 225 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_ll_adc.h"

 
 
# 247 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_ll_adc.h"

 
 
# 269 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_ll_adc.h"


 
 
 
 
 
 
# 284 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_ll_adc.h"

 
 
 
 
 
 
 
 

 




 
 











 
# 321 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_ll_adc.h"

 
 
# 330 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_ll_adc.h"


 







 



 
 


 








 


 


 








 





 


 
# 578 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_ll_adc.h"

 


 




 
# 623 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_ll_adc.h"


 




 
# 642 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_ll_adc.h"


 



 
 
 
 






 



 
# 678 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_ll_adc.h"


 



 
 
 
 
 
 






 



 






 



 




 



 




 



 






 



 




 



 




 



 




 


 





 



 
# 807 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_ll_adc.h"


 



 
# 921 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_ll_adc.h"


 



 





 



 
# 944 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_ll_adc.h"


 



 




 



 





 




 




 




 




 



 
# 1006 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_ll_adc.h"


 



 
# 1022 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_ll_adc.h"


 



 
# 1045 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_ll_adc.h"


 



 
# 1155 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_ll_adc.h"


 



 





 



 




 



 





 



 






 



 




 



 






 



 
# 1230 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_ll_adc.h"


 



 





 



 





 



 
# 1351 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_ll_adc.h"


 



 





 



 
# 1376 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_ll_adc.h"


 



 







 



 




 



 
# 1412 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_ll_adc.h"


 



 
# 1428 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_ll_adc.h"


 




 
# 1444 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_ll_adc.h"


 



 







 



 
# 1475 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_ll_adc.h"


 



 





 










 

 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 

 
 
 
 


 
 
 
 


 
 
 
 


 
 
 
 
 
 
 




 



 


 


 



 







 







 



 



 





















































 
# 1650 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_ll_adc.h"





















































 
# 1718 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_ll_adc.h"






























































 














































































 








































 
# 2015 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_ll_adc.h"




































































































































































 
# 2189 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_ll_adc.h"




















 





















 














 















 
















 
















 
# 2320 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_ll_adc.h"










 
# 2345 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_ll_adc.h"
















 
# 2392 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_ll_adc.h"













 




















 
# 2434 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_ll_adc.h"
















 
# 2457 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_ll_adc.h"

























 
# 2490 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_ll_adc.h"













































 
# 2549 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_ll_adc.h"












































 
# 2610 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_ll_adc.h"



 



 


 


 



 
 
 
 






























 

static __inline uint32_t LL_ADC_DMA_GetRegAddr(ADC_TypeDef *ADCx, uint32_t Register)
{
  uint32_t data_reg_addr;

  if (Register == (0x00000000UL))
  {
     
    data_reg_addr = (uint32_t) &(ADCx->DR);
  }
  else  
  {
     
    data_reg_addr = (uint32_t) &(((((ADC_Common_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x08000300UL))))->CDR);
  }

  return data_reg_addr;
}
# 2690 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_ll_adc.h"



 



 


































 
static __inline void LL_ADC_SetCommonClock(ADC_Common_TypeDef *ADCxy_COMMON, uint32_t CommonClock)
{
  (((ADCxy_COMMON->CCR)) = ((((((ADCxy_COMMON->CCR))) & (~((0x3UL << (16U)) | (0xFUL << (18U))))) | (CommonClock))));
}























 
static __inline uint32_t LL_ADC_GetCommonClock(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (uint32_t)(((ADCxy_COMMON->CCR) & ((0x3UL << (16U)) | (0xFUL << (18U)))));
}































 
static __inline void LL_ADC_SetCommonPathInternalCh(ADC_Common_TypeDef *ADCxy_COMMON, uint32_t PathInternal)
{
  (((ADCxy_COMMON->CCR)) = ((((((ADCxy_COMMON->CCR))) & (~((0x1UL << (22U)) | (0x1UL << (23U)) | (0x1UL << (24U))))) | (PathInternal))));
}






























 
static __inline void LL_ADC_SetCommonPathInternalChAdd(ADC_Common_TypeDef *ADCxy_COMMON, uint32_t PathInternal)
{
  ((ADCxy_COMMON->CCR) |= (PathInternal));
}



















 
static __inline void LL_ADC_SetCommonPathInternalChRem(ADC_Common_TypeDef *ADCxy_COMMON, uint32_t PathInternal)
{
  ((ADCxy_COMMON->CCR) &= ~(PathInternal));
}

















 
static __inline uint32_t LL_ADC_GetCommonPathInternalCh(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (uint32_t)(((ADCxy_COMMON->CCR) & ((0x1UL << (22U)) | (0x1UL << (23U)) | (0x1UL << (24U)))));
}



 



 































 
static __inline void LL_ADC_SetCalibrationFactor(ADC_TypeDef *ADCx, uint32_t SingleDiff, uint32_t CalibrationFactor)
{
  (((ADCx->CALFACT)) = ((((((ADCx->CALFACT))) & (~(SingleDiff & ((0x7FUL << (16U)) | (0x7FUL << (0U)))))) | (CalibrationFactor << (((SingleDiff & (0x00010000UL)) >> ((16UL) - 4UL)) & ~(SingleDiff & (0x7FUL << (0U))))))));


}
















 
static __inline uint32_t LL_ADC_GetCalibrationFactor(ADC_TypeDef *ADCx, uint32_t SingleDiff)
{
   
   
   
   
  return (uint32_t)(((ADCx->CALFACT) & ((SingleDiff & ((0x7FUL << (16U)) | (0x7FUL << (0U)))))) >> ((SingleDiff & (0x00010000UL)) >>

                                                                                  ((16UL) - 4UL)));
}

















 
static __inline void LL_ADC_SetResolution(ADC_TypeDef *ADCx, uint32_t Resolution)
{
  (((ADCx->CFGR)) = ((((((ADCx->CFGR))) & (~((0x3UL << (3U))))) | (Resolution))));
}












 
static __inline uint32_t LL_ADC_GetResolution(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR) & ((0x3UL << (3U)))));
}















 
static __inline void LL_ADC_SetDataAlignment(ADC_TypeDef *ADCx, uint32_t DataAlignment)
{
  (((ADCx->CFGR)) = ((((((ADCx->CFGR))) & (~((0x1UL << (15U))))) | (DataAlignment))));
}










 
static __inline uint32_t LL_ADC_GetDataAlignment(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR) & ((0x1UL << (15U)))));
}



















































 
static __inline void LL_ADC_SetLowPowerMode(ADC_TypeDef *ADCx, uint32_t LowPowerMode)
{
  (((ADCx->CFGR)) = ((((((ADCx->CFGR))) & (~((0x1UL << (14U))))) | (LowPowerMode))));
}














































 
static __inline uint32_t LL_ADC_GetLowPowerMode(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR) & ((0x1UL << (14U)))));
}




















































































 
static __inline void LL_ADC_SetOffset(ADC_TypeDef *ADCx, uint32_t Offsety, uint32_t Channel, uint32_t OffsetLevel)
{
  volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->OFR1)) + ((Offsety) << 2UL))));

  (((*preg)) = ((((((*preg))) & (~((0x1UL << (31U)) | (0x1FUL << (26U)) | (0xFFFUL << (0U))))) | ((0x1UL << (31U)) | (Channel & ((0x1FUL << (26U)))) | OffsetLevel))));


}










































































 
static __inline uint32_t LL_ADC_GetOffsetChannel(ADC_TypeDef *ADCx, uint32_t Offsety)
{
  const volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->OFR1)) + ((Offsety) << 2UL))));

  return (uint32_t) ((*preg) & ((0x1FUL << (26U))));
}



















 
static __inline uint32_t LL_ADC_GetOffsetLevel(ADC_TypeDef *ADCx, uint32_t Offsety)
{
  const volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->OFR1)) + ((Offsety) << 2UL))));

  return (uint32_t) ((*preg) & ((0xFFFUL << (0U))));
}


























 
static __inline void LL_ADC_SetOffsetState(ADC_TypeDef *ADCx, uint32_t Offsety, uint32_t OffsetState)
{
  volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->OFR1)) + ((Offsety) << 2UL))));

  (((*preg)) = ((((((*preg))) & (~((0x1UL << (31U))))) | (OffsetState))));


}

















 
static __inline uint32_t LL_ADC_GetOffsetState(ADC_TypeDef *ADCx, uint32_t Offsety)
{
  const volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->OFR1)) + ((Offsety) << 2UL))));

  return (uint32_t) ((*preg) & ((0x1UL << (31U))));
}






















 
static __inline void LL_ADC_SetOffsetSign(ADC_TypeDef *ADCx, uint32_t Offsety, uint32_t OffsetSign)
{
  volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->OFR1)) + ((Offsety) << 2UL))));

  (((*preg)) = ((((((*preg))) & (~((0x1UL << (24U))))) | (OffsetSign))));


}

















 
static __inline uint32_t LL_ADC_GetOffsetSign(ADC_TypeDef *ADCx, uint32_t Offsety)
{
  const volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->OFR1)) + ((Offsety) << 2UL))));

  return (uint32_t) ((*preg) & ((0x1UL << (24U))));
}






















 
static __inline void LL_ADC_SetOffsetSaturation(ADC_TypeDef *ADCx, uint32_t Offsety, uint32_t OffsetSaturation)
{
  volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->OFR1)) + ((Offsety) << 2UL))));

  (((*preg)) = ((((((*preg))) & (~((0x1UL << (25U))))) | (OffsetSaturation))));


}

















 
static __inline uint32_t LL_ADC_GetOffsetSaturation(ADC_TypeDef *ADCx, uint32_t Offsety)
{
  const volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->OFR1)) + ((Offsety) << 2UL))));

  return (uint32_t) ((*preg) & ((0x1UL << (25U))));
}




















 
static __inline void LL_ADC_SetGainCompensation(ADC_TypeDef *ADCx, uint32_t GainCompensation)
{
  (((ADCx->GCOMP)) = ((((((ADCx->GCOMP))) & (~((0x3FFFUL << (0U))))) | (GainCompensation))));
  (((ADCx->CFGR2)) = ((((((ADCx->CFGR2))) & (~((0x1UL << (16U))))) | (((GainCompensation == 0UL) ? 0UL : 1UL) << (16U)))));
}









 
static __inline uint32_t LL_ADC_GetGainCompensation(ADC_TypeDef *ADCx)
{
  return ((((ADCx->CFGR2) & ((0x1UL << (16U)))) == (0x1UL << (16U))) ? ((ADCx->GCOMP) & ((0x3FFFUL << (0U)))) : 0UL);
}















 
static __inline void LL_ADC_SetSamplingTimeCommonConfig(ADC_TypeDef *ADCx, uint32_t SamplingTimeCommonConfig)
{
  (((ADCx->SMPR1)) = ((((((ADCx->SMPR1))) & (~((0x1UL << (31U))))) | (SamplingTimeCommonConfig))));
}









 
static __inline uint32_t LL_ADC_GetSamplingTimeCommonConfig(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->SMPR1) & ((0x1UL << (31U)))));
}




 



 


































































 
static __inline void LL_ADC_REG_SetTriggerSource(ADC_TypeDef *ADCx, uint32_t TriggerSource)
{
  (((ADCx->CFGR)) = ((((((ADCx->CFGR))) & (~((0x3UL << (10U)) | (0x1FUL << (5U))))) | (TriggerSource))));
}





























































 
static __inline uint32_t LL_ADC_REG_GetTriggerSource(ADC_TypeDef *ADCx)
{
  volatile uint32_t TriggerSource = ((ADCx->CFGR) & ((0x1FUL << (5U)) | (0x3UL << (10U))));

   
   
  uint32_t ShiftExten = ((TriggerSource & (0x3UL << (10U))) >> (((10U)) - 2UL));

   
   
  return ((TriggerSource
           & (((((0x00000000UL) & (0x1FUL << (5U))) << (4U * 0UL)) | (((0x1FUL << (5U))) << (4U * 1UL)) | (((0x1FUL << (5U))) << (4U * 2UL)) | (((0x1FUL << (5U))) << (4U * 3UL)) ) >> ShiftExten) & (0x1FUL << (5U)))
          | ((((((0x00000000UL) & (0x3UL << (10U))) << (4U * 0UL)) | ((((0x1UL << (10U)))) << (4U * 1UL)) | ((((0x1UL << (10U)))) << (4U * 2UL)) | ((((0x1UL << (10U)))) << (4U * 3UL)) ) >> ShiftExten) & (0x3UL << (10U)))
         );
}











 
static __inline uint32_t LL_ADC_REG_IsTriggerSourceSWStart(ADC_TypeDef *ADCx)
{
  return ((((ADCx->CFGR) & ((0x3UL << (10U)))) == ((0x00000000UL) & (0x3UL << (10U)))) ? 1UL : 0UL);
}















 
static __inline void LL_ADC_REG_SetTriggerEdge(ADC_TypeDef *ADCx, uint32_t ExternalTriggerEdge)
{
  (((ADCx->CFGR)) = ((((((ADCx->CFGR))) & (~((0x3UL << (10U))))) | (ExternalTriggerEdge))));
}










 
static __inline uint32_t LL_ADC_REG_GetTriggerEdge(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR) & ((0x3UL << (10U)))));
}


















 
static __inline void LL_ADC_REG_SetSamplingMode(ADC_TypeDef *ADCx, uint32_t SamplingMode)
{
  (((ADCx->CFGR2)) = ((((((ADCx->CFGR2))) & (~((0x1UL << (26U)) | (0x1UL << (27U))))) | (SamplingMode))));
}










 
static __inline uint32_t LL_ADC_REG_GetSamplingMode(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR2) & ((0x1UL << (26U)) | (0x1UL << (27U)))));
}






















































 
static __inline void LL_ADC_REG_SetSequencerLength(ADC_TypeDef *ADCx, uint32_t SequencerNbRanks)
{
  (((ADCx->SQR1)) = ((((((ADCx->SQR1))) & (~((0xFUL << (0U))))) | (SequencerNbRanks))));
}

















































 
static __inline uint32_t LL_ADC_REG_GetSequencerLength(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->SQR1) & ((0xFUL << (0U)))));
}



























 
static __inline void LL_ADC_REG_SetSequencerDiscont(ADC_TypeDef *ADCx, uint32_t SeqDiscont)
{
  (((ADCx->CFGR)) = ((((((ADCx->CFGR))) & (~((0x1UL << (16U)) | (0x7UL << (17U))))) | (SeqDiscont))));
}


















 
static __inline uint32_t LL_ADC_REG_GetSequencerDiscont(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR) & ((0x1UL << (16U)) | (0x7UL << (17U)))));
}


































































































 
static __inline void LL_ADC_REG_SetSequencerRanks(ADC_TypeDef *ADCx, uint32_t Rank, uint32_t Channel)
{
   
   
   
   
  volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->SQR1)) + ((((Rank & ((0x00000000UL) | (0x00000100UL) | (0x00000200UL) | (0x00000300UL))) >> (8UL))) << 2UL))));

  (((*preg)) = ((((((*preg))) & (~(((0x1FUL << (0U))) << (Rank & (((0x1FUL << (0U)))))))) | (((Channel & ((0x1FUL << (26U)))) >> ((26U))) << (Rank & (((0x1FUL << (0U)))))))));


}




































































































 
static __inline uint32_t LL_ADC_REG_GetSequencerRanks(ADC_TypeDef *ADCx, uint32_t Rank)
{
  const volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->SQR1)) + ((((Rank & ((0x00000000UL) | (0x00000100UL) | (0x00000200UL) | (0x00000300UL))) >> (8UL))) << 2UL))));

  return (uint32_t)((((*preg) & (((0x1FUL << (0U))) << (Rank & (((0x1FUL << (0U)))))))

                     >> (Rank & (((0x1FUL << (0U)))))) << ((26U))
                   );
}



















 
static __inline void LL_ADC_REG_SetContinuousMode(ADC_TypeDef *ADCx, uint32_t Continuous)
{
  (((ADCx->CFGR)) = ((((((ADCx->CFGR))) & (~((0x1UL << (13U))))) | (Continuous))));
}












 
static __inline uint32_t LL_ADC_REG_GetContinuousMode(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR) & ((0x1UL << (13U)))));
}



































 
static __inline void LL_ADC_REG_SetDMATransfer(ADC_TypeDef *ADCx, uint32_t DMATransfer)
{
  (((ADCx->CFGR)) = ((((((ADCx->CFGR))) & (~((0x1UL << (0U)) | (0x1UL << (1U))))) | (DMATransfer))));
}






























 
static __inline uint32_t LL_ADC_REG_GetDMATransfer(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR) & ((0x1UL << (0U)) | (0x1UL << (1U)))));
}




















 
static __inline void LL_ADC_REG_SetOverrun(ADC_TypeDef *ADCx, uint32_t Overrun)
{
  (((ADCx->CFGR)) = ((((((ADCx->CFGR))) & (~((0x1UL << (12U))))) | (Overrun))));
}









 
static __inline uint32_t LL_ADC_REG_GetOverrun(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR) & ((0x1UL << (12U)))));
}



 



 

































































 
static __inline void LL_ADC_INJ_SetTriggerSource(ADC_TypeDef *ADCx, uint32_t TriggerSource)
{
  (((ADCx->JSQR)) = ((((((ADCx->JSQR))) & (~((0x1FUL << (2U)) | (0x3UL << (7U))))) | (TriggerSource))));
}




























































 
static __inline uint32_t LL_ADC_INJ_GetTriggerSource(ADC_TypeDef *ADCx)
{
  volatile uint32_t TriggerSource = ((ADCx->JSQR) & ((0x1FUL << (2U)) | (0x3UL << (7U))));

   
   
  uint32_t ShiftJexten = ((TriggerSource & (0x3UL << (7U))) >> (((7U)) - 2UL));

   
   
  return ((TriggerSource
           & (((((0x00000000UL) & (0x1FUL << (2U))) << (4U * 0UL)) | (((0x1FUL << (2U))) << (4U * 1UL)) | (((0x1FUL << (2U))) << (4U * 2UL)) | (((0x1FUL << (2U))) << (4U * 3UL)) ) >> ShiftJexten) & (0x1FUL << (2U)))
          | ((((((0x00000000UL) & (0x3UL << (7U))) << (4U * 0UL)) | ((((0x1UL << (7U)))) << (4U * 1UL)) | ((((0x1UL << (7U)))) << (4U * 2UL)) | ((((0x1UL << (7U)))) << (4U * 3UL)) ) >> ShiftJexten) & (0x3UL << (7U)))
         );
}











 
static __inline uint32_t LL_ADC_INJ_IsTriggerSourceSWStart(ADC_TypeDef *ADCx)
{
  return ((((ADCx->JSQR) & ((0x3UL << (7U)))) == ((0x00000000UL) & (0x3UL << (7U)))) ? 1UL : 0UL);
}















 
static __inline void LL_ADC_INJ_SetTriggerEdge(ADC_TypeDef *ADCx, uint32_t ExternalTriggerEdge)
{
  (((ADCx->JSQR)) = ((((((ADCx->JSQR))) & (~((0x3UL << (7U))))) | (ExternalTriggerEdge))));
}










 
static __inline uint32_t LL_ADC_INJ_GetTriggerEdge(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->JSQR) & ((0x3UL << (7U)))));
}





















 
static __inline void LL_ADC_INJ_SetSequencerLength(ADC_TypeDef *ADCx, uint32_t SequencerNbRanks)
{
  (((ADCx->JSQR)) = ((((((ADCx->JSQR))) & (~((0x3UL << (0U))))) | (SequencerNbRanks))));
}
















 
static __inline uint32_t LL_ADC_INJ_GetSequencerLength(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->JSQR) & ((0x3UL << (0U)))));
}













 
static __inline void LL_ADC_INJ_SetSequencerDiscont(ADC_TypeDef *ADCx, uint32_t SeqDiscont)
{
  (((ADCx->CFGR)) = ((((((ADCx->CFGR))) & (~((0x1UL << (20U))))) | (SeqDiscont))));
}










 
static __inline uint32_t LL_ADC_INJ_GetSequencerDiscont(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR) & ((0x1UL << (20U)))));
}





































































 
static __inline void LL_ADC_INJ_SetSequencerRanks(ADC_TypeDef *ADCx, uint32_t Rank, uint32_t Channel)
{
   
   
   
   
  (((ADCx->JSQR)) = ((((((ADCx->JSQR))) & (~((((0x1FUL << (26U))) >> ((26U))) << (Rank & (((0x1FUL << (0U)))))))) | (((Channel & ((0x1FUL << (26U)))) >> ((26U))) << (Rank & (((0x1FUL << (0U)))))))));


}








































































 
static __inline uint32_t LL_ADC_INJ_GetSequencerRanks(ADC_TypeDef *ADCx, uint32_t Rank)
{
  return (uint32_t)((((ADCx->JSQR) & ((((0x1FUL << (26U))) >> ((26U))) << (Rank & (((0x1FUL << (0U)))))))

                     >> (Rank & (((0x1FUL << (0U)))))) << ((26U))
                   );
}






























 
static __inline void LL_ADC_INJ_SetTrigAuto(ADC_TypeDef *ADCx, uint32_t TrigAuto)
{
  (((ADCx->CFGR)) = ((((((ADCx->CFGR))) & (~((0x1UL << (25U))))) | (TrigAuto))));
}









 
static __inline uint32_t LL_ADC_INJ_GetTrigAuto(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR) & ((0x1UL << (25U)))));
}









































 
static __inline void LL_ADC_INJ_SetQueueMode(ADC_TypeDef *ADCx, uint32_t QueueMode)
{
  (((ADCx->CFGR)) = ((((((ADCx->CFGR))) & (~((0x1UL << (21U)) | (0x1UL << (31U))))) | (QueueMode))));
}










 
static __inline uint32_t LL_ADC_INJ_GetQueueMode(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR) & ((0x1UL << (21U)) | (0x1UL << (31U)))));
}





































































































































































































































































 
static __inline void LL_ADC_INJ_ConfigQueueContext(ADC_TypeDef *ADCx,
                                                   uint32_t TriggerSource,
                                                   uint32_t ExternalTriggerEdge,
                                                   uint32_t SequencerNbRanks,
                                                   uint32_t Rank1_Channel,
                                                   uint32_t Rank2_Channel,
                                                   uint32_t Rank3_Channel,
                                                   uint32_t Rank4_Channel)
{
   
   
   
   
   
   
  uint32_t is_trigger_not_sw = (uint32_t)((TriggerSource != (0x00000000UL)) ? 1UL : 0UL);
  (((ADCx->JSQR)) = ((((((ADCx->JSQR))) & (~((0x1FUL << (2U)) | (0x3UL << (7U)) | (0x1FUL << (27U)) | (0x1FUL << (21U)) | (0x1FUL << (15U)) | (0x1FUL << (9U)) | (0x3UL << (0U))))) | ((TriggerSource & (0x1FUL << (2U))) | (ExternalTriggerEdge * (is_trigger_not_sw)) | (((Rank4_Channel & ((0x1FUL << (26U)))) >> ((26U))) << (((0x00000300UL) | ((27U))) & (((0x1FUL << (0U)))))) | (((Rank3_Channel & ((0x1FUL << (26U)))) >> ((26U))) << (((0x00000200UL) | ((21U))) & (((0x1FUL << (0U)))))) | (((Rank2_Channel & ((0x1FUL << (26U)))) >> ((26U))) << (((0x00000100UL) | ((15U))) & (((0x1FUL << (0U)))))) | (((Rank1_Channel & ((0x1FUL << (26U)))) >> ((26U))) << (((0x00000000UL) | ((9U))) & (((0x1FUL << (0U)))))) | SequencerNbRanks))));
# 5218 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_ll_adc.h"
}



 



 







































































































 
static __inline void LL_ADC_SetChannelSamplingTime(ADC_TypeDef *ADCx, uint32_t Channel, uint32_t SamplingTime)
{
   
   
   
   
  volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->SMPR1)) + ((((Channel & ((0x00000000UL) | (0x02000000UL))) >> (25UL))) << 2UL))));

  (((*preg)) = ((((((*preg))) & (~((0x7UL << (0U)) << ((Channel & (0x01F00000UL)) >> (20UL))))) | (SamplingTime << ((Channel & (0x01F00000UL)) >> (20UL))))));


}























































































 
static __inline uint32_t LL_ADC_GetChannelSamplingTime(ADC_TypeDef *ADCx, uint32_t Channel)
{
  const volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->SMPR1)) + ((((Channel & ((0x00000000UL) | (0x02000000UL))) >> (25UL))) << 2UL))));

  return (uint32_t)(((*preg) & ((0x7UL << (0U)) << ((Channel & (0x01F00000UL)) >> (20UL))))

                    >> ((Channel & (0x01F00000UL)) >> (20UL))
                   );
}























































 
static __inline void LL_ADC_SetChannelSingleDiff(ADC_TypeDef *ADCx, uint32_t Channel, uint32_t SingleDiff)
{
   
   
   
  (((ADCx->DIFSEL)) = ((((((ADCx->DIFSEL))) & (~(Channel & (((0x7FFFFUL << (0U))))))) | ((Channel & (((0x7FFFFUL << (0U))))) & ((0x7FFFFUL << (0U)) >> (SingleDiff & ((0x10UL << (0U)) | (0x08UL << (0U)))))))));


}















































 
static __inline uint32_t LL_ADC_GetChannelSingleDiff(ADC_TypeDef *ADCx, uint32_t Channel)
{
  return (uint32_t)(((ADCx->DIFSEL) & ((Channel & (((0x7FFFFUL << (0U))))))));
}



 



 
























































































































































 
static __inline void LL_ADC_SetAnalogWDMonitChannels(ADC_TypeDef *ADCx, uint32_t AWDy, uint32_t AWDChannelGroup)
{
   
   
   
   
  volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->CFGR)) + ((((AWDy & ((0x00000000UL) | (0x00100000UL) | (0x00200000UL))) >> (20UL)) + ((AWDy & ((0x00001UL << (0U)))) * (0x00000024UL))) << 2UL))));


  (((*preg)) = ((((((*preg))) & (~((AWDy & (((0x1FUL << (26U)) | (0x1UL << (24U)) | (0x1UL << (23U)) | (0x1UL << (22U))) | ((0x7FFFFUL << (0U)))))))) | (AWDChannelGroup & AWDy))));


}


























































































































 
static __inline uint32_t LL_ADC_GetAnalogWDMonitChannels(ADC_TypeDef *ADCx, uint32_t AWDy)
{
  const volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->CFGR)) + ((((AWDy & ((0x00000000UL) | (0x00100000UL) | (0x00200000UL))) >> (20UL)) + ((AWDy & ((0x00001UL << (0U)))) * (0x00000024UL))) << 2UL))));


  uint32_t AnalogWDMonitChannels = (((*preg) & (AWDy)) & AWDy & (((0x1FUL << (26U)) | (0x1UL << (24U)) | (0x1UL << (23U)) | (0x1UL << (22U))) | ((0x7FFFFUL << (0U)))));

   
   
   
   
  if (AnalogWDMonitChannels != 0UL)
  {
    if (AWDy == (((0x1FUL << (26U)) | (0x1UL << (24U)) | (0x1UL << (23U)) | (0x1UL << (22U))) | (0x00000000UL)))
    {
      if ((AnalogWDMonitChannels & (0x1UL << (22U))) == 0UL)
      {
         
        AnalogWDMonitChannels = ((AnalogWDMonitChannels
                                  | (((0x7FFFFUL << (0U))))
                                 )
                                 & (~((0x1FUL << (26U))))
                                );
      }
      else
      {
         
        AnalogWDMonitChannels = (AnalogWDMonitChannels
                                 | ((0x00001UL << (0U)) << (AnalogWDMonitChannels >> (26U)))
                                );
      }
    }
    else
    {
      if ((AnalogWDMonitChannels & ((0x7FFFFUL << (0U)))) == ((0x7FFFFUL << (0U))))
      {
         
        AnalogWDMonitChannels = (((0x7FFFFUL << (0U)))
                                 | (((0x1UL << (24U)) | (0x1UL << (23U))))
                                );
      }
      else
      {
         
         
        AnalogWDMonitChannels = (AnalogWDMonitChannels
                                 | ((0x1UL << (24U)) | (0x1UL << (23U)) | (0x1UL << (22U)))
                                 | (((((AnalogWDMonitChannels) & ((0x7FFFFUL << (0U)))) == 0UL) ? ( ((AnalogWDMonitChannels) & ((0x1FUL << (26U)))) >> ((26U)) ) : ( (uint32_t)(__clz(__rbit((AnalogWDMonitChannels)))) ) ) << (26U))
                                );
      }
    }
  }

  return AnalogWDMonitChannels;
}
















































 
static __inline void LL_ADC_ConfigAnalogWDThresholds(ADC_TypeDef *ADCx, uint32_t AWDy, uint32_t AWDThresholdHighValue,
                                                     uint32_t AWDThresholdLowValue)
{
   
   
   
   
   
  volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->TR1)) + ((((AWDy & (((0x00000000UL)) | ((0x00100000UL)) | ((0x00200000UL)))) >> ((20UL)))) << 2UL))));

  (((*preg)) = ((((((*preg))) & (~((0xFFFUL << (16U)) | (0xFFFUL << (0U))))) | ((AWDThresholdHighValue << ((16U))) | AWDThresholdLowValue))));


}






















































 
static __inline void LL_ADC_SetAnalogWDThresholds(ADC_TypeDef *ADCx, uint32_t AWDy, uint32_t AWDThresholdsHighLow,
                                                  uint32_t AWDThresholdValue)
{
   
   
   
   
   
  volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->TR1)) + ((((AWDy & (((0x00000000UL)) | ((0x00100000UL)) | ((0x00200000UL)))) >> ((20UL)))) << 2UL))));


  (((*preg)) = ((((((*preg))) & (~(AWDThresholdsHighLow))) | (AWDThresholdValue << ((AWDThresholdsHighLow & (0x00010000UL)) >> ((16UL) - 4UL))))));


}




























 
static __inline uint32_t LL_ADC_GetAnalogWDThresholds(ADC_TypeDef *ADCx, uint32_t AWDy, uint32_t AWDThresholdsHighLow)
{
  const volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->TR1)) + ((((AWDy & (((0x00000000UL)) | ((0x00100000UL)) | ((0x00200000UL)))) >> ((20UL)))) << 2UL))));


  return (uint32_t)(((*preg) & ((AWDThresholdsHighLow | (0xFFFUL << (0U)))))

                    >> (((AWDThresholdsHighLow & (0x00010000UL)) >> ((16UL) - 4UL))
                        & ~(AWDThresholdsHighLow & (0xFFFUL << (0U)))));
}























 
static __inline void LL_ADC_SetAWDFilteringConfiguration(ADC_TypeDef *ADCx, uint32_t AWDy, uint32_t FilteringConfig)
{
   
  (void)(AWDy);
  (((ADCx->TR1)) = ((((((ADCx->TR1))) & (~((0x7UL << (12U))))) | (FilteringConfig))));
}


















 
static __inline uint32_t LL_ADC_GetAWDFilteringConfiguration(ADC_TypeDef *ADCx, uint32_t AWDy)
{
   
  (void)(AWDy);
  return (uint32_t)(((ADCx->TR1) & ((0x7UL << (12U)))));
}



 



 

























 
static __inline void LL_ADC_SetOverSamplingScope(ADC_TypeDef *ADCx, uint32_t OvsScope)
{
  (((ADCx->CFGR2)) = ((((((ADCx->CFGR2))) & (~((0x1UL << (0U)) | (0x1UL << (1U)) | (0x1UL << (10U))))) | (OvsScope))));
}




















 
static __inline uint32_t LL_ADC_GetOverSamplingScope(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR2) & ((0x1UL << (0U)) | (0x1UL << (1U)) | (0x1UL << (10U)))));
}






















 
static __inline void LL_ADC_SetOverSamplingDiscont(ADC_TypeDef *ADCx, uint32_t OverSamplingDiscont)
{
  (((ADCx->CFGR2)) = ((((((ADCx->CFGR2))) & (~((0x1UL << (9U))))) | (OverSamplingDiscont))));
}














 
static __inline uint32_t LL_ADC_GetOverSamplingDiscont(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR2) & ((0x1UL << (9U)))));
}


































 
static __inline void LL_ADC_ConfigOverSamplingRatioShift(ADC_TypeDef *ADCx, uint32_t Ratio, uint32_t Shift)
{
  (((ADCx->CFGR2)) = ((((((ADCx->CFGR2))) & (~(((0xFUL << (5U)) | (0x7UL << (2U)))))) | ((Shift | Ratio)))));
}















 
static __inline uint32_t LL_ADC_GetOverSamplingRatio(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR2) & ((0x7UL << (2U)))));
}
















 
static __inline uint32_t LL_ADC_GetOverSamplingShift(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR2) & ((0xFUL << (5U)))));
}



 



 



























 
static __inline void LL_ADC_SetMultimode(ADC_Common_TypeDef *ADCxy_COMMON, uint32_t Multimode)
{
  (((ADCxy_COMMON->CCR)) = ((((((ADCxy_COMMON->CCR))) & (~((0x1FUL << (0U))))) | (Multimode))));
}



















 
static __inline uint32_t LL_ADC_GetMultimode(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (uint32_t)(((ADCxy_COMMON->CCR) & ((0x1FUL << (0U)))));
}














































 
static __inline void LL_ADC_SetMultiDMATransfer(ADC_Common_TypeDef *ADCxy_COMMON, uint32_t MultiDMATransfer)
{
  (((ADCxy_COMMON->CCR)) = ((((((ADCxy_COMMON->CCR))) & (~((0x3UL << (14U)) | (0x1UL << (13U))))) | (MultiDMATransfer))));
}









































 
static __inline uint32_t LL_ADC_GetMultiDMATransfer(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (uint32_t)(((ADCxy_COMMON->CCR) & ((0x3UL << (14U)) | (0x1UL << (13U)))));
}



































 
static __inline void LL_ADC_SetMultiTwoSamplingDelay(ADC_Common_TypeDef *ADCxy_COMMON, uint32_t MultiTwoSamplingDelay)
{
  (((ADCxy_COMMON->CCR)) = ((((((ADCxy_COMMON->CCR))) & (~((0xFUL << (8U))))) | (MultiTwoSamplingDelay))));
}























 
static __inline uint32_t LL_ADC_GetMultiTwoSamplingDelay(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (uint32_t)(((ADCxy_COMMON->CCR) & ((0xFUL << (8U)))));
}




 


 













 
static __inline void LL_ADC_EnableDeepPowerDown(ADC_TypeDef *ADCx)
{
   
   
   
  (((ADCx->CR)) = ((((((ADCx->CR))) & (~(((0x1UL << (31U)) | (0x1UL << (5U)) | (0x1UL << (4U)) | (0x1UL << (3U)) | (0x1UL << (2U)) | (0x1UL << (1U)) | (0x1UL << (0U)))))) | ((0x1UL << (29U))))));


}













 
static __inline void LL_ADC_DisableDeepPowerDown(ADC_TypeDef *ADCx)
{
   
   
   
  ((ADCx->CR) &= ~(((0x1UL << (29U)) | ((0x1UL << (31U)) | (0x1UL << (5U)) | (0x1UL << (4U)) | (0x1UL << (3U)) | (0x1UL << (2U)) | (0x1UL << (1U)) | (0x1UL << (0U))))));
}






 
static __inline uint32_t LL_ADC_IsDeepPowerDownEnabled(ADC_TypeDef *ADCx)
{
  return ((((ADCx->CR) & ((0x1UL << (29U)))) == ((0x1UL << (29U)))) ? 1UL : 0UL);
}














 
static __inline void LL_ADC_EnableInternalRegulator(ADC_TypeDef *ADCx)
{
   
   
   
  (((ADCx->CR)) = ((((((ADCx->CR))) & (~(((0x1UL << (31U)) | (0x1UL << (5U)) | (0x1UL << (4U)) | (0x1UL << (3U)) | (0x1UL << (2U)) | (0x1UL << (1U)) | (0x1UL << (0U)))))) | ((0x1UL << (28U))))));


}









 
static __inline void LL_ADC_DisableInternalRegulator(ADC_TypeDef *ADCx)
{
  ((ADCx->CR) &= ~(((0x1UL << (28U)) | ((0x1UL << (31U)) | (0x1UL << (5U)) | (0x1UL << (4U)) | (0x1UL << (3U)) | (0x1UL << (2U)) | (0x1UL << (1U)) | (0x1UL << (0U))))));
}






 
static __inline uint32_t LL_ADC_IsInternalRegulatorEnabled(ADC_TypeDef *ADCx)
{
  return ((((ADCx->CR) & ((0x1UL << (28U)))) == ((0x1UL << (28U)))) ? 1UL : 0UL);
}
















 
static __inline void LL_ADC_Enable(ADC_TypeDef *ADCx)
{
   
   
   
  (((ADCx->CR)) = ((((((ADCx->CR))) & (~(((0x1UL << (31U)) | (0x1UL << (5U)) | (0x1UL << (4U)) | (0x1UL << (3U)) | (0x1UL << (2U)) | (0x1UL << (1U)) | (0x1UL << (0U)))))) | ((0x1UL << (0U))))));


}










 
static __inline void LL_ADC_Disable(ADC_TypeDef *ADCx)
{
   
   
   
  (((ADCx->CR)) = ((((((ADCx->CR))) & (~(((0x1UL << (31U)) | (0x1UL << (5U)) | (0x1UL << (4U)) | (0x1UL << (3U)) | (0x1UL << (2U)) | (0x1UL << (1U)) | (0x1UL << (0U)))))) | ((0x1UL << (1U))))));


}









 
static __inline uint32_t LL_ADC_IsEnabled(ADC_TypeDef *ADCx)
{
  return ((((ADCx->CR) & ((0x1UL << (0U)))) == ((0x1UL << (0U)))) ? 1UL : 0UL);
}






 
static __inline uint32_t LL_ADC_IsDisableOngoing(ADC_TypeDef *ADCx)
{
  return ((((ADCx->CR) & ((0x1UL << (1U)))) == ((0x1UL << (1U)))) ? 1UL : 0UL);
}























 
static __inline void LL_ADC_StartCalibration(ADC_TypeDef *ADCx, uint32_t SingleDiff)
{
   
   
   
  (((ADCx->CR)) = ((((((ADCx->CR))) & (~((0x1UL << (30U)) | ((0x1UL << (31U)) | (0x1UL << (5U)) | (0x1UL << (4U)) | (0x1UL << (3U)) | (0x1UL << (2U)) | (0x1UL << (1U)) | (0x1UL << (0U)))))) | ((0x1UL << (31U)) | (SingleDiff & ((0x1UL << (30U))))))));


}






 
static __inline uint32_t LL_ADC_IsCalibrationOnGoing(ADC_TypeDef *ADCx)
{
  return ((((ADCx->CR) & ((0x1UL << (31U)))) == ((0x1UL << (31U)))) ? 1UL : 0UL);
}



 



 


















 
static __inline void LL_ADC_REG_StartConversion(ADC_TypeDef *ADCx)
{
   
   
   
  (((ADCx->CR)) = ((((((ADCx->CR))) & (~(((0x1UL << (31U)) | (0x1UL << (5U)) | (0x1UL << (4U)) | (0x1UL << (3U)) | (0x1UL << (2U)) | (0x1UL << (1U)) | (0x1UL << (0U)))))) | ((0x1UL << (2U))))));


}










 
static __inline void LL_ADC_REG_StopConversion(ADC_TypeDef *ADCx)
{
   
   
   
  (((ADCx->CR)) = ((((((ADCx->CR))) & (~(((0x1UL << (31U)) | (0x1UL << (5U)) | (0x1UL << (4U)) | (0x1UL << (3U)) | (0x1UL << (2U)) | (0x1UL << (1U)) | (0x1UL << (0U)))))) | ((0x1UL << (4U))))));


}






 
static __inline uint32_t LL_ADC_REG_IsConversionOngoing(ADC_TypeDef *ADCx)
{
  return ((((ADCx->CR) & ((0x1UL << (2U)))) == ((0x1UL << (2U)))) ? 1UL : 0UL);
}






 
static __inline uint32_t LL_ADC_REG_IsStopConversionOngoing(ADC_TypeDef *ADCx)
{
  return ((((ADCx->CR) & ((0x1UL << (4U)))) == ((0x1UL << (4U)))) ? 1UL : 0UL);
}















 
static __inline void LL_ADC_REG_StartSamplingPhase(ADC_TypeDef *ADCx)
{
  ((ADCx->CFGR2) |= ((0x1UL << (25U))));
}

















 
static __inline void LL_ADC_REG_StopSamplingPhase(ADC_TypeDef *ADCx)
{
  ((ADCx->CFGR2) &= ~((0x1UL << (25U))));
}









 
static __inline uint32_t LL_ADC_REG_ReadConversionData32(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->DR) & ((0xFFFFUL << (0U)))));
}










 
static __inline uint16_t LL_ADC_REG_ReadConversionData12(ADC_TypeDef *ADCx)
{
  return (uint16_t)(((ADCx->DR) & ((0xFFFFUL << (0U)))));
}










 
static __inline uint16_t LL_ADC_REG_ReadConversionData10(ADC_TypeDef *ADCx)
{
  return (uint16_t)(((ADCx->DR) & ((0xFFFFUL << (0U)))));
}










 
static __inline uint8_t LL_ADC_REG_ReadConversionData8(ADC_TypeDef *ADCx)
{
  return (uint8_t)(((ADCx->DR) & ((0xFFFFUL << (0U)))));
}










 
static __inline uint8_t LL_ADC_REG_ReadConversionData6(ADC_TypeDef *ADCx)
{
  return (uint8_t)(((ADCx->DR) & ((0xFFFFUL << (0U)))));
}






















 
static __inline uint32_t LL_ADC_REG_ReadMultiConversionData32(ADC_Common_TypeDef *ADCxy_COMMON, uint32_t ConversionData)
{
  return (uint32_t)(((ADCxy_COMMON->CDR) & (ConversionData))

                    >> ((__clz(__rbit(ConversionData))) & 0x1FUL)
                   );
}




 



 


















 
static __inline void LL_ADC_INJ_StartConversion(ADC_TypeDef *ADCx)
{
   
   
   
  (((ADCx->CR)) = ((((((ADCx->CR))) & (~(((0x1UL << (31U)) | (0x1UL << (5U)) | (0x1UL << (4U)) | (0x1UL << (3U)) | (0x1UL << (2U)) | (0x1UL << (1U)) | (0x1UL << (0U)))))) | ((0x1UL << (3U))))));


}










 
static __inline void LL_ADC_INJ_StopConversion(ADC_TypeDef *ADCx)
{
   
   
   
  (((ADCx->CR)) = ((((((ADCx->CR))) & (~(((0x1UL << (31U)) | (0x1UL << (5U)) | (0x1UL << (4U)) | (0x1UL << (3U)) | (0x1UL << (2U)) | (0x1UL << (1U)) | (0x1UL << (0U)))))) | ((0x1UL << (5U))))));


}






 
static __inline uint32_t LL_ADC_INJ_IsConversionOngoing(ADC_TypeDef *ADCx)
{
  return ((((ADCx->CR) & ((0x1UL << (3U)))) == ((0x1UL << (3U)))) ? 1UL : 0UL);
}






 
static __inline uint32_t LL_ADC_INJ_IsStopConversionOngoing(ADC_TypeDef *ADCx)
{
  return ((((ADCx->CR) & ((0x1UL << (5U)))) == ((0x1UL << (5U)))) ? 1UL : 0UL);
}

















 
static __inline uint32_t LL_ADC_INJ_ReadConversionData32(ADC_TypeDef *ADCx, uint32_t Rank)
{
  const volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->JDR1)) + ((((Rank & ((0x00000000UL) | (0x00000100UL) | (0x00000200UL) | (0x00000300UL))) >> (8UL))) << 2UL))));

  return (uint32_t)(((*preg) & ((0xFFFFUL << (0U))))

                   );
}


















 
static __inline uint16_t LL_ADC_INJ_ReadConversionData12(ADC_TypeDef *ADCx, uint32_t Rank)
{
  const volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->JDR1)) + ((((Rank & ((0x00000000UL) | (0x00000100UL) | (0x00000200UL) | (0x00000300UL))) >> (8UL))) << 2UL))));

  return (uint16_t)(((*preg) & ((0xFFFFUL << (0U))))

                   );
}


















 
static __inline uint16_t LL_ADC_INJ_ReadConversionData10(ADC_TypeDef *ADCx, uint32_t Rank)
{
  const volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->JDR1)) + ((((Rank & ((0x00000000UL) | (0x00000100UL) | (0x00000200UL) | (0x00000300UL))) >> (8UL))) << 2UL))));

  return (uint16_t)(((*preg) & ((0xFFFFUL << (0U))))

                   );
}


















 
static __inline uint8_t LL_ADC_INJ_ReadConversionData8(ADC_TypeDef *ADCx, uint32_t Rank)
{
  const volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->JDR1)) + ((((Rank & ((0x00000000UL) | (0x00000100UL) | (0x00000200UL) | (0x00000300UL))) >> (8UL))) << 2UL))));

  return (uint8_t)(((*preg) & ((0xFFFFUL << (0U))))

                  );
}


















 
static __inline uint8_t LL_ADC_INJ_ReadConversionData6(ADC_TypeDef *ADCx, uint32_t Rank)
{
  const volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->JDR1)) + ((((Rank & ((0x00000000UL) | (0x00000100UL) | (0x00000200UL) | (0x00000300UL))) >> (8UL))) << 2UL))));

  return (uint8_t)(((*preg) & ((0xFFFFUL << (0U))))

                  );
}



 



 









 
static __inline uint32_t LL_ADC_IsActiveFlag_ADRDY(ADC_TypeDef *ADCx)
{
  return ((((ADCx->ISR) & ((0x1UL << (0U)))) == ((0x1UL << (0U)))) ? 1UL : 0UL);
}






 
static __inline uint32_t LL_ADC_IsActiveFlag_EOC(ADC_TypeDef *ADCx)
{
  return ((((ADCx->ISR) & ((0x1UL << (2U)))) == ((0x1UL << (2U)))) ? 1UL : 0UL);
}






 
static __inline uint32_t LL_ADC_IsActiveFlag_EOS(ADC_TypeDef *ADCx)
{
  return ((((ADCx->ISR) & ((0x1UL << (3U)))) == ((0x1UL << (3U)))) ? 1UL : 0UL);
}






 
static __inline uint32_t LL_ADC_IsActiveFlag_OVR(ADC_TypeDef *ADCx)
{
  return ((((ADCx->ISR) & ((0x1UL << (4U)))) == ((0x1UL << (4U)))) ? 1UL : 0UL);
}






 
static __inline uint32_t LL_ADC_IsActiveFlag_EOSMP(ADC_TypeDef *ADCx)
{
  return ((((ADCx->ISR) & ((0x1UL << (1U)))) == ((0x1UL << (1U)))) ? 1UL : 0UL);
}






 
static __inline uint32_t LL_ADC_IsActiveFlag_JEOC(ADC_TypeDef *ADCx)
{
  return ((((ADCx->ISR) & ((0x1UL << (5U)))) == ((0x1UL << (5U)))) ? 1UL : 0UL);
}






 
static __inline uint32_t LL_ADC_IsActiveFlag_JEOS(ADC_TypeDef *ADCx)
{
  return ((((ADCx->ISR) & ((0x1UL << (6U)))) == ((0x1UL << (6U)))) ? 1UL : 0UL);
}






 
static __inline uint32_t LL_ADC_IsActiveFlag_JQOVF(ADC_TypeDef *ADCx)
{
  return ((((ADCx->ISR) & ((0x1UL << (10U)))) == ((0x1UL << (10U)))) ? 1UL : 0UL);
}






 
static __inline uint32_t LL_ADC_IsActiveFlag_AWD1(ADC_TypeDef *ADCx)
{
  return ((((ADCx->ISR) & ((0x1UL << (7U)))) == ((0x1UL << (7U)))) ? 1UL : 0UL);
}






 
static __inline uint32_t LL_ADC_IsActiveFlag_AWD2(ADC_TypeDef *ADCx)
{
  return ((((ADCx->ISR) & ((0x1UL << (8U)))) == ((0x1UL << (8U)))) ? 1UL : 0UL);
}






 
static __inline uint32_t LL_ADC_IsActiveFlag_AWD3(ADC_TypeDef *ADCx)
{
  return ((((ADCx->ISR) & ((0x1UL << (9U)))) == ((0x1UL << (9U)))) ? 1UL : 0UL);
}









 
static __inline void LL_ADC_ClearFlag_ADRDY(ADC_TypeDef *ADCx)
{
  ((ADCx->ISR) = ((0x1UL << (0U))));
}






 
static __inline void LL_ADC_ClearFlag_EOC(ADC_TypeDef *ADCx)
{
  ((ADCx->ISR) = ((0x1UL << (2U))));
}






 
static __inline void LL_ADC_ClearFlag_EOS(ADC_TypeDef *ADCx)
{
  ((ADCx->ISR) = ((0x1UL << (3U))));
}






 
static __inline void LL_ADC_ClearFlag_OVR(ADC_TypeDef *ADCx)
{
  ((ADCx->ISR) = ((0x1UL << (4U))));
}






 
static __inline void LL_ADC_ClearFlag_EOSMP(ADC_TypeDef *ADCx)
{
  ((ADCx->ISR) = ((0x1UL << (1U))));
}






 
static __inline void LL_ADC_ClearFlag_JEOC(ADC_TypeDef *ADCx)
{
  ((ADCx->ISR) = ((0x1UL << (5U))));
}






 
static __inline void LL_ADC_ClearFlag_JEOS(ADC_TypeDef *ADCx)
{
  ((ADCx->ISR) = ((0x1UL << (6U))));
}






 
static __inline void LL_ADC_ClearFlag_JQOVF(ADC_TypeDef *ADCx)
{
  ((ADCx->ISR) = ((0x1UL << (10U))));
}






 
static __inline void LL_ADC_ClearFlag_AWD1(ADC_TypeDef *ADCx)
{
  ((ADCx->ISR) = ((0x1UL << (7U))));
}






 
static __inline void LL_ADC_ClearFlag_AWD2(ADC_TypeDef *ADCx)
{
  ((ADCx->ISR) = ((0x1UL << (8U))));
}






 
static __inline void LL_ADC_ClearFlag_AWD3(ADC_TypeDef *ADCx)
{
  ((ADCx->ISR) = ((0x1UL << (9U))));
}








 
static __inline uint32_t LL_ADC_IsActiveFlag_MST_ADRDY(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (0U)))) == ((0x1UL << (0U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_SLV_ADRDY(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (16U)))) == ((0x1UL << (16U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_MST_EOC(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (18U)))) == ((0x1UL << (18U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_SLV_EOC(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (18U)))) == ((0x1UL << (18U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_MST_EOS(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (3U)))) == ((0x1UL << (3U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_SLV_EOS(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (19U)))) == ((0x1UL << (19U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_MST_OVR(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (4U)))) == ((0x1UL << (4U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_SLV_OVR(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (20U)))) == ((0x1UL << (20U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_MST_EOSMP(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (1U)))) == ((0x1UL << (1U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_SLV_EOSMP(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (17U)))) == ((0x1UL << (17U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_MST_JEOC(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (5U)))) == ((0x1UL << (5U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_SLV_JEOC(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (21U)))) == ((0x1UL << (21U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_MST_JEOS(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (6U)))) == ((0x1UL << (6U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_SLV_JEOS(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (22U)))) == ((0x1UL << (22U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_MST_JQOVF(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (10U)))) == ((0x1UL << (10U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_SLV_JQOVF(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (26U)))) == ((0x1UL << (26U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_MST_AWD1(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (7U)))) == ((0x1UL << (7U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_SLV_AWD1(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (23U)))) == ((0x1UL << (23U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_MST_AWD2(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (8U)))) == ((0x1UL << (8U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_SLV_AWD2(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (24U)))) == ((0x1UL << (24U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_MST_AWD3(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (9U)))) == ((0x1UL << (9U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_SLV_AWD3(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (25U)))) == ((0x1UL << (25U)))) ? 1UL : 0UL);
}




 



 






 
static __inline void LL_ADC_EnableIT_ADRDY(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) |= ((0x1UL << (0U))));
}






 
static __inline void LL_ADC_EnableIT_EOC(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) |= ((0x1UL << (2U))));
}






 
static __inline void LL_ADC_EnableIT_EOS(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) |= ((0x1UL << (3U))));
}






 
static __inline void LL_ADC_EnableIT_OVR(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) |= ((0x1UL << (4U))));
}






 
static __inline void LL_ADC_EnableIT_EOSMP(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) |= ((0x1UL << (1U))));
}






 
static __inline void LL_ADC_EnableIT_JEOC(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) |= ((0x1UL << (5U))));
}






 
static __inline void LL_ADC_EnableIT_JEOS(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) |= ((0x1UL << (6U))));
}






 
static __inline void LL_ADC_EnableIT_JQOVF(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) |= ((0x1UL << (10U))));
}






 
static __inline void LL_ADC_EnableIT_AWD1(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) |= ((0x1UL << (7U))));
}






 
static __inline void LL_ADC_EnableIT_AWD2(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) |= ((0x1UL << (8U))));
}






 
static __inline void LL_ADC_EnableIT_AWD3(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) |= ((0x1UL << (9U))));
}






 
static __inline void LL_ADC_DisableIT_ADRDY(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) &= ~((0x1UL << (0U))));
}






 
static __inline void LL_ADC_DisableIT_EOC(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) &= ~((0x1UL << (2U))));
}






 
static __inline void LL_ADC_DisableIT_EOS(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) &= ~((0x1UL << (3U))));
}






 
static __inline void LL_ADC_DisableIT_OVR(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) &= ~((0x1UL << (4U))));
}






 
static __inline void LL_ADC_DisableIT_EOSMP(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) &= ~((0x1UL << (1U))));
}






 
static __inline void LL_ADC_DisableIT_JEOC(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) &= ~((0x1UL << (5U))));
}






 
static __inline void LL_ADC_DisableIT_JEOS(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) &= ~((0x1UL << (6U))));
}






 
static __inline void LL_ADC_DisableIT_JQOVF(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) &= ~((0x1UL << (10U))));
}






 
static __inline void LL_ADC_DisableIT_AWD1(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) &= ~((0x1UL << (7U))));
}






 
static __inline void LL_ADC_DisableIT_AWD2(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) &= ~((0x1UL << (8U))));
}






 
static __inline void LL_ADC_DisableIT_AWD3(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) &= ~((0x1UL << (9U))));
}







 
static __inline uint32_t LL_ADC_IsEnabledIT_ADRDY(ADC_TypeDef *ADCx)
{
  return ((((ADCx->IER) & ((0x1UL << (0U)))) == ((0x1UL << (0U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsEnabledIT_EOC(ADC_TypeDef *ADCx)
{
  return ((((ADCx->IER) & ((0x1UL << (2U)))) == ((0x1UL << (2U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsEnabledIT_EOS(ADC_TypeDef *ADCx)
{
  return ((((ADCx->IER) & ((0x1UL << (3U)))) == ((0x1UL << (3U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsEnabledIT_OVR(ADC_TypeDef *ADCx)
{
  return ((((ADCx->IER) & ((0x1UL << (4U)))) == ((0x1UL << (4U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsEnabledIT_EOSMP(ADC_TypeDef *ADCx)
{
  return ((((ADCx->IER) & ((0x1UL << (1U)))) == ((0x1UL << (1U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsEnabledIT_JEOC(ADC_TypeDef *ADCx)
{
  return ((((ADCx->IER) & ((0x1UL << (5U)))) == ((0x1UL << (5U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsEnabledIT_JEOS(ADC_TypeDef *ADCx)
{
  return ((((ADCx->IER) & ((0x1UL << (6U)))) == ((0x1UL << (6U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsEnabledIT_JQOVF(ADC_TypeDef *ADCx)
{
  return ((((ADCx->IER) & ((0x1UL << (10U)))) == ((0x1UL << (10U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsEnabledIT_AWD1(ADC_TypeDef *ADCx)
{
  return ((((ADCx->IER) & ((0x1UL << (7U)))) == ((0x1UL << (7U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsEnabledIT_AWD2(ADC_TypeDef *ADCx)
{
  return ((((ADCx->IER) & ((0x1UL << (8U)))) == ((0x1UL << (8U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsEnabledIT_AWD3(ADC_TypeDef *ADCx)
{
  return ((((ADCx->IER) & ((0x1UL << (9U)))) == ((0x1UL << (9U)))) ? 1UL : 0UL);
}



 

# 8157 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_ll_adc.h"



 



 





 







 
# 33 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_adc.h"



 



 

 


 



 
typedef struct
{
  uint32_t Ratio;                         
 

  uint32_t RightBitShift;                 
 

  uint32_t TriggeredMode;                 
 

  uint32_t OversamplingStopReset;         





 

} ADC_OversamplingTypeDef;
















 
typedef struct
{
  uint32_t ClockPrescaler;        








 

  uint32_t Resolution;            
 

  uint32_t DataAlign;             

 

  uint32_t GainCompensation;      








 

  uint32_t ScanConvMode;          





 

  uint32_t EOCSelection;          
 

  FunctionalState LowPowerAutoWait; 









 

  FunctionalState ContinuousConvMode; 

 

  uint32_t NbrOfConversion;       



 

  FunctionalState DiscontinuousConvMode; 



 

  uint32_t NbrOfDiscConversion;   

 

  uint32_t ExternalTrigConv;      


 

  uint32_t ExternalTrigConvEdge;  

 

  uint32_t SamplingMode;          
 

  FunctionalState DMAContinuousRequests; 


 

  uint32_t Overrun;               








 

  FunctionalState OversamplingMode;       

 

  ADC_OversamplingTypeDef Oversampling;   
 

} ADC_InitTypeDef;











 
typedef struct
{
  uint32_t Channel;                

 

  uint32_t Rank;                   


 

  uint32_t SamplingTime;           








 

  uint32_t SingleDiff;             









 

  uint32_t OffsetNumber;           

 

  uint32_t Offset;                 




 

  uint32_t OffsetSign;                


 
  FunctionalState OffsetSaturation;   


 

} ADC_ChannelConfTypeDef;







 
typedef struct
{
  uint32_t WatchdogNumber;    


 

  uint32_t WatchdogMode;      


 

  uint32_t Channel;           


 

  FunctionalState ITMode;     
 

  uint32_t HighThreshold;     







 

  uint32_t LowThreshold;      







 

  uint32_t FilteringConfig;   



 
} ADC_AnalogWDGConfTypeDef;




 
typedef struct
{
  uint32_t ContextQueue;                 

 

  uint32_t ChannelCount;                  
} ADC_InjectionConfigTypeDef;



 








 
 





 




 






 





 




 




 



 



typedef struct

{
  ADC_TypeDef                   *Instance;               
  ADC_InitTypeDef               Init;                    
  DMA_HandleTypeDef             *DMA_Handle;             
  HAL_LockTypeDef               Lock;                    
  volatile uint32_t                 State;                   
  volatile uint32_t                 ErrorCode;               
  ADC_InjectionConfigTypeDef    InjectionConfig ;        
# 409 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_adc.h"
} ADC_HandleTypeDef;

# 436 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_adc.h"



 


 



 



 
# 460 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_adc.h"


 



 




# 483 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_adc.h"


 



 






 



 




 



 




 



 
 
# 560 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_adc.h"


 



 






 



 
# 584 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_adc.h"


 



 




 



 




 



 
# 625 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_adc.h"


 



 
# 641 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_adc.h"


 



 
 
 
# 680 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_adc.h"


 



 





 



 
# 705 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_adc.h"


 



 
# 719 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_adc.h"


 



 
# 734 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_adc.h"


 



 
# 750 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_adc.h"


 



 




 



 




 



 
# 781 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_adc.h"


 




 
# 800 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_adc.h"





 



 
# 821 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_adc.h"



 



 

 



 
 
 





 







 








 







 







 







 







 
# 907 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_adc.h"





 









 







 







 






 







 










 
# 1123 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_adc.h"





 








 







 







 
# 1163 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_adc.h"





 
# 1185 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_adc.h"



 


 



 

 
 
 
 
 
 
 


 
 
 




 

 



 
 
 



 




 
# 1242 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_adc.h"


















 




















 



















 




















 




















 
 





 



 





















































 























































 
































































 














































































 








































 
















 













 



















 















 




















 
# 1744 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_adc.h"
















 
# 1767 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_adc.h"

























 

















































 
# 1849 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_adc.h"












































 
# 1906 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_adc.h"



 



 

 
# 1 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_adc_ex.h"

















 

 







 
# 30 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_adc_ex.h"



 



 

 


 



 
typedef struct
{
  uint32_t Ratio;                         
 

  uint32_t RightBitShift;                 
 
} ADC_InjOversamplingTypeDef;
















 
typedef struct
{
  uint32_t InjectedChannel;               

 

  uint32_t InjectedRank;                  


 

  uint32_t InjectedSamplingTime;          








 

  uint32_t InjectedSingleDiff;            









 

  uint32_t InjectedOffsetNumber;          

 

  uint32_t InjectedOffset;                




 

  uint32_t InjectedOffsetSign;                

 
  FunctionalState InjectedOffsetSaturation;   

 

  uint32_t InjectedNbrOfConversion;       



 

  FunctionalState InjectedDiscontinuousConvMode; 







 

  FunctionalState AutoInjectedConv;       






 

  FunctionalState QueueInjectedContext;   








 

  uint32_t ExternalTrigInjecConv;         



 

  uint32_t ExternalTrigInjecConvEdge;     



 

  FunctionalState InjecOversamplingMode;         

 

  ADC_InjOversamplingTypeDef  InjecOversampling; 

 
} ADC_InjectionConfTypeDef;






 
typedef struct
{
  uint32_t Mode;              
 

  uint32_t DMAAccessMode;     

 

  uint32_t TwoSamplingDelay;  



 
} ADC_MultiModeTypeDef;




 

 



 



 
 
# 257 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_adc_ex.h"


 



 






 



 




 



 







 



 




 



 






 




 
# 325 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_adc_ex.h"



 





 



 
# 351 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_adc_ex.h"


 



 




 





 



 
# 379 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_adc_ex.h"


 



 
# 397 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_adc_ex.h"


 



 

 



 



 

 




 














 





 


 



 
 
 






 








 
# 486 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_adc_ex.h"






 







 






 






 






 






 






 






 







 














 














 













 










 
# 618 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_adc_ex.h"

# 637 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_adc_ex.h"






 
# 651 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_adc_ex.h"






 










 
# 676 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_adc_ex.h"





 






 






 








 
# 843 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_adc_ex.h"






 
# 924 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_adc_ex.h"





 







 










 







 










 
# 1124 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_adc_ex.h"





 










 
# 1149 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_adc_ex.h"





 








 
# 1177 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_adc_ex.h"





 








 
# 1199 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_adc_ex.h"





 
# 1213 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_adc_ex.h"






 








 
# 1235 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_adc_ex.h"





 
# 1249 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_adc_ex.h"





 
# 1264 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_adc_ex.h"





 







 










 









 




 


 


 



 
 

 
HAL_StatusTypeDef       HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef *hadc, uint32_t SingleDiff);
uint32_t                HAL_ADCEx_Calibration_GetValue(ADC_HandleTypeDef *hadc, uint32_t SingleDiff);
HAL_StatusTypeDef       HAL_ADCEx_Calibration_SetValue(ADC_HandleTypeDef *hadc, uint32_t SingleDiff,
                                                       uint32_t CalibrationFactor);

 
HAL_StatusTypeDef       HAL_ADCEx_InjectedStart(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef       HAL_ADCEx_InjectedStop(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef       HAL_ADCEx_InjectedPollForConversion(ADC_HandleTypeDef *hadc, uint32_t Timeout);

 
HAL_StatusTypeDef       HAL_ADCEx_InjectedStart_IT(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef       HAL_ADCEx_InjectedStop_IT(ADC_HandleTypeDef *hadc);


 
HAL_StatusTypeDef       HAL_ADCEx_MultiModeStart_DMA(ADC_HandleTypeDef *hadc, uint32_t *pData, uint32_t Length);
HAL_StatusTypeDef       HAL_ADCEx_MultiModeStop_DMA(ADC_HandleTypeDef *hadc);
uint32_t                HAL_ADCEx_MultiModeGetValue(ADC_HandleTypeDef *hadc);


 
uint32_t                HAL_ADCEx_InjectedGetValue(ADC_HandleTypeDef *hadc, uint32_t InjectedRank);

 
void                    HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc);
void                    HAL_ADCEx_InjectedQueueOverflowCallback(ADC_HandleTypeDef *hadc);
void                    HAL_ADCEx_LevelOutOfWindow2Callback(ADC_HandleTypeDef *hadc);
void                    HAL_ADCEx_LevelOutOfWindow3Callback(ADC_HandleTypeDef *hadc);
void                    HAL_ADCEx_EndOfSamplingCallback(ADC_HandleTypeDef *hadc);

 
HAL_StatusTypeDef HAL_ADCEx_RegularStop(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef HAL_ADCEx_RegularStop_IT(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef HAL_ADCEx_RegularStop_DMA(ADC_HandleTypeDef *hadc);

HAL_StatusTypeDef HAL_ADCEx_RegularMultiModeStop_DMA(ADC_HandleTypeDef *hadc);




 



 
 
HAL_StatusTypeDef       HAL_ADCEx_InjectedConfigChannel(ADC_HandleTypeDef *hadc,
                                                        ADC_InjectionConfTypeDef *sConfigInjected);

HAL_StatusTypeDef       HAL_ADCEx_MultiModeConfigChannel(ADC_HandleTypeDef *hadc, ADC_MultiModeTypeDef *multimode);

HAL_StatusTypeDef       HAL_ADCEx_EnableInjectedQueue(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef       HAL_ADCEx_DisableInjectedQueue(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef       HAL_ADCEx_DisableVoltageRegulator(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef       HAL_ADCEx_EnterADCDeepPowerDownMode(ADC_HandleTypeDef *hadc);



 



 



 



 








 
# 1917 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_adc.h"

 


 




 
 
HAL_StatusTypeDef       HAL_ADC_Init(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef       HAL_ADC_DeInit(ADC_HandleTypeDef *hadc);
void                    HAL_ADC_MspInit(ADC_HandleTypeDef *hadc);
void                    HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc);

# 1939 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_adc.h"


 




 
 

 
HAL_StatusTypeDef       HAL_ADC_Start(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef       HAL_ADC_Stop(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef       HAL_ADC_PollForConversion(ADC_HandleTypeDef *hadc, uint32_t Timeout);
HAL_StatusTypeDef       HAL_ADC_PollForEvent(ADC_HandleTypeDef *hadc, uint32_t EventType, uint32_t Timeout);

 
HAL_StatusTypeDef       HAL_ADC_Start_IT(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef       HAL_ADC_Stop_IT(ADC_HandleTypeDef *hadc);

 
HAL_StatusTypeDef       HAL_ADC_Start_DMA(ADC_HandleTypeDef *hadc, uint32_t *pData, uint32_t Length);
HAL_StatusTypeDef       HAL_ADC_Stop_DMA(ADC_HandleTypeDef *hadc);

 
uint32_t                HAL_ADC_GetValue(ADC_HandleTypeDef *hadc);

 
HAL_StatusTypeDef HAL_ADC_StartSampling(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef HAL_ADC_StopSampling(ADC_HandleTypeDef *hadc);

 
void                    HAL_ADC_IRQHandler(ADC_HandleTypeDef *hadc);
void                    HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);
void                    HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc);
void                    HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef *hadc);
void                    HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc);


 




 
 
HAL_StatusTypeDef       HAL_ADC_ConfigChannel(ADC_HandleTypeDef *hadc, ADC_ChannelConfTypeDef *sConfig);
HAL_StatusTypeDef       HAL_ADC_AnalogWDGConfig(ADC_HandleTypeDef *hadc, ADC_AnalogWDGConfTypeDef *AnalogWDGConfig);



 

 


 
uint32_t                HAL_ADC_GetState(ADC_HandleTypeDef *hadc);
uint32_t                HAL_ADC_GetError(ADC_HandleTypeDef *hadc);



 



 

 


 
HAL_StatusTypeDef ADC_ConversionStop(ADC_HandleTypeDef *hadc, uint32_t ConversionGroup);
HAL_StatusTypeDef ADC_Enable(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef ADC_Disable(ADC_HandleTypeDef *hadc);
void ADC_DMAConvCplt(DMA_HandleTypeDef *hdma);
void ADC_DMAHalfConvCplt(DMA_HandleTypeDef *hdma);
void ADC_DMAError(DMA_HandleTypeDef *hdma);



 



 



 








 
# 228 "..\\..\\User\\stm32g4xx_hal_conf.h"










# 1 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_crc.h"

















 

 







 
# 30 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_crc.h"



 



 

 


 



 
typedef enum
{
  HAL_CRC_STATE_RESET     = 0x00U,   
  HAL_CRC_STATE_READY     = 0x01U,   
  HAL_CRC_STATE_BUSY      = 0x02U,   
  HAL_CRC_STATE_TIMEOUT   = 0x03U,   
  HAL_CRC_STATE_ERROR     = 0x04U    
} HAL_CRC_StateTypeDef;



 
typedef struct
{
  uint8_t DefaultPolynomialUse;       



 

  uint8_t DefaultInitValueUse;        


 

  uint32_t GeneratingPolynomial;      


 

  uint32_t CRCLength;                 




 

  uint32_t InitValue;                 
 

  uint32_t InputDataInversionMode;    




 

  uint32_t OutputDataInversionMode;   


 
} CRC_InitTypeDef;



 
typedef struct
{
  CRC_TypeDef                 *Instance;    

  CRC_InitTypeDef             Init;         

  HAL_LockTypeDef             Lock;         

  volatile HAL_CRC_StateTypeDef   State;        

  uint32_t InputDataFormat;                






 
} CRC_HandleTypeDef;


 

 


 



 



 



 



 



 




 



 




 



 






 



 






 



 



 






 



 




 



 

 


 




 






 







 








 







 



 


 


 


















 

 
# 1 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_crc_ex.h"

















 

 







 
# 30 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_crc_ex.h"



 



 

 
 


 



 






 



 




 



 

 


 





 






 







 




 

 


 











 

 



 



 
 
HAL_StatusTypeDef HAL_CRCEx_Polynomial_Set(CRC_HandleTypeDef *hcrc, uint32_t Pol, uint32_t PolyLength);
HAL_StatusTypeDef HAL_CRCEx_Input_Data_Reverse(CRC_HandleTypeDef *hcrc, uint32_t InputReverseMode);
HAL_StatusTypeDef HAL_CRCEx_Output_Data_Reverse(CRC_HandleTypeDef *hcrc, uint32_t OutputReverseMode);



 



 



 



 







 
# 288 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_crc.h"

 


 

 


 
HAL_StatusTypeDef HAL_CRC_Init(CRC_HandleTypeDef *hcrc);
HAL_StatusTypeDef HAL_CRC_DeInit(CRC_HandleTypeDef *hcrc);
void HAL_CRC_MspInit(CRC_HandleTypeDef *hcrc);
void HAL_CRC_MspDeInit(CRC_HandleTypeDef *hcrc);


 

 


 
uint32_t HAL_CRC_Accumulate(CRC_HandleTypeDef *hcrc, uint32_t pBuffer[], uint32_t BufferLength);
uint32_t HAL_CRC_Calculate(CRC_HandleTypeDef *hcrc, uint32_t pBuffer[], uint32_t BufferLength);


 

 


 
HAL_CRC_StateTypeDef HAL_CRC_GetState(CRC_HandleTypeDef *hcrc);


 



 



 



 







 
# 240 "..\\..\\User\\stm32g4xx_hal_conf.h"


# 1 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_cryp.h"

















 

 







 
# 30 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_cryp.h"



 

# 638 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_cryp.h"


 







 
# 244 "..\\..\\User\\stm32g4xx_hal_conf.h"






# 1 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_exti.h"

















 

 







 
# 30 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_exti.h"



 




 

 



 
typedef enum
{
  HAL_EXTI_COMMON_CB_ID         = 0x00UL
} EXTI_CallbackIDTypeDef;




 
typedef struct
{
  uint32_t Line;                     
  void (* PendingCallback)(void);    
} EXTI_HandleTypeDef;



 
typedef struct
{
  uint32_t Line;      
 
  uint32_t Mode;      
 
  uint32_t Trigger;   
 
  uint32_t GPIOSel;   

 
} EXTI_ConfigTypeDef;



 

 


 



 
# 132 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_exti.h"


 



 





 



 






 




 
# 168 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_exti.h"


 



 

 


 



 

 


 


 
# 198 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_exti.h"



 








 




 




 




 

 


 
# 237 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_exti.h"








# 252 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_exti.h"










 


 



 




 
 
HAL_StatusTypeDef HAL_EXTI_SetConfigLine(EXTI_HandleTypeDef *hexti, EXTI_ConfigTypeDef *pExtiConfig);
HAL_StatusTypeDef HAL_EXTI_GetConfigLine(EXTI_HandleTypeDef *hexti, EXTI_ConfigTypeDef *pExtiConfig);
HAL_StatusTypeDef HAL_EXTI_ClearConfigLine(EXTI_HandleTypeDef *hexti);
HAL_StatusTypeDef HAL_EXTI_RegisterCallback(EXTI_HandleTypeDef *hexti, EXTI_CallbackIDTypeDef CallbackID, void (*pPendingCbfn)(void));
HAL_StatusTypeDef HAL_EXTI_GetHandle(EXTI_HandleTypeDef *hexti, uint32_t ExtiLine);


 




 
 
void              HAL_EXTI_IRQHandler(EXTI_HandleTypeDef *hexti);
uint32_t          HAL_EXTI_GetPending(EXTI_HandleTypeDef *hexti, uint32_t Edge);
void              HAL_EXTI_ClearPending(EXTI_HandleTypeDef *hexti, uint32_t Edge);
void              HAL_EXTI_GenerateSWI(EXTI_HandleTypeDef *hexti);



 



 



 



 







 
# 252 "..\\..\\User\\stm32g4xx_hal_conf.h"


# 1 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_fdcan.h"

















 

 







 
# 30 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_fdcan.h"





 



 

 


 



 
typedef enum
{
  HAL_FDCAN_STATE_RESET      = 0x00U,  
  HAL_FDCAN_STATE_READY      = 0x01U,  
  HAL_FDCAN_STATE_BUSY       = 0x02U,  
  HAL_FDCAN_STATE_ERROR      = 0x03U   
} HAL_FDCAN_StateTypeDef;



 
typedef struct
{
  uint32_t ClockDivider;                 



 

  uint32_t FrameFormat;                  
 

  uint32_t Mode;                         
 

  FunctionalState AutoRetransmission;    
 

  FunctionalState TransmitPause;         
 

  FunctionalState ProtocolException;      
 

  uint32_t NominalPrescaler;             

 

  uint32_t NominalSyncJumpWidth;         


 

  uint32_t NominalTimeSeg1;              
 

  uint32_t NominalTimeSeg2;              
 

  uint32_t DataPrescaler;                

 

  uint32_t DataSyncJumpWidth;            


 

  uint32_t DataTimeSeg1;                 
 

  uint32_t DataTimeSeg2;                 
 

  uint32_t StdFiltersNbr;                
 

  uint32_t ExtFiltersNbr;                
 

  uint32_t TxFifoQueueMode;              
 

} FDCAN_InitTypeDef;



 
typedef struct
{
  uint32_t IdType;           
 

  uint32_t FilterIndex;      


 

  uint32_t FilterType;       


 

  uint32_t FilterConfig;     
 

  uint32_t FilterID1;        


 

  uint32_t FilterID2;        


 

} FDCAN_FilterTypeDef;



 
typedef struct
{
  uint32_t Identifier;          


 

  uint32_t IdType;              

 

  uint32_t TxFrameType;         
 

  uint32_t DataLength;          
 

  uint32_t ErrorStateIndicator; 
 

  uint32_t BitRateSwitch;       

 

  uint32_t FDFormat;            

 

  uint32_t TxEventFifoControl;  
 

  uint32_t MessageMarker;       

 

} FDCAN_TxHeaderTypeDef;



 
typedef struct
{
  uint32_t Identifier;            


 

  uint32_t IdType;                
 

  uint32_t RxFrameType;           
 

  uint32_t DataLength;            
 

  uint32_t ErrorStateIndicator;   
 

  uint32_t BitRateSwitch;         

 

  uint32_t FDFormat;              

 

  uint32_t RxTimestamp;           

 

  uint32_t FilterIndex;           


 

  uint32_t IsFilterMatchingFrame; 


 

} FDCAN_RxHeaderTypeDef;



 
typedef struct
{
  uint32_t Identifier;          


 

  uint32_t IdType;              
 

  uint32_t TxFrameType;         
 

  uint32_t DataLength;          
 

  uint32_t ErrorStateIndicator; 
 

  uint32_t BitRateSwitch;       

 

  uint32_t FDFormat;            

 

  uint32_t TxTimestamp;         

 

  uint32_t MessageMarker;       

 

  uint32_t EventType;           
 

} FDCAN_TxEventFifoTypeDef;



 
typedef struct
{
  uint32_t FilterList;     


 

  uint32_t FilterIndex;    


 

  uint32_t MessageStorage; 
 

  uint32_t MessageIndex;   




 

} FDCAN_HpMsgStatusTypeDef;



 
typedef struct
{
  uint32_t LastErrorCode;     
 

  uint32_t DataLastErrorCode; 

 

  uint32_t Activity;          
 

  uint32_t ErrorPassive;      


 

  uint32_t Warning;           



 

  uint32_t BusOff;            


 

  uint32_t RxESIflag;         


 

  uint32_t RxBRSflag;         


 

  uint32_t RxFDFflag;         


 

  uint32_t ProtocolException; 


 

  uint32_t TDCvalue;          
 

} FDCAN_ProtocolStatusTypeDef;



 
typedef struct
{
  uint32_t TxErrorCnt;     
 

  uint32_t RxErrorCnt;     
 

  uint32_t RxErrorPassive; 



 

  uint32_t ErrorLogging;   



 

} FDCAN_ErrorCountersTypeDef;



 
typedef struct
{
  uint32_t StandardFilterSA; 
 

  uint32_t ExtendedFilterSA; 
 

  uint32_t RxFIFO0SA;        
 

  uint32_t RxFIFO1SA;        
 

  uint32_t TxEventFIFOSA;    
 

  uint32_t TxFIFOQSA;        
 

} FDCAN_MsgRamAddressTypeDef;



 

typedef struct __FDCAN_HandleTypeDef



{
  FDCAN_GlobalTypeDef         *Instance;         

  FDCAN_InitTypeDef           Init;              

  FDCAN_MsgRamAddressTypeDef  msgRam;            

  uint32_t                    LatestTxFifoQRequest; 
 

  volatile HAL_FDCAN_StateTypeDef State;             

  HAL_LockTypeDef             Lock;              

  volatile uint32_t               ErrorCode;         


  void (* TxEventFifoCallback)(struct __FDCAN_HandleTypeDef *hfdcan, uint32_t TxEventFifoITs);      
  void (* RxFifo0Callback)(struct __FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);              
  void (* RxFifo1Callback)(struct __FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs);              
  void (* TxFifoEmptyCallback)(struct __FDCAN_HandleTypeDef *hfdcan);                               
  void (* TxBufferCompleteCallback)(struct __FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes);  
  void (* TxBufferAbortCallback)(struct __FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes);     
  void (* HighPriorityMessageCallback)(struct __FDCAN_HandleTypeDef *hfdcan);                       
  void (* TimestampWraparoundCallback)(struct __FDCAN_HandleTypeDef *hfdcan);                       
  void (* TimeoutOccurredCallback)(struct __FDCAN_HandleTypeDef *hfdcan);                           
  void (* ErrorCallback)(struct __FDCAN_HandleTypeDef *hfdcan);                                     
  void (* ErrorStatusCallback)(struct __FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs);      

  void (* MspInitCallback)(struct __FDCAN_HandleTypeDef *hfdcan);                                   
  void (* MspDeInitCallback)(struct __FDCAN_HandleTypeDef *hfdcan);                                 



} FDCAN_HandleTypeDef;




 
typedef enum
{
  HAL_FDCAN_TX_FIFO_EMPTY_CB_ID        = 0x00U,     
  HAL_FDCAN_HIGH_PRIO_MESSAGE_CB_ID    = 0x01U,     
  HAL_FDCAN_TIMESTAMP_WRAPAROUND_CB_ID = 0x02U,     
  HAL_FDCAN_TIMEOUT_OCCURRED_CB_ID     = 0x03U,     
  HAL_FDCAN_ERROR_CALLBACK_CB_ID       = 0x04U,     

  HAL_FDCAN_MSPINIT_CB_ID              = 0x05U,     
  HAL_FDCAN_MSPDEINIT_CB_ID            = 0x06U,     

} HAL_FDCAN_CallbackIDTypeDef;



 
typedef  void (*pFDCAN_CallbackTypeDef)(FDCAN_HandleTypeDef *hfdcan);                                          
typedef  void (*pFDCAN_TxEventFifoCallbackTypeDef)(FDCAN_HandleTypeDef *hfdcan, uint32_t TxEventFifoITs);      
typedef  void (*pFDCAN_RxFifo0CallbackTypeDef)(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);              
typedef  void (*pFDCAN_RxFifo1CallbackTypeDef)(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs);              
typedef  void (*pFDCAN_TxBufferCompleteCallbackTypeDef)(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes);  
typedef  void (*pFDCAN_TxBufferAbortCallbackTypeDef)(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes);     
typedef  void (*pFDCAN_ErrorStatusCallbackTypeDef)(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs);      





 

 


 



 
# 521 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_fdcan.h"






 



 





 



 







 



 
# 570 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_fdcan.h"


 



 




 



 




 



 




 



 
# 620 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_fdcan.h"


 



 




 



 




 



 




 



 




 



 






 



 
# 681 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_fdcan.h"


 



 





 



 




 



 




 



 






 



 
# 735 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_fdcan.h"


 



 






 



 




 



 





 



 




 



 




 



 




 



 
# 815 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_fdcan.h"


 



 






 



 




 



 
# 866 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_fdcan.h"


 



 



 





 



 



 



 




 



 





 



 





 



 





 



 
# 940 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_fdcan.h"


 



 





 



 



 
# 985 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_fdcan.h"


 



 
# 1023 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_fdcan.h"


 



 

 


 




 
# 1049 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_fdcan.h"







 









 









 








 









 








 








 




 

 


 



 
 
HAL_StatusTypeDef HAL_FDCAN_Init(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef HAL_FDCAN_DeInit(FDCAN_HandleTypeDef *hfdcan);
void              HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *hfdcan);
void              HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef HAL_FDCAN_EnterPowerDownMode(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef HAL_FDCAN_ExitPowerDownMode(FDCAN_HandleTypeDef *hfdcan);


 
HAL_StatusTypeDef HAL_FDCAN_RegisterCallback(FDCAN_HandleTypeDef *hfdcan, HAL_FDCAN_CallbackIDTypeDef CallbackID,
                                             pFDCAN_CallbackTypeDef pCallback);
HAL_StatusTypeDef HAL_FDCAN_UnRegisterCallback(FDCAN_HandleTypeDef *hfdcan, HAL_FDCAN_CallbackIDTypeDef CallbackID);
HAL_StatusTypeDef HAL_FDCAN_RegisterTxEventFifoCallback(FDCAN_HandleTypeDef *hfdcan,
                                                        pFDCAN_TxEventFifoCallbackTypeDef pCallback);
HAL_StatusTypeDef HAL_FDCAN_UnRegisterTxEventFifoCallback(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef HAL_FDCAN_RegisterRxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                                                    pFDCAN_RxFifo0CallbackTypeDef pCallback);
HAL_StatusTypeDef HAL_FDCAN_UnRegisterRxFifo0Callback(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef HAL_FDCAN_RegisterRxFifo1Callback(FDCAN_HandleTypeDef *hfdcan,
                                                    pFDCAN_RxFifo1CallbackTypeDef pCallback);
HAL_StatusTypeDef HAL_FDCAN_UnRegisterRxFifo1Callback(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef HAL_FDCAN_RegisterTxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan,
                                                             pFDCAN_TxBufferCompleteCallbackTypeDef pCallback);
HAL_StatusTypeDef HAL_FDCAN_UnRegisterTxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef HAL_FDCAN_RegisterTxBufferAbortCallback(FDCAN_HandleTypeDef *hfdcan,
                                                          pFDCAN_TxBufferAbortCallbackTypeDef pCallback);
HAL_StatusTypeDef HAL_FDCAN_UnRegisterTxBufferAbortCallback(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef HAL_FDCAN_RegisterErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan,
                                                        pFDCAN_ErrorStatusCallbackTypeDef pCallback);
HAL_StatusTypeDef HAL_FDCAN_UnRegisterErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan);



 



 
 
HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *hfdcan, FDCAN_FilterTypeDef *sFilterConfig);
HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *hfdcan, uint32_t NonMatchingStd,
                                               uint32_t NonMatchingExt, uint32_t RejectRemoteStd,
                                               uint32_t RejectRemoteExt);
HAL_StatusTypeDef HAL_FDCAN_ConfigExtendedIdMask(FDCAN_HandleTypeDef *hfdcan, uint32_t Mask);
HAL_StatusTypeDef HAL_FDCAN_ConfigRxFifoOverwrite(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo, uint32_t OperationMode);
HAL_StatusTypeDef HAL_FDCAN_ConfigRamWatchdog(FDCAN_HandleTypeDef *hfdcan, uint32_t CounterStartValue);
HAL_StatusTypeDef HAL_FDCAN_ConfigTimestampCounter(FDCAN_HandleTypeDef *hfdcan, uint32_t TimestampPrescaler);
HAL_StatusTypeDef HAL_FDCAN_EnableTimestampCounter(FDCAN_HandleTypeDef *hfdcan, uint32_t TimestampOperation);
HAL_StatusTypeDef HAL_FDCAN_DisableTimestampCounter(FDCAN_HandleTypeDef *hfdcan);
uint16_t          HAL_FDCAN_GetTimestampCounter(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef HAL_FDCAN_ResetTimestampCounter(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef HAL_FDCAN_ConfigTimeoutCounter(FDCAN_HandleTypeDef *hfdcan, uint32_t TimeoutOperation,
                                                 uint32_t TimeoutPeriod);
HAL_StatusTypeDef HAL_FDCAN_EnableTimeoutCounter(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef HAL_FDCAN_DisableTimeoutCounter(FDCAN_HandleTypeDef *hfdcan);
uint16_t          HAL_FDCAN_GetTimeoutCounter(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef HAL_FDCAN_ResetTimeoutCounter(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef HAL_FDCAN_ConfigTxDelayCompensation(FDCAN_HandleTypeDef *hfdcan, uint32_t TdcOffset,
                                                      uint32_t TdcFilter);
HAL_StatusTypeDef HAL_FDCAN_EnableTxDelayCompensation(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef HAL_FDCAN_DisableTxDelayCompensation(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef HAL_FDCAN_EnableISOMode(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef HAL_FDCAN_DisableISOMode(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef HAL_FDCAN_EnableEdgeFiltering(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef HAL_FDCAN_DisableEdgeFiltering(FDCAN_HandleTypeDef *hfdcan);


 



 
 
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef HAL_FDCAN_Stop(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *hfdcan, FDCAN_TxHeaderTypeDef *pTxHeader,
                                                uint8_t *pTxData);
uint32_t HAL_FDCAN_GetLatestTxFifoQRequestBuffer(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef HAL_FDCAN_AbortTxRequest(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndex);
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *hfdcan, uint32_t RxLocation,
                                         FDCAN_RxHeaderTypeDef *pRxHeader, uint8_t *pRxData);
HAL_StatusTypeDef HAL_FDCAN_GetTxEvent(FDCAN_HandleTypeDef *hfdcan, FDCAN_TxEventFifoTypeDef *pTxEvent);
HAL_StatusTypeDef HAL_FDCAN_GetHighPriorityMessageStatus(FDCAN_HandleTypeDef *hfdcan,
                                                         FDCAN_HpMsgStatusTypeDef *HpMsgStatus);
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(FDCAN_HandleTypeDef *hfdcan, FDCAN_ProtocolStatusTypeDef *ProtocolStatus);
HAL_StatusTypeDef HAL_FDCAN_GetErrorCounters(FDCAN_HandleTypeDef *hfdcan, FDCAN_ErrorCountersTypeDef *ErrorCounters);
uint32_t HAL_FDCAN_IsTxBufferMessagePending(FDCAN_HandleTypeDef *hfdcan, uint32_t TxBufferIndex);
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo);
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_HandleTypeDef *hfdcan);
uint32_t HAL_FDCAN_IsRestrictedOperationMode(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef HAL_FDCAN_ExitRestrictedOperationMode(FDCAN_HandleTypeDef *hfdcan);


 



 
 
HAL_StatusTypeDef HAL_FDCAN_ConfigInterruptLines(FDCAN_HandleTypeDef *hfdcan, uint32_t ITList, uint32_t InterruptLine);
HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *hfdcan, uint32_t ActiveITs,
                                                 uint32_t BufferIndexes);
HAL_StatusTypeDef HAL_FDCAN_DeactivateNotification(FDCAN_HandleTypeDef *hfdcan, uint32_t InactiveITs);
void              HAL_FDCAN_IRQHandler(FDCAN_HandleTypeDef *hfdcan);


 



 
 
void HAL_FDCAN_TxEventFifoCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t TxEventFifoITs);
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs);
void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef *hfdcan);
void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes);
void HAL_FDCAN_TxBufferAbortCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes);
void HAL_FDCAN_HighPriorityMessageCallback(FDCAN_HandleTypeDef *hfdcan);
void HAL_FDCAN_TimestampWraparoundCallback(FDCAN_HandleTypeDef *hfdcan);
void HAL_FDCAN_TimeoutOccurredCallback(FDCAN_HandleTypeDef *hfdcan);
void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan);
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs);


 



 
 
uint32_t HAL_FDCAN_GetError(FDCAN_HandleTypeDef *hfdcan);
HAL_FDCAN_StateTypeDef HAL_FDCAN_GetState(FDCAN_HandleTypeDef *hfdcan);


 



 

 


 



 

 


 



 

 


 



 

 


 
# 1418 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_fdcan.h"


 

 


 



 

 


 



 


 



 









 
# 256 "..\\..\\User\\stm32g4xx_hal_conf.h"


# 1 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_flash.h"
















 

 







 
# 29 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_flash.h"



 



 

 


 



 
typedef struct
{
  uint32_t TypeErase;   
 
  uint32_t Banks;       

 
  uint32_t Page;        

 
  uint32_t NbPages;     
 
} FLASH_EraseInitTypeDef;



 
typedef struct
{
  uint32_t OptionType;     
 
  uint32_t WRPArea;        

 
  uint32_t WRPStartOffset; 
 
  uint32_t WRPEndOffset;   
 
  uint32_t RDPLevel;       
 
  uint32_t USERType;       
 
  uint32_t USERConfig;     







 
  uint32_t PCROPConfig;    

 
  uint32_t PCROPStartAddr; 

 
  uint32_t PCROPEndAddr;   
 
  uint32_t BootEntryPoint; 
 
  uint32_t SecBank;        




 
  uint32_t SecSize;        


 
} FLASH_OBProgramInitTypeDef;



 
typedef enum
{
  FLASH_PROC_NONE = 0,
  FLASH_PROC_PAGE_ERASE,
  FLASH_PROC_MASS_ERASE,
  FLASH_PROC_PROGRAM,
  FLASH_PROC_PROGRAM_LAST
} FLASH_ProcedureTypeDef;



 
typedef enum
{
  FLASH_CACHE_DISABLED = 0,
  FLASH_CACHE_ICACHE_ENABLED,
  FLASH_CACHE_DCACHE_ENABLED,
  FLASH_CACHE_ICACHE_DCACHE_ENABLED
} FLASH_CacheTypeDef;



 
typedef struct
{
  HAL_LockTypeDef             Lock;               
  volatile uint32_t               ErrorCode;          
  volatile FLASH_ProcedureTypeDef ProcedureOnGoing;   
  volatile uint32_t               Address;            
  volatile uint32_t               Bank;               
  volatile uint32_t               Page;               
  volatile uint32_t               NbPagesToErase;     
  volatile FLASH_CacheTypeDef     CacheToReactivate;  
} FLASH_ProcessTypeDef;



 

 


 



 
# 176 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_flash.h"


 



 




 



 
# 199 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_flash.h"


 




 







 



 
# 225 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_flash.h"


 



 
# 238 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_flash.h"


 



 




 



 






 



 
# 287 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_flash.h"


 



 







 



 




 



 




 



 




 



 




 



 




 



 




 



 




 

# 385 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_flash.h"

# 396 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_flash.h"



 




 



 




 



 




 



 




 



 




 



 





 



 




 



 






 



 
# 491 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_flash.h"


 



 













 



 
# 534 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_flash.h"

# 545 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_flash.h"


 




 






 



 

 



 






















 






















 





 





 





 





 





 





 






 








 








 









 








 





 




 




 










 













 


























 



























 





 

 
# 1 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_flash_ex.h"
















 

 







 
# 29 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_flash_ex.h"



 



 

 

 

 

 


 

 


 
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *pEraseInit, uint32_t *PageError);
HAL_StatusTypeDef HAL_FLASHEx_Erase_IT(FLASH_EraseInitTypeDef *pEraseInit);
HAL_StatusTypeDef HAL_FLASHEx_OBProgram(FLASH_OBProgramInitTypeDef *pOBInit);
void              HAL_FLASHEx_OBGetConfig(FLASH_OBProgramInitTypeDef *pOBInit);
HAL_StatusTypeDef HAL_FLASHEx_EnableSecMemProtection(uint32_t Bank);
void              HAL_FLASHEx_EnableDebugger(void);
void              HAL_FLASHEx_DisableDebugger(void);


 



 



 
void              FLASH_PageErase(uint32_t Page, uint32_t Banks);
void              FLASH_FlushCaches(void);


 



 



 







 
# 801 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_flash.h"
# 1 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_flash_ramfunc.h"
















 

 







 
# 29 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_flash_ramfunc.h"



 



 

 
 
 


 



 
 
 HAL_StatusTypeDef HAL_FLASHEx_EnableRunPowerDown(void);
 HAL_StatusTypeDef HAL_FLASHEx_DisableRunPowerDown(void);





 



 



 



 







 
# 802 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_flash.h"

 


 
extern FLASH_ProcessTypeDef pFlash;


 

 


 

 


 
HAL_StatusTypeDef  HAL_FLASH_Program(uint32_t TypeProgram, uint32_t Address, uint64_t Data);
HAL_StatusTypeDef  HAL_FLASH_Program_IT(uint32_t TypeProgram, uint32_t Address, uint64_t Data);
 
void               HAL_FLASH_IRQHandler(void);
 
void               HAL_FLASH_EndOfOperationCallback(uint32_t ReturnValue);
void               HAL_FLASH_OperationErrorCallback(uint32_t ReturnValue);


 

 


 
HAL_StatusTypeDef  HAL_FLASH_Unlock(void);
HAL_StatusTypeDef  HAL_FLASH_Lock(void);
 
HAL_StatusTypeDef  HAL_FLASH_OB_Unlock(void);
HAL_StatusTypeDef  HAL_FLASH_OB_Lock(void);
HAL_StatusTypeDef  HAL_FLASH_OB_Launch(void);


 

 


 
uint32_t HAL_FLASH_GetError(void);


 



 



 
HAL_StatusTypeDef  FLASH_WaitForLastOperation(uint32_t Timeout);


 

 


 


# 885 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_flash.h"







 

 


 




# 911 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_flash.h"



















# 936 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_flash.h"
























































# 1000 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_flash.h"


 



 



 







 
# 260 "..\\..\\User\\stm32g4xx_hal_conf.h"














# 1 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_iwdg.h"

















 

 







 
# 30 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_iwdg.h"



 



 

 


 



 
typedef struct
{
  uint32_t Prescaler;  
 

  uint32_t Reload;     
 

  uint32_t Window;     
 

} IWDG_InitTypeDef;



 
typedef struct
{
  IWDG_TypeDef                 *Instance;   

  IWDG_InitTypeDef             Init;        
} IWDG_HandleTypeDef;




 

 


 



 
# 90 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_iwdg.h"


 



 



 



 

 


 





 







 




 

 


 



 
 
HAL_StatusTypeDef     HAL_IWDG_Init(IWDG_HandleTypeDef *hiwdg);


 



 
 
HAL_StatusTypeDef     HAL_IWDG_Refresh(IWDG_HandleTypeDef *hiwdg);


 



 

 


 



 







 

 


 





 






 






 
# 205 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_iwdg.h"





 






 





 



 



 








 
# 276 "..\\..\\User\\stm32g4xx_hal_conf.h"


# 1 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_i2c.h"

















 

 







 
# 30 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_i2c.h"



 



 

 


 




 
typedef struct
{
  uint32_t Timing;              

 

  uint32_t OwnAddress1;         
 

  uint32_t AddressingMode;      
 

  uint32_t DualAddressMode;     
 

  uint32_t OwnAddress2;         
 

  uint32_t OwnAddress2Masks;    
 

  uint32_t GeneralCallMode;     
 

  uint32_t NoStretchMode;       
 

} I2C_InitTypeDef;



 



























 
typedef enum
{
  HAL_I2C_STATE_RESET             = 0x00U,    
  HAL_I2C_STATE_READY             = 0x20U,    
  HAL_I2C_STATE_BUSY              = 0x24U,    
  HAL_I2C_STATE_BUSY_TX           = 0x21U,    
  HAL_I2C_STATE_BUSY_RX           = 0x22U,    
  HAL_I2C_STATE_LISTEN            = 0x28U,    
  HAL_I2C_STATE_BUSY_TX_LISTEN    = 0x29U,   
 
  HAL_I2C_STATE_BUSY_RX_LISTEN    = 0x2AU,   
 
  HAL_I2C_STATE_ABORT             = 0x60U,    
  HAL_I2C_STATE_TIMEOUT           = 0xA0U,    
  HAL_I2C_STATE_ERROR             = 0xE0U     

} HAL_I2C_StateTypeDef;



 


















 
typedef enum
{
  HAL_I2C_MODE_NONE               = 0x00U,    
  HAL_I2C_MODE_MASTER             = 0x10U,    
  HAL_I2C_MODE_SLAVE              = 0x20U,    
  HAL_I2C_MODE_MEM                = 0x40U     

} HAL_I2C_ModeTypeDef;



 




 
# 178 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_i2c.h"


 




 
typedef struct __I2C_HandleTypeDef
{
  I2C_TypeDef                *Instance;       

  I2C_InitTypeDef            Init;            

  uint8_t                    *pBuffPtr;       

  uint16_t                   XferSize;        

  volatile uint16_t              XferCount;       

  volatile uint32_t              XferOptions;    
 

  volatile uint32_t              PreviousState;   

  HAL_StatusTypeDef(*XferISR)(struct __I2C_HandleTypeDef *hi2c, uint32_t ITFlags, uint32_t ITSources);   

  DMA_HandleTypeDef          *hdmatx;         

  DMA_HandleTypeDef          *hdmarx;         

  HAL_LockTypeDef            Lock;            

  volatile HAL_I2C_StateTypeDef  State;           

  volatile HAL_I2C_ModeTypeDef   Mode;            

  volatile uint32_t              ErrorCode;       

  volatile uint32_t              AddrEventCount;  

# 236 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_i2c.h"
} I2C_HandleTypeDef;

# 266 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_i2c.h"


 



 
 



 



 
# 288 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_i2c.h"



 




 



 




 



 




 



 
# 327 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_i2c.h"


 



 




 



 




 



 




 



 




 



 





 



 






 






 
# 401 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_i2c.h"


 



 
# 424 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_i2c.h"


 



 

 



 




 
# 451 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_i2c.h"














 















 















 

























 




















 






 





 





 



 

 
# 1 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_i2c_ex.h"

















 

 







 
# 30 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_i2c_ex.h"



 



 

 
 


 



 




 



 
# 70 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_i2c_ex.h"


 



 

 


 



 

 


 



 
 
HAL_StatusTypeDef HAL_I2CEx_ConfigAnalogFilter(I2C_HandleTypeDef *hi2c, uint32_t AnalogFilter);
HAL_StatusTypeDef HAL_I2CEx_ConfigDigitalFilter(I2C_HandleTypeDef *hi2c, uint32_t DigitalFilter);


 



 
HAL_StatusTypeDef HAL_I2CEx_EnableWakeUp(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef HAL_I2CEx_DisableWakeUp(I2C_HandleTypeDef *hi2c);


 



 
void HAL_I2CEx_EnableFastModePlus(uint32_t ConfigFastModePlus);
void HAL_I2CEx_DisableFastModePlus(uint32_t ConfigFastModePlus);


 




 

 


 



 

 


 





# 152 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_i2c_ex.h"


 

 


 
 


 



 



 







 
# 571 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_i2c.h"

 


 



 
 
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c);

 
# 595 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_i2c.h"


 



 
 
 
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size,
                                          uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size,
                                         uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Slave_Transmit(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Slave_Receive(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress,
                                    uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress,
                                   uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint32_t Trials,
                                        uint32_t Timeout);

 
HAL_StatusTypeDef HAL_I2C_Master_Transmit_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData,
                                             uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Master_Receive_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData,
                                            uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Slave_Transmit_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Slave_Receive_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Mem_Write_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress,
                                       uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Mem_Read_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress,
                                      uint16_t MemAddSize, uint8_t *pData, uint16_t Size);

HAL_StatusTypeDef HAL_I2C_Master_Seq_Transmit_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData,
                                                 uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Master_Seq_Receive_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData,
                                                uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Slave_Seq_Transmit_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size,
                                                uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Slave_Seq_Receive_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size,
                                               uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_EnableListen_IT(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef HAL_I2C_DisableListen_IT(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef HAL_I2C_Master_Abort_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress);

 
HAL_StatusTypeDef HAL_I2C_Master_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData,
                                              uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Master_Receive_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData,
                                             uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Slave_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Slave_Receive_DMA(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Mem_Write_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress,
                                        uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Mem_Read_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress,
                                       uint16_t MemAddSize, uint8_t *pData, uint16_t Size);

HAL_StatusTypeDef HAL_I2C_Master_Seq_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData,
                                                  uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Master_Seq_Receive_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData,
                                                 uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Slave_Seq_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size,
                                                 uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Slave_Seq_Receive_DMA(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size,
                                                uint32_t XferOptions);


 



 
 
void HAL_I2C_EV_IRQHandler(I2C_HandleTypeDef *hi2c);
void HAL_I2C_ER_IRQHandler(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode);
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c);


 



 
 
HAL_I2C_StateTypeDef HAL_I2C_GetState(I2C_HandleTypeDef *hi2c);
HAL_I2C_ModeTypeDef  HAL_I2C_GetMode(I2C_HandleTypeDef *hi2c);
uint32_t             HAL_I2C_GetError(I2C_HandleTypeDef *hi2c);



 



 

 


 



 

 


 







# 729 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_i2c.h"



















# 755 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_i2c.h"




























 

 


 
 


 



 



 








 
# 280 "..\\..\\User\\stm32g4xx_hal_conf.h"


# 1 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_i2s.h"

















 

 







 
# 30 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_i2s.h"




 



 

 


 



 
typedef struct
{
  uint32_t Mode;                
 

  uint32_t Standard;            
 

  uint32_t DataFormat;          
 

  uint32_t MCLKOutput;          
 

  uint32_t AudioFreq;           
 

  uint32_t CPOL;                
 
} I2S_InitTypeDef;



 
typedef enum
{
  HAL_I2S_STATE_RESET      = 0x00U,   
  HAL_I2S_STATE_READY      = 0x01U,   
  HAL_I2S_STATE_BUSY       = 0x02U,   
  HAL_I2S_STATE_BUSY_TX    = 0x03U,   
  HAL_I2S_STATE_BUSY_RX    = 0x04U,   
  HAL_I2S_STATE_TIMEOUT    = 0x06U,   
  HAL_I2S_STATE_ERROR      = 0x07U    
} HAL_I2S_StateTypeDef;



 



typedef struct

{
  SPI_TypeDef                *Instance;     

  I2S_InitTypeDef            Init;          

  uint16_t                   *pTxBuffPtr;   

  volatile uint16_t              TxXferSize;    

  volatile uint16_t              TxXferCount;   

  uint16_t                   *pRxBuffPtr;   

  volatile uint16_t              RxXferSize;    

  volatile uint16_t              RxXferCount;  




 
  DMA_HandleTypeDef          *hdmatx;       

  DMA_HandleTypeDef          *hdmarx;       

  volatile HAL_LockTypeDef       Lock;          

  volatile HAL_I2S_StateTypeDef  State;         

  volatile uint32_t              ErrorCode;    
 

# 133 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_i2s.h"
} I2S_HandleTypeDef;

# 157 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_i2s.h"


 

 


 


 
# 178 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_i2s.h"


 



 






 



 







 



 






 



 




 



 
# 238 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_i2s.h"


 



 




 



 





 



 














 



 

 


 




 
# 302 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_i2s.h"




 





 










 










 











 















 





 
# 375 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_i2s.h"



 








 







 

 


 



 
 
HAL_StatusTypeDef HAL_I2S_Init(I2S_HandleTypeDef *hi2s);
HAL_StatusTypeDef HAL_I2S_DeInit(I2S_HandleTypeDef *hi2s);
void HAL_I2S_MspInit(I2S_HandleTypeDef *hi2s);
void HAL_I2S_MspDeInit(I2S_HandleTypeDef *hi2s);

 







 



 
 
 
HAL_StatusTypeDef HAL_I2S_Transmit(I2S_HandleTypeDef *hi2s, uint16_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2S_Receive(I2S_HandleTypeDef *hi2s, uint16_t *pData, uint16_t Size, uint32_t Timeout);

 
HAL_StatusTypeDef HAL_I2S_Transmit_IT(I2S_HandleTypeDef *hi2s, uint16_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2S_Receive_IT(I2S_HandleTypeDef *hi2s, uint16_t *pData, uint16_t Size);
void HAL_I2S_IRQHandler(I2S_HandleTypeDef *hi2s);

 
HAL_StatusTypeDef HAL_I2S_Transmit_DMA(I2S_HandleTypeDef *hi2s, uint16_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2S_Receive_DMA(I2S_HandleTypeDef *hi2s, uint16_t *pData, uint16_t Size);

HAL_StatusTypeDef HAL_I2S_DMAPause(I2S_HandleTypeDef *hi2s);
HAL_StatusTypeDef HAL_I2S_DMAResume(I2S_HandleTypeDef *hi2s);
HAL_StatusTypeDef HAL_I2S_DMAStop(I2S_HandleTypeDef *hi2s);

 
void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s);
void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s);
void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s);
void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s);
void HAL_I2S_ErrorCallback(I2S_HandleTypeDef *hi2s);


 



 
 
HAL_I2S_StateTypeDef HAL_I2S_GetState(I2S_HandleTypeDef *hi2s);
uint32_t HAL_I2S_GetError(I2S_HandleTypeDef *hi2s);


 



 

 
 
 
 


 












 











 







 



























 





 



 



 








 
# 284 "..\\..\\User\\stm32g4xx_hal_conf.h"






















# 1 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_pwr.h"

















 

 







 
# 30 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_pwr.h"



 



 

 



 



 
typedef struct
{
  uint32_t PVDLevel;   
 

  uint32_t Mode;      
 
}PWR_PVDTypeDef;




 

 



 




 
# 80 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_pwr.h"


 



 
# 94 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_pwr.h"


 






 




 



 




 



 




 




 



 



 



 



 

 


 








































 





















 






 





 





 





 





 





 





 






 






 









 









 





 





 




 


 


 






# 329 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_pwr.h"










 

 
# 1 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_pwr_ex.h"

















 

 







 
# 30 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_pwr_ex.h"



 



 


 



 




 
typedef struct
{
  uint32_t PVMType;   
 
  uint32_t Mode;      
 
}PWR_PVMTypeDef;



 

 



 



 



 




 
# 95 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_pwr_ex.h"


 



 
# 110 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_pwr_ex.h"


 



 
# 124 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_pwr_ex.h"


 





 







 




 




 



 




 



 
# 180 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_pwr_ex.h"


 



 
# 194 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_pwr_ex.h"


 



 
# 209 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_pwr_ex.h"


 



 
# 224 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_pwr_ex.h"


 










 
# 246 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_pwr_ex.h"

# 259 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_pwr_ex.h"


 



 

 


 





 





 





 





 





 





 





 






 






 









 









 





 





 









 





 





 





 





 





 





 






 






 









 









 





 





 








 





 





 





 





 





 





 






 






 









 









 





 





 








 





 





 





 





 





 





 






 






 









 









 





 





 





















 
# 662 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_pwr_ex.h"



 

 


 

# 687 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_pwr_ex.h"






# 700 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_pwr_ex.h"

# 709 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_pwr_ex.h"








# 725 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_pwr_ex.h"




 




 



 


 
uint32_t HAL_PWREx_GetVoltageRange(void);
HAL_StatusTypeDef HAL_PWREx_ControlVoltageScaling(uint32_t VoltageScaling);
void HAL_PWREx_EnableBatteryCharging(uint32_t ResistorSelection);
void HAL_PWREx_DisableBatteryCharging(void);
void HAL_PWREx_EnableInternalWakeUpLine(void);
void HAL_PWREx_DisableInternalWakeUpLine(void);
HAL_StatusTypeDef HAL_PWREx_EnableGPIOPullUp(uint32_t GPIO, uint32_t GPIONumber);
HAL_StatusTypeDef HAL_PWREx_DisableGPIOPullUp(uint32_t GPIO, uint32_t GPIONumber);
HAL_StatusTypeDef HAL_PWREx_EnableGPIOPullDown(uint32_t GPIO, uint32_t GPIONumber);
HAL_StatusTypeDef HAL_PWREx_DisableGPIOPullDown(uint32_t GPIO, uint32_t GPIONumber);
void HAL_PWREx_EnablePullUpPullDownConfig(void);
void HAL_PWREx_DisablePullUpPullDownConfig(void);
void HAL_PWREx_EnableSRAM2ContentRetention(void);
void HAL_PWREx_DisableSRAM2ContentRetention(void);

void HAL_PWREx_EnablePVM1(void);
void HAL_PWREx_DisablePVM1(void);


void HAL_PWREx_EnablePVM2(void);
void HAL_PWREx_DisablePVM2(void);

void HAL_PWREx_EnablePVM3(void);
void HAL_PWREx_DisablePVM3(void);
void HAL_PWREx_EnablePVM4(void);
void HAL_PWREx_DisablePVM4(void);
HAL_StatusTypeDef HAL_PWREx_ConfigPVM(PWR_PVMTypeDef *sConfigPVM);

 
void HAL_PWREx_EnableLowPowerRunMode(void);
HAL_StatusTypeDef HAL_PWREx_DisableLowPowerRunMode(void);
void HAL_PWREx_EnterSTOP0Mode(uint8_t STOPEntry);
void HAL_PWREx_EnterSTOP1Mode(uint8_t STOPEntry);
void HAL_PWREx_EnterSHUTDOWNMode(void);

void HAL_PWREx_PVD_PVM_IRQHandler(void);

void HAL_PWREx_PVM1Callback(void);


void HAL_PWREx_PVM2Callback(void);

void HAL_PWREx_PVM3Callback(void);
void HAL_PWREx_PVM4Callback(void);


void HAL_PWREx_EnableUCPDStandbyMode(void);
void HAL_PWREx_DisableUCPDStandbyMode(void);


void HAL_PWREx_EnableUCPDDeadBattery(void);
void HAL_PWREx_DisableUCPDDeadBattery(void);




 



 



 



 








 
# 343 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_pwr.h"

 



 



 

 
void HAL_PWR_DeInit(void);
void HAL_PWR_EnableBkUpAccess(void);
void HAL_PWR_DisableBkUpAccess(void);



 



 

 
HAL_StatusTypeDef HAL_PWR_ConfigPVD(PWR_PVDTypeDef *sConfigPVD);
void HAL_PWR_EnablePVD(void);
void HAL_PWR_DisablePVD(void);


 
void HAL_PWR_EnableWakeUpPin(uint32_t WakeUpPinPolarity);
void HAL_PWR_DisableWakeUpPin(uint32_t WakeUpPinx);

 
void HAL_PWR_EnterSLEEPMode(uint32_t Regulator, uint8_t SLEEPEntry);
void HAL_PWR_EnterSTOPMode(uint32_t Regulator, uint8_t STOPEntry);
void HAL_PWR_EnterSTANDBYMode(void);

void HAL_PWR_EnableSleepOnExit(void);
void HAL_PWR_DisableSleepOnExit(void);
void HAL_PWR_EnableSEVOnPend(void);
void HAL_PWR_DisableSEVOnPend(void);

void HAL_PWR_PVDCallback(void);




 



 



 



 








 
# 308 "..\\..\\User\\stm32g4xx_hal_conf.h"


































# 1 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim.h"

















 

 







 
# 30 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim.h"



 



 

 


 



 
typedef struct
{
  uint32_t Prescaler;         

 

  uint32_t CounterMode;       
 

  uint32_t Period;            



 

  uint32_t ClockDivision;     
 

  uint32_t RepetitionCounter;  






 

  uint32_t AutoReloadPreload;  
 
} TIM_Base_InitTypeDef;



 
typedef struct
{
  uint32_t OCMode;        
 

  uint32_t Pulse;         


 

  uint32_t OCPolarity;    
 

  uint32_t OCNPolarity;   

 

  uint32_t OCFastMode;    

 


  uint32_t OCIdleState;   

 

  uint32_t OCNIdleState;  

 
} TIM_OC_InitTypeDef;



 
typedef struct
{
  uint32_t OCMode;        
 

  uint32_t Pulse;         


 

  uint32_t OCPolarity;    
 

  uint32_t OCNPolarity;   

 

  uint32_t OCIdleState;   

 

  uint32_t OCNIdleState;  

 

  uint32_t ICPolarity;    
 

  uint32_t ICSelection;   
 

  uint32_t ICFilter;      
 
} TIM_OnePulse_InitTypeDef;



 
typedef struct
{
  uint32_t  ICPolarity;  
 

  uint32_t ICSelection;  
 

  uint32_t ICPrescaler;  
 

  uint32_t ICFilter;     
 
} TIM_IC_InitTypeDef;



 
typedef struct
{
  uint32_t EncoderMode;   
 

  uint32_t IC1Polarity;   
 

  uint32_t IC1Selection;  
 

  uint32_t IC1Prescaler;  
 

  uint32_t IC1Filter;     
 

  uint32_t IC2Polarity;   
 

  uint32_t IC2Selection;  
 

  uint32_t IC2Prescaler;  
 

  uint32_t IC2Filter;     
 
} TIM_Encoder_InitTypeDef;



 
typedef struct
{
  uint32_t ClockSource;     
 
  uint32_t ClockPolarity;   
 
  uint32_t ClockPrescaler;  
 
  uint32_t ClockFilter;     
 
} TIM_ClockConfigTypeDef;



 
typedef struct
{
  uint32_t ClearInputState;      
 
  uint32_t ClearInputSource;     
 
  uint32_t ClearInputPolarity;   
 
  uint32_t ClearInputPrescaler;  
 
  uint32_t ClearInputFilter;     
 
} TIM_ClearInputConfigTypeDef;





 
typedef struct
{
  uint32_t  MasterOutputTrigger;   
 
  uint32_t  MasterOutputTrigger2;  
 
  uint32_t  MasterSlaveMode;       





 
} TIM_MasterConfigTypeDef;



 
typedef struct
{
  uint32_t  SlaveMode;         
 
  uint32_t  InputTrigger;      
 
  uint32_t  TriggerPolarity;   
 
  uint32_t  TriggerPrescaler;  
 
  uint32_t  TriggerFilter;     
 

} TIM_SlaveConfigTypeDef;





 
typedef struct
{
  uint32_t OffStateRunMode;      
 
  uint32_t OffStateIDLEMode;     
 
  uint32_t LockLevel;            
 
  uint32_t DeadTime;             
 
  uint32_t BreakState;           
 
  uint32_t BreakPolarity;        
 
  uint32_t BreakFilter;          
 
  uint32_t BreakAFMode;          
 
  uint32_t Break2State;          
 
  uint32_t Break2Polarity;       
 
  uint32_t Break2Filter;         
 
  uint32_t Break2AFMode;         
 
  uint32_t AutomaticOutput;      
 
} TIM_BreakDeadTimeConfigTypeDef;



 
typedef enum
{
  HAL_TIM_STATE_RESET             = 0x00U,     
  HAL_TIM_STATE_READY             = 0x01U,     
  HAL_TIM_STATE_BUSY              = 0x02U,     
  HAL_TIM_STATE_TIMEOUT           = 0x03U,     
  HAL_TIM_STATE_ERROR             = 0x04U      
} HAL_TIM_StateTypeDef;



 
typedef enum
{
  HAL_TIM_CHANNEL_STATE_RESET             = 0x00U,     
  HAL_TIM_CHANNEL_STATE_READY             = 0x01U,     
  HAL_TIM_CHANNEL_STATE_BUSY              = 0x02U,     
} HAL_TIM_ChannelStateTypeDef;



 
typedef enum
{
  HAL_DMA_BURST_STATE_RESET             = 0x00U,     
  HAL_DMA_BURST_STATE_READY             = 0x01U,     
  HAL_DMA_BURST_STATE_BUSY              = 0x02U,     
} HAL_TIM_DMABurstStateTypeDef;



 
typedef enum
{
  HAL_TIM_ACTIVE_CHANNEL_1        = 0x01U,     
  HAL_TIM_ACTIVE_CHANNEL_2        = 0x02U,     
  HAL_TIM_ACTIVE_CHANNEL_3        = 0x04U,     
  HAL_TIM_ACTIVE_CHANNEL_4        = 0x08U,     
  HAL_TIM_ACTIVE_CHANNEL_5        = 0x10U,     
  HAL_TIM_ACTIVE_CHANNEL_6        = 0x20U,     
  HAL_TIM_ACTIVE_CHANNEL_CLEARED  = 0x00U      
} HAL_TIM_ActiveChannel;



 



typedef struct

{
  TIM_TypeDef                        *Instance;          
  TIM_Base_InitTypeDef               Init;               
  HAL_TIM_ActiveChannel              Channel;            
  DMA_HandleTypeDef                  *hdma[7];          
 
  HAL_LockTypeDef                    Lock;               
  volatile HAL_TIM_StateTypeDef          State;              
  volatile HAL_TIM_ChannelStateTypeDef   ChannelState[6];    
  volatile HAL_TIM_ChannelStateTypeDef   ChannelNState[4];   
  volatile HAL_TIM_DMABurstStateTypeDef  DMABurstState;      

# 406 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim.h"
} TIM_HandleTypeDef;

# 455 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim.h"



 
 

 


 



 
# 484 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim.h"


 



 
# 518 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim.h"


 



 
# 534 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim.h"


 



 





 



 




 



 






 



 







 



 




 



 





 



 




 



 





 



 




 



 




 



 




 



 




 



 




 



 




 



 





 



 




 



 







 



 






 



 




 



 
# 735 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim.h"


 



 
# 754 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim.h"


 



 




 



 
# 777 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim.h"


 



 
# 804 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim.h"


 



 
# 818 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim.h"


 



 
# 847 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim.h"


 



 







 



 






 



 




 



 






 



 




 



 




 


 






 



 




 



 




 



 




 



 




 



 




 



 




 



 





 



 






 



 
# 1009 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim.h"


 



 
# 1032 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim.h"


 



 




 



 
# 1055 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim.h"


 



 
# 1078 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim.h"


 



 
# 1106 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim.h"


 



 







 



 






 



 




 



 
# 1171 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim.h"


 



 
# 1185 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim.h"


 



 






 



 






 



 
 

 


 




 
# 1270 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim.h"





 






 






 
# 1300 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim.h"






 
# 1317 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim.h"






 



















 



















 














 














 



























 



























 




















 




















 








 







 







 








 







 









 






 







 










 











 
# 1588 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim.h"








 


















 




















 



















 
# 1664 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim.h"













 
# 1685 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim.h"













 
# 1706 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim.h"













 
# 1727 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim.h"

















 
# 1752 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim.h"

















 
# 1777 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim.h"








 












 
















 








 
 

 


 

 




 
 

 


 
# 1862 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim.h"

# 1890 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim.h"





















































# 1952 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim.h"



# 1962 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim.h"









# 2041 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim.h"































































# 2113 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim.h"

# 2131 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim.h"




# 2142 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim.h"

# 2149 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim.h"

# 2160 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim.h"

# 2213 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim.h"

















# 2256 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim.h"







































# 2302 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim.h"

# 2310 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim.h"

# 2319 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim.h"













# 2338 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim.h"



 
 

 
# 1 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim_ex.h"

















 

 







 
# 30 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim_ex.h"



 



 

 


 



 

typedef struct
{
  uint32_t IC1Polarity;         
 

  uint32_t IC1Prescaler;        
 

  uint32_t IC1Filter;           
 

  uint32_t Commutation_Delay;   
 
} TIM_HallSensor_InitTypeDef;



 
typedef struct
{
  uint32_t Source;         
 
  uint32_t Enable;         
 
  uint32_t Polarity;       
 
}
TIMEx_BreakInputConfigTypeDef;



 
typedef struct
{
  uint32_t Polarity;                  
 
  uint32_t Prescaler;                 
 
  uint32_t Filter;                    
 
  FunctionalState  FirstIndexEnable;  
 
  uint32_t Position;                  
 
  uint32_t Direction;                 
 
} TIMEx_EncoderIndexConfigTypeDef ;



 
 

 


 



 
# 131 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim_ex.h"

# 152 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim_ex.h"

# 172 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim_ex.h"

# 191 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim_ex.h"

# 210 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim_ex.h"

# 233 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim_ex.h"

# 258 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim_ex.h"


 



 




 



 
# 288 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim_ex.h"


 



 




 



 




 



 







# 328 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim_ex.h"

# 337 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim_ex.h"









# 360 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim_ex.h"

# 375 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim_ex.h"





# 394 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim_ex.h"

# 409 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim_ex.h"












# 455 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim_ex.h"









# 474 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim_ex.h"

# 484 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim_ex.h"


# 495 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim_ex.h"


# 506 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim_ex.h"


# 515 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim_ex.h"


 



 




 



 
# 537 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim_ex.h"


 



 





 



 




 



 






 



 
 

 


 







 










 











 










 












 












 













 






 
 

 


 





# 697 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim_ex.h"













# 1760 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim_ex.h"

# 1839 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim_ex.h"

# 1901 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim_ex.h"






























# 1937 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim_ex.h"







 
 

 


 




 
 
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Init(TIM_HandleTypeDef *htim, TIM_HallSensor_InitTypeDef *sConfig);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_DeInit(TIM_HandleTypeDef *htim);

void HAL_TIMEx_HallSensor_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIMEx_HallSensor_MspDeInit(TIM_HandleTypeDef *htim);

 
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Start(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Stop(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Start_IT(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Stop_IT(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Start_DMA(TIM_HandleTypeDef *htim, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Stop_DMA(TIM_HandleTypeDef *htim);


 




 
 
 
HAL_StatusTypeDef HAL_TIMEx_OCN_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_OCN_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);

 
HAL_StatusTypeDef HAL_TIMEx_OCN_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_OCN_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);

 
HAL_StatusTypeDef HAL_TIMEx_OCN_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIMEx_OCN_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);


 




 
 
 
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);

 
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);


 




 
 
 
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Start(TIM_HandleTypeDef *htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Stop(TIM_HandleTypeDef *htim, uint32_t OutputChannel);

 
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Start_IT(TIM_HandleTypeDef *htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Stop_IT(TIM_HandleTypeDef *htim, uint32_t OutputChannel);


 




 
 
HAL_StatusTypeDef HAL_TIMEx_ConfigCommutEvent(TIM_HandleTypeDef *htim, uint32_t  InputTrigger,
                                              uint32_t  CommutationSource);
HAL_StatusTypeDef HAL_TIMEx_ConfigCommutEvent_IT(TIM_HandleTypeDef *htim, uint32_t  InputTrigger,
                                                 uint32_t  CommutationSource);
HAL_StatusTypeDef HAL_TIMEx_ConfigCommutEvent_DMA(TIM_HandleTypeDef *htim, uint32_t  InputTrigger,
                                                  uint32_t  CommutationSource);
HAL_StatusTypeDef HAL_TIMEx_MasterConfigSynchronization(TIM_HandleTypeDef *htim,
                                                        TIM_MasterConfigTypeDef *sMasterConfig);
HAL_StatusTypeDef HAL_TIMEx_ConfigBreakDeadTime(TIM_HandleTypeDef *htim,
                                                TIM_BreakDeadTimeConfigTypeDef *sBreakDeadTimeConfig);
HAL_StatusTypeDef HAL_TIMEx_ConfigBreakInput(TIM_HandleTypeDef *htim, uint32_t BreakInput,
                                             TIMEx_BreakInputConfigTypeDef *sBreakInputConfig);
HAL_StatusTypeDef HAL_TIMEx_GroupChannel5(TIM_HandleTypeDef *htim, uint32_t Channels);
HAL_StatusTypeDef HAL_TIMEx_RemapConfig(TIM_HandleTypeDef *htim, uint32_t Remap);
HAL_StatusTypeDef  HAL_TIMEx_TISelection(TIM_HandleTypeDef *htim, uint32_t TISelection, uint32_t Channel);

HAL_StatusTypeDef HAL_TIMEx_DisarmBreakInput(TIM_HandleTypeDef *htim, uint32_t BreakInput);
HAL_StatusTypeDef HAL_TIMEx_ReArmBreakInput(TIM_HandleTypeDef *htim, uint32_t BreakInput);
HAL_StatusTypeDef HAL_TIMEx_DitheringEnable(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIMEx_DitheringDisable(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIMEx_OC_ConfigPulseOnCompare(TIM_HandleTypeDef *htim, uint32_t PulseWidthPrescaler,
                                                    uint32_t PulseWidth);
HAL_StatusTypeDef HAL_TIMEx_ConfigSlaveModePreload(TIM_HandleTypeDef *htim, uint32_t Source);
HAL_StatusTypeDef HAL_TIMEx_EnableSlaveModePreload(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIMEx_DisableSlaveModePreload(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIMEx_EnableDeadTimePreload(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIMEx_DisableDeadTimePreload(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIMEx_ConfigDeadTime(TIM_HandleTypeDef *htim, uint32_t Deadtime);
HAL_StatusTypeDef HAL_TIMEx_ConfigAsymmetricalDeadTime(TIM_HandleTypeDef *htim, uint32_t FallingDeadtime);
HAL_StatusTypeDef HAL_TIMEx_EnableAsymmetricalDeadTime(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIMEx_DisableAsymmetricalDeadTime(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIMEx_ConfigEncoderIndex(TIM_HandleTypeDef *htim,
                                               TIMEx_EncoderIndexConfigTypeDef *sEncoderIndexConfig);
HAL_StatusTypeDef HAL_TIMEx_EnableEncoderIndex(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIMEx_DisableEncoderIndex(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIMEx_EnableEncoderFirstIndex(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIMEx_DisableEncoderFirstIndex(TIM_HandleTypeDef *htim);


 




 
 
void HAL_TIMEx_CommutCallback(TIM_HandleTypeDef *htim);
void HAL_TIMEx_CommutHalfCpltCallback(TIM_HandleTypeDef *htim);
void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef *htim);
void HAL_TIMEx_Break2Callback(TIM_HandleTypeDef *htim);
void HAL_TIMEx_EncoderIndexCallback(TIM_HandleTypeDef *htim);
void HAL_TIMEx_DirectionChangeCallback(TIM_HandleTypeDef *htim);
void HAL_TIMEx_IndexErrorCallback(TIM_HandleTypeDef *htim);
void HAL_TIMEx_TransitionErrorCallback(TIM_HandleTypeDef *htim);


 




 
 
HAL_TIM_StateTypeDef HAL_TIMEx_HallSensor_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_ChannelStateTypeDef HAL_TIMEx_GetChannelNState(TIM_HandleTypeDef *htim,  uint32_t ChannelN);


 



 
 

 


 
void TIMEx_DMACommutationCplt(DMA_HandleTypeDef *hdma);
void TIMEx_DMACommutationHalfCplt(DMA_HandleTypeDef *hdma);


 
 



 



 








 
# 2346 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_tim.h"

 


 




 
 
HAL_StatusTypeDef HAL_TIM_Base_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_Base_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_Base_Stop(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_Base_Stop_IT(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_Base_Start_DMA(TIM_HandleTypeDef *htim, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_Base_Stop_DMA(TIM_HandleTypeDef *htim);


 




 
 
HAL_StatusTypeDef HAL_TIM_OC_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_OC_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_OC_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_OC_MspDeInit(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_OC_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_OC_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_OC_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_OC_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_OC_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_OC_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);


 




 
 
HAL_StatusTypeDef HAL_TIM_PWM_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_PWM_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_PWM_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_PWM_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_PWM_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_PWM_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);


 




 
 
HAL_StatusTypeDef HAL_TIM_IC_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_IC_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_IC_MspDeInit(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_IC_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_IC_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_IC_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_IC_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_IC_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_IC_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);


 




 
 
HAL_StatusTypeDef HAL_TIM_OnePulse_Init(TIM_HandleTypeDef *htim, uint32_t OnePulseMode);
HAL_StatusTypeDef HAL_TIM_OnePulse_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_OnePulse_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_OnePulse_MspDeInit(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_OnePulse_Start(TIM_HandleTypeDef *htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIM_OnePulse_Stop(TIM_HandleTypeDef *htim, uint32_t OutputChannel);
 
HAL_StatusTypeDef HAL_TIM_OnePulse_Start_IT(TIM_HandleTypeDef *htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIM_OnePulse_Stop_IT(TIM_HandleTypeDef *htim, uint32_t OutputChannel);


 




 
 
HAL_StatusTypeDef HAL_TIM_Encoder_Init(TIM_HandleTypeDef *htim,  TIM_Encoder_InitTypeDef *sConfig);
HAL_StatusTypeDef HAL_TIM_Encoder_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_Encoder_MspDeInit(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_Encoder_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_Encoder_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_Encoder_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_Encoder_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData1,
                                            uint32_t *pData2, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_Encoder_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);


 




 
 
void HAL_TIM_IRQHandler(TIM_HandleTypeDef *htim);


 




 
 
HAL_StatusTypeDef HAL_TIM_OC_ConfigChannel(TIM_HandleTypeDef *htim, TIM_OC_InitTypeDef *sConfig, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *htim, TIM_OC_InitTypeDef *sConfig, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_IC_ConfigChannel(TIM_HandleTypeDef *htim, TIM_IC_InitTypeDef *sConfig, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_OnePulse_ConfigChannel(TIM_HandleTypeDef *htim, TIM_OnePulse_InitTypeDef *sConfig,
                                                 uint32_t OutputChannel,  uint32_t InputChannel);
HAL_StatusTypeDef HAL_TIM_ConfigOCrefClear(TIM_HandleTypeDef *htim, TIM_ClearInputConfigTypeDef *sClearInputConfig,
                                           uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_ConfigClockSource(TIM_HandleTypeDef *htim, TIM_ClockConfigTypeDef *sClockSourceConfig);
HAL_StatusTypeDef HAL_TIM_ConfigTI1Input(TIM_HandleTypeDef *htim, uint32_t TI1_Selection);
HAL_StatusTypeDef HAL_TIM_SlaveConfigSynchro(TIM_HandleTypeDef *htim, TIM_SlaveConfigTypeDef *sSlaveConfig);
HAL_StatusTypeDef HAL_TIM_SlaveConfigSynchro_IT(TIM_HandleTypeDef *htim, TIM_SlaveConfigTypeDef *sSlaveConfig);
HAL_StatusTypeDef HAL_TIM_DMABurst_WriteStart(TIM_HandleTypeDef *htim, uint32_t BurstBaseAddress,
                                              uint32_t BurstRequestSrc, uint32_t  *BurstBuffer, uint32_t  BurstLength);
HAL_StatusTypeDef HAL_TIM_DMABurst_MultiWriteStart(TIM_HandleTypeDef *htim, uint32_t BurstBaseAddress,
                                                   uint32_t BurstRequestSrc, uint32_t *BurstBuffer, uint32_t BurstLength,
                                                   uint32_t DataLength);
HAL_StatusTypeDef HAL_TIM_DMABurst_WriteStop(TIM_HandleTypeDef *htim, uint32_t BurstRequestSrc);
HAL_StatusTypeDef HAL_TIM_DMABurst_ReadStart(TIM_HandleTypeDef *htim, uint32_t BurstBaseAddress,
                                             uint32_t BurstRequestSrc, uint32_t  *BurstBuffer, uint32_t  BurstLength);
HAL_StatusTypeDef HAL_TIM_DMABurst_MultiReadStart(TIM_HandleTypeDef *htim, uint32_t BurstBaseAddress,
                                                  uint32_t BurstRequestSrc, uint32_t  *BurstBuffer, uint32_t  BurstLength,
                                                  uint32_t  DataLength);
HAL_StatusTypeDef HAL_TIM_DMABurst_ReadStop(TIM_HandleTypeDef *htim, uint32_t BurstRequestSrc);
HAL_StatusTypeDef HAL_TIM_GenerateEvent(TIM_HandleTypeDef *htim, uint32_t EventSource);
uint32_t HAL_TIM_ReadCapturedValue(TIM_HandleTypeDef *htim, uint32_t Channel);


 




 
 
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_PeriodElapsedHalfCpltCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_IC_CaptureHalfCpltCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_TriggerCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_TriggerHalfCpltCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim);

 








 




 
 
HAL_TIM_StateTypeDef HAL_TIM_Base_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_OC_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_PWM_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_IC_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_OnePulse_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_Encoder_GetState(TIM_HandleTypeDef *htim);

 
HAL_TIM_ActiveChannel HAL_TIM_GetActiveChannel(TIM_HandleTypeDef *htim);
HAL_TIM_ChannelStateTypeDef HAL_TIM_GetChannelState(TIM_HandleTypeDef *htim,  uint32_t Channel);
HAL_TIM_DMABurstStateTypeDef HAL_TIM_DMABurstState(TIM_HandleTypeDef *htim);


 



 
 

 


 
void TIM_Base_SetConfig(TIM_TypeDef *TIMx, TIM_Base_InitTypeDef *Structure);
void TIM_TI1_SetConfig(TIM_TypeDef *TIMx, uint32_t TIM_ICPolarity, uint32_t TIM_ICSelection, uint32_t TIM_ICFilter);
void TIM_OC2_SetConfig(TIM_TypeDef *TIMx, TIM_OC_InitTypeDef *OC_Config);
void TIM_ETR_SetConfig(TIM_TypeDef *TIMx, uint32_t TIM_ExtTRGPrescaler,
                       uint32_t TIM_ExtTRGPolarity, uint32_t ExtTRGFilter);

void TIM_DMADelayPulseHalfCplt(DMA_HandleTypeDef *hdma);
void TIM_DMAError(DMA_HandleTypeDef *hdma);
void TIM_DMACaptureCplt(DMA_HandleTypeDef *hdma);
void TIM_DMACaptureHalfCplt(DMA_HandleTypeDef *hdma);
void TIM_CCxChannelCmd(TIM_TypeDef *TIMx, uint32_t Channel, uint32_t ChannelState);







 
 



 



 







 
# 344 "..\\..\\User\\stm32g4xx_hal_conf.h"


# 1 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_uart.h"

















 

 







 
# 30 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_uart.h"



 



 

 


 



 
typedef struct
{
  uint32_t BaudRate;                















 

  uint32_t WordLength;              
 

  uint32_t StopBits;                
 

  uint32_t Parity;                  




 

  uint32_t Mode;                    
 

  uint32_t HwFlowCtl;               

 

  uint32_t OverSampling;            

 

  uint32_t OneBitSampling;          

 

  uint32_t ClockPrescaler;          
 

} UART_InitTypeDef;



 
typedef struct
{
  uint32_t AdvFeatureInit;        


 

  uint32_t TxPinLevelInvert;      
 

  uint32_t RxPinLevelInvert;      
 

  uint32_t DataInvert;            

 

  uint32_t Swap;                  
 

  uint32_t OverrunDisable;        
 

  uint32_t DMADisableonRxError;   
 

  uint32_t AutoBaudRateEnable;    
 

  uint32_t AutoBaudRateMode;      

 

  uint32_t MSBFirst;              
 
} UART_AdvFeatureInitTypeDef;








































 
typedef uint32_t HAL_UART_StateTypeDef;



 
typedef enum
{
  UART_CLOCKSOURCE_PCLK1      = 0x00U,     
  UART_CLOCKSOURCE_PCLK2      = 0x01U,     
  UART_CLOCKSOURCE_HSI        = 0x02U,     
  UART_CLOCKSOURCE_SYSCLK     = 0x04U,     
  UART_CLOCKSOURCE_LSE        = 0x08U,     
  UART_CLOCKSOURCE_UNDEFINED  = 0x10U      
} UART_ClockSourceTypeDef;









 
typedef uint32_t HAL_UART_RxTypeTypeDef;



 
typedef struct __UART_HandleTypeDef
{
  USART_TypeDef            *Instance;                 

  UART_InitTypeDef         Init;                      

  UART_AdvFeatureInitTypeDef AdvancedInit;            

  uint8_t                  *pTxBuffPtr;               

  uint16_t                 TxXferSize;                

  volatile uint16_t            TxXferCount;               

  uint8_t                  *pRxBuffPtr;               

  uint16_t                 RxXferSize;                

  volatile uint16_t            RxXferCount;               

  uint16_t                 Mask;                      

  uint32_t                 FifoMode;                 
 

  uint16_t                 NbRxDataToProcess;         

  uint16_t                 NbTxDataToProcess;         

  volatile HAL_UART_RxTypeTypeDef ReceptionType;          

  void (*RxISR)(struct __UART_HandleTypeDef *huart);  

  void (*TxISR)(struct __UART_HandleTypeDef *huart);  

  DMA_HandleTypeDef        *hdmatx;                   

  DMA_HandleTypeDef        *hdmarx;                   

  HAL_LockTypeDef           Lock;                     

  volatile HAL_UART_StateTypeDef    gState;              

 

  volatile HAL_UART_StateTypeDef    RxState;             
 

  volatile uint32_t                 ErrorCode;            

# 276 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_uart.h"

} UART_HandleTypeDef;

# 310 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_uart.h"



 

 


 



 
# 340 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_uart.h"


 



 
# 354 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_uart.h"






 



 






 



 





 



 






 



 





 



 




 



 




 



 




 



 
# 446 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_uart.h"


 



 
# 461 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_uart.h"


 



 




 



 




 



 




 



 




 



 




 



 




 



 




 



 







 



 
# 552 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_uart.h"


 



 




 



 




 



 




 



 




 



 




 



 




 



 




 



 






 



 




 



 




 



 



 



 






 



 




 



 




 



 




 



 



 



 



 





 
# 742 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_uart.h"


 


















 
# 780 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_uart.h"








 



 
# 805 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_uart.h"


 



 






 


 
extern const uint16_t UARTPrescTable[12];



 

 


 




 
# 850 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_uart.h"




 























 





 





 





 





 





 





 

































 
























 
# 983 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_uart.h"























 
# 1016 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_uart.h"























 

























 
# 1073 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_uart.h"



















 












 





 





 





 





 














 


















 


















 


















 







 

 


 



 
# 1232 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_uart.h"






 










 








 






 







 





 





 






 









 







 








 










 






 







 







 







 









 







 






 







 







 







 







 







 







 










 
# 1455 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_uart.h"





 







 







 







 







 







 








 







 







 







 







 








 







 
# 1571 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_uart.h"



 

 
# 1 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_uart_ex.h"

















 

 







 
# 30 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_uart_ex.h"



 



 

 


 



 
typedef struct
{
  uint32_t WakeUpEvent;        


 

  uint16_t AddressLength;      
 

  uint8_t Address;              
} UART_WakeUpTypeDef;



 

 


 



 





 



 




 




 




 




 
# 108 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_uart_ex.h"


 




 
# 122 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_uart_ex.h"


 



 

 
 


 



 

 
HAL_StatusTypeDef HAL_RS485Ex_Init(UART_HandleTypeDef *huart, uint32_t Polarity, uint32_t AssertionTime,
                                   uint32_t DeassertionTime);



 



 

void HAL_UARTEx_WakeupCallback(UART_HandleTypeDef *huart);

void HAL_UARTEx_RxFifoFullCallback(UART_HandleTypeDef *huart);
void HAL_UARTEx_TxFifoEmptyCallback(UART_HandleTypeDef *huart);



 



 

 
HAL_StatusTypeDef HAL_UARTEx_StopModeWakeUpSourceConfig(UART_HandleTypeDef *huart, UART_WakeUpTypeDef WakeUpSelection);
HAL_StatusTypeDef HAL_UARTEx_EnableStopMode(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UARTEx_DisableStopMode(UART_HandleTypeDef *huart);

HAL_StatusTypeDef HAL_MultiProcessorEx_AddressLength_Set(UART_HandleTypeDef *huart, uint32_t AddressLength);

HAL_StatusTypeDef HAL_UARTEx_EnableFifoMode(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UARTEx_DisableFifoMode(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UARTEx_SetTxFifoThreshold(UART_HandleTypeDef *huart, uint32_t Threshold);
HAL_StatusTypeDef HAL_UARTEx_SetRxFifoThreshold(UART_HandleTypeDef *huart, uint32_t Threshold);

HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint16_t *RxLen,
                                           uint32_t Timeout);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);




 



 

 


 





 
# 541 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_uart_ex.h"









 
# 591 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_uart_ex.h"





 








 







 
# 620 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_uart_ex.h"





 
# 632 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_uart_ex.h"



 

 



 



 







 
# 1578 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_uart.h"

 


 



 

 
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_HalfDuplex_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_LIN_Init(UART_HandleTypeDef *huart, uint32_t BreakDetectLength);
HAL_StatusTypeDef HAL_MultiProcessor_Init(UART_HandleTypeDef *huart, uint8_t Address, uint32_t WakeUpMethod);
HAL_StatusTypeDef HAL_UART_DeInit(UART_HandleTypeDef *huart);
void HAL_UART_MspInit(UART_HandleTypeDef *huart);
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart);

 
# 1606 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_uart.h"



 



 

 
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_DMAPause(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_DMAResume(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *huart);
 
HAL_StatusTypeDef HAL_UART_Abort(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_Abort_IT(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_AbortTransmit_IT(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_AbortReceive_IT(UART_HandleTypeDef *huart);

void HAL_UART_IRQHandler(UART_HandleTypeDef *huart);
void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
void HAL_UART_AbortCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_AbortTransmitCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef *huart);

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);



 



 

 
void HAL_UART_ReceiverTimeout_Config(UART_HandleTypeDef *huart, uint32_t TimeoutValue);
HAL_StatusTypeDef HAL_UART_EnableReceiverTimeout(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_DisableReceiverTimeout(UART_HandleTypeDef *huart);

HAL_StatusTypeDef HAL_LIN_SendBreak(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_MultiProcessor_EnableMuteMode(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_MultiProcessor_DisableMuteMode(UART_HandleTypeDef *huart);
void HAL_MultiProcessor_EnterMuteMode(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_HalfDuplex_EnableTransmitter(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_HalfDuplex_EnableReceiver(UART_HandleTypeDef *huart);



 



 

 
HAL_UART_StateTypeDef HAL_UART_GetState(UART_HandleTypeDef *huart);
uint32_t              HAL_UART_GetError(UART_HandleTypeDef *huart);



 



 

 


 



HAL_StatusTypeDef UART_SetConfig(UART_HandleTypeDef *huart);
HAL_StatusTypeDef UART_CheckIdleState(UART_HandleTypeDef *huart);
HAL_StatusTypeDef UART_WaitOnFlagUntilTimeout(UART_HandleTypeDef *huart, uint32_t Flag, FlagStatus Status,
                                              uint32_t Tickstart, uint32_t Timeout);
void              UART_AdvFeatureConfig(UART_HandleTypeDef *huart);
HAL_StatusTypeDef UART_Start_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef UART_Start_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);



 



 



 







 
# 348 "..\\..\\User\\stm32g4xx_hal_conf.h"


# 1 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_usart.h"

















 

 







 
# 30 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_usart.h"



 



 

 


 



 
typedef struct
{
  uint32_t BaudRate;                  









 

  uint32_t WordLength;                
 

  uint32_t StopBits;                  
 

  uint32_t Parity;                   




 

  uint32_t Mode;                      
 

  uint32_t CLKPolarity;               
 

  uint32_t CLKPhase;                  
 

  uint32_t CLKLastBit;                

 

  uint32_t ClockPrescaler;            
 
} USART_InitTypeDef;



 
typedef enum
{
  HAL_USART_STATE_RESET             = 0x00U,     
  HAL_USART_STATE_READY             = 0x01U,     
  HAL_USART_STATE_BUSY              = 0x02U,     
  HAL_USART_STATE_BUSY_TX           = 0x12U,     
  HAL_USART_STATE_BUSY_RX           = 0x22U,     
  HAL_USART_STATE_BUSY_TX_RX        = 0x32U,     
  HAL_USART_STATE_TIMEOUT           = 0x03U,     
  HAL_USART_STATE_ERROR             = 0x04U      
} HAL_USART_StateTypeDef;



 
typedef enum
{
  USART_CLOCKSOURCE_PCLK1      = 0x00U,     
  USART_CLOCKSOURCE_PCLK2      = 0x01U,     
  USART_CLOCKSOURCE_HSI        = 0x02U,     
  USART_CLOCKSOURCE_SYSCLK     = 0x04U,     
  USART_CLOCKSOURCE_LSE        = 0x08U,     
  USART_CLOCKSOURCE_UNDEFINED  = 0x10U      
} USART_ClockSourceTypeDef;



 
typedef struct __USART_HandleTypeDef
{
  USART_TypeDef                 *Instance;                

  USART_InitTypeDef             Init;                     

  uint8_t                       *pTxBuffPtr;              

  uint16_t                      TxXferSize;               

  volatile uint16_t                 TxXferCount;              

  uint8_t                       *pRxBuffPtr;              

  uint16_t                      RxXferSize;               

  volatile uint16_t                 RxXferCount;              

  uint16_t                      Mask;                     

  uint16_t                      NbRxDataToProcess;        

  uint16_t                      NbTxDataToProcess;        

  uint32_t                      SlaveMode;               
 

  uint32_t                      FifoMode;                
 

  void (*RxISR)(struct __USART_HandleTypeDef *husart);    

  void (*TxISR)(struct __USART_HandleTypeDef *husart);    

  DMA_HandleTypeDef             *hdmatx;                  

  DMA_HandleTypeDef             *hdmarx;                  

  HAL_LockTypeDef               Lock;                     

  volatile HAL_USART_StateTypeDef   State;                    

  volatile uint32_t                 ErrorCode;                

# 180 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_usart.h"

} USART_HandleTypeDef;

# 210 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_usart.h"



 

 


 



 
# 234 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_usart.h"


 



 






 



 





 



 





 



 




 



 




 



 




 



 




 



 
# 320 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_usart.h"



 



 




 





 
# 358 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_usart.h"


 










 

# 388 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_usart.h"



 



 
# 405 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_usart.h"


 



 







 



 

 


 




 
# 443 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_usart.h"

























 
















 





 





 





 





 





 





 





 



















 
# 554 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_usart.h"


















 
# 579 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_usart.h"




















 























 
# 631 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_usart.h"















 










 





 





 





 





 




 

 


 




 
# 710 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_usart.h"






 








 
# 796 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_usart.h"





 






 









 








 






 







 






 






 







 







 
# 885 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_usart.h"



 

 
# 1 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_usart_ex.h"

















 

 







 
# 30 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_usart_ex.h"



 



 

 
 


 



 





 



 




 





 




 




 




 




 
# 95 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_usart_ex.h"


 




 
# 109 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_usart_ex.h"


 



 

 


 









 
# 171 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_usart_ex.h"





 








 







 







 







 
# 216 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_usart_ex.h"





 
# 228 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_usart_ex.h"


 

 


 



 

 
void HAL_USARTEx_RxFifoFullCallback(USART_HandleTypeDef *husart);
void HAL_USARTEx_TxFifoEmptyCallback(USART_HandleTypeDef *husart);



 



 

 
HAL_StatusTypeDef HAL_USARTEx_EnableSlaveMode(USART_HandleTypeDef *husart);
HAL_StatusTypeDef HAL_USARTEx_DisableSlaveMode(USART_HandleTypeDef *husart);
HAL_StatusTypeDef HAL_USARTEx_ConfigNSS(USART_HandleTypeDef *husart, uint32_t NSSConfig);
HAL_StatusTypeDef HAL_USARTEx_EnableFifoMode(USART_HandleTypeDef *husart);
HAL_StatusTypeDef HAL_USARTEx_DisableFifoMode(USART_HandleTypeDef *husart);
HAL_StatusTypeDef HAL_USARTEx_SetTxFifoThreshold(USART_HandleTypeDef *husart, uint32_t Threshold);
HAL_StatusTypeDef HAL_USARTEx_SetRxFifoThreshold(USART_HandleTypeDef *husart, uint32_t Threshold);



 



 



 



 







 
# 892 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_usart.h"

 


 



 

 
HAL_StatusTypeDef HAL_USART_Init(USART_HandleTypeDef *husart);
HAL_StatusTypeDef HAL_USART_DeInit(USART_HandleTypeDef *husart);
void HAL_USART_MspInit(USART_HandleTypeDef *husart);
void HAL_USART_MspDeInit(USART_HandleTypeDef *husart);

 








 



 

 
HAL_StatusTypeDef HAL_USART_Transmit(USART_HandleTypeDef *husart, uint8_t *pTxData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_USART_Receive(USART_HandleTypeDef *husart, uint8_t *pRxData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_USART_TransmitReceive(USART_HandleTypeDef *husart, uint8_t *pTxData, uint8_t *pRxData,
                                            uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_USART_Transmit_IT(USART_HandleTypeDef *husart, uint8_t *pTxData, uint16_t Size);
HAL_StatusTypeDef HAL_USART_Receive_IT(USART_HandleTypeDef *husart, uint8_t *pRxData, uint16_t Size);
HAL_StatusTypeDef HAL_USART_TransmitReceive_IT(USART_HandleTypeDef *husart, uint8_t *pTxData, uint8_t *pRxData,
                                               uint16_t Size);
HAL_StatusTypeDef HAL_USART_Transmit_DMA(USART_HandleTypeDef *husart, uint8_t *pTxData, uint16_t Size);
HAL_StatusTypeDef HAL_USART_Receive_DMA(USART_HandleTypeDef *husart, uint8_t *pRxData, uint16_t Size);
HAL_StatusTypeDef HAL_USART_TransmitReceive_DMA(USART_HandleTypeDef *husart, uint8_t *pTxData, uint8_t *pRxData,
                                                uint16_t Size);
HAL_StatusTypeDef HAL_USART_DMAPause(USART_HandleTypeDef *husart);
HAL_StatusTypeDef HAL_USART_DMAResume(USART_HandleTypeDef *husart);
HAL_StatusTypeDef HAL_USART_DMAStop(USART_HandleTypeDef *husart);
 
HAL_StatusTypeDef HAL_USART_Abort(USART_HandleTypeDef *husart);
HAL_StatusTypeDef HAL_USART_Abort_IT(USART_HandleTypeDef *husart);

void HAL_USART_IRQHandler(USART_HandleTypeDef *husart);
void HAL_USART_TxHalfCpltCallback(USART_HandleTypeDef *husart);
void HAL_USART_TxCpltCallback(USART_HandleTypeDef *husart);
void HAL_USART_RxCpltCallback(USART_HandleTypeDef *husart);
void HAL_USART_RxHalfCpltCallback(USART_HandleTypeDef *husart);
void HAL_USART_TxRxCpltCallback(USART_HandleTypeDef *husart);
void HAL_USART_ErrorCallback(USART_HandleTypeDef *husart);
void HAL_USART_AbortCpltCallback(USART_HandleTypeDef *husart);



 



 

 
HAL_USART_StateTypeDef HAL_USART_GetState(USART_HandleTypeDef *husart);
uint32_t               HAL_USART_GetError(USART_HandleTypeDef *husart);



 



 



 



 







 
# 352 "..\\..\\User\\stm32g4xx_hal_conf.h"


# 1 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_wwdg.h"

















 

 







 
# 30 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_wwdg.h"



 



 

 



 



 
typedef struct
{
  uint32_t Prescaler;     
 

  uint32_t Window;        
 

  uint32_t Counter;       
 

  uint32_t EWIMode ;      
 

} WWDG_InitTypeDef;



 



typedef struct

{
  WWDG_TypeDef      *Instance;   

  WWDG_InitTypeDef  Init;        






} WWDG_HandleTypeDef;

# 100 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_wwdg.h"


 

 



 



 



 




 



 



 
# 138 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_wwdg.h"


 



 




 



 

 



 
# 168 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal_wwdg.h"









 


 



 





 










 









 








 









 









 








 





 

 



 



 
 
HAL_StatusTypeDef     HAL_WWDG_Init(WWDG_HandleTypeDef *hwwdg);
void                  HAL_WWDG_MspInit(WWDG_HandleTypeDef *hwwdg);
 








 



 
 
HAL_StatusTypeDef     HAL_WWDG_Refresh(WWDG_HandleTypeDef *hwwdg);
void                  HAL_WWDG_IRQHandler(WWDG_HandleTypeDef *hwwdg);
void                  HAL_WWDG_EarlyWakeupCallback(WWDG_HandleTypeDef *hwwdg);


 



 



 



 







 
# 356 "..\\..\\User\\stm32g4xx_hal_conf.h"


 
# 374 "..\\..\\User\\stm32g4xx_hal_conf.h"







 
# 31 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal.h"



 



 

 
 



 



 







 



 



 















 



 
# 92 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal.h"



 



 
# 132 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal.h"



 




 






 



 





 




 






 



 


 
# 185 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal.h"



 



 



 

 



 


 











































































































 



 


 



 



 
























 






 
 







 






 




 











 





 





 





 








 




 








 










 

 


 

# 458 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal.h"























# 498 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Inc\\stm32g4xx_hal.h"


 



 





 

 



 



 
 
HAL_StatusTypeDef HAL_Init(void);
HAL_StatusTypeDef HAL_DeInit(void);
void HAL_MspInit(void);
void HAL_MspDeInit(void);
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority);



 



 

 
void HAL_IncTick(void);
void HAL_Delay(uint32_t Delay);
uint32_t HAL_GetTick(void);
uint32_t HAL_GetTickPrio(void);
HAL_StatusTypeDef HAL_SetTickFreq(uint32_t Freq);
uint32_t HAL_GetTickFreq(void);
void HAL_SuspendTick(void);
void HAL_ResumeTick(void);
uint32_t HAL_GetHalVersion(void);
uint32_t HAL_GetREVID(void);
uint32_t HAL_GetDEVID(void);



 



 

 
void HAL_DBGMCU_EnableDBGSleepMode(void);
void HAL_DBGMCU_DisableDBGSleepMode(void);
void HAL_DBGMCU_EnableDBGStopMode(void);
void HAL_DBGMCU_DisableDBGStopMode(void);
void HAL_DBGMCU_EnableDBGStandbyMode(void);
void HAL_DBGMCU_DisableDBGStandbyMode(void);



 

 


 
extern volatile uint32_t uwTick;
extern uint32_t uwTickPrio;
extern uint32_t uwTickFreq;


 



 

 
void HAL_SYSCFG_CCMSRAMErase(void);
void HAL_SYSCFG_EnableMemorySwappingBank(void);
void HAL_SYSCFG_DisableMemorySwappingBank(void);


void HAL_SYSCFG_VREFBUF_VoltageScalingConfig(uint32_t VoltageScaling);
void HAL_SYSCFG_VREFBUF_HighImpedanceConfig(uint32_t Mode);
void HAL_SYSCFG_VREFBUF_TrimmingConfig(uint32_t TrimmingValue);
HAL_StatusTypeDef HAL_SYSCFG_EnableVREFBUF(void);
void HAL_SYSCFG_DisableVREFBUF(void);


void HAL_SYSCFG_EnableIOSwitchBooster(void);
void HAL_SYSCFG_DisableIOSwitchBooster(void);
void HAL_SYSCFG_EnableIOSwitchVDD(void);
void HAL_SYSCFG_DisableIOSwitchVDD(void);

void HAL_SYSCFG_CCMSRAM_WriteProtectionEnable(uint32_t Page);



 



 



 



 







 
# 306 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Src\\stm32g4xx_hal_adc.c"



 




 



 
 



 






 
 
 
 



 
 
 
 
 
 
 
 





 

 
 
 
 



 













 




























 
HAL_StatusTypeDef HAL_ADC_Init(ADC_HandleTypeDef *hadc)
{
  HAL_StatusTypeDef tmp_hal_status = HAL_OK;
  uint32_t tmpCFGR;
  uint32_t tmp_adc_reg_is_conversion_on_going;
  volatile uint32_t wait_loop_index = 0UL;
  uint32_t tmp_adc_is_conversion_on_going_regular;
  uint32_t tmp_adc_is_conversion_on_going_injected;

   
  if (hadc == 0)
  {
    return HAL_ERROR;
  }

   
  ((void)0U);
  ((void)0U);
  ((void)0U);
  ((void)0U);
  ((void)0U);
  ((void)0U);
  ((void)0U);
  ((void)0U);
  ((void)0U);
  ((void)0U);
  ((void)0U);
  ((void)0U);
  ((void)0U);
  ((void)0U);
  ((void)0U);

  if (hadc->Init.ScanConvMode != (0x00000000UL))
  {
    ((void)0U);
    ((void)0U);

    if (hadc->Init.DiscontinuousConvMode == ENABLE)
    {
      ((void)0U);
    }
  }

   
  ((void)0U);

   
   
  if (hadc->State == (0x00000000UL))
  {
# 473 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Src\\stm32g4xx_hal_adc.c"
     
    HAL_ADC_MspInit(hadc);


     
    ((hadc)->ErrorCode = (0x00U));

     
    hadc->Lock = HAL_UNLOCKED;
  }

   
  if (LL_ADC_IsDeepPowerDownEnabled(hadc->Instance) != 0UL)
  {
     
    LL_ADC_DisableDeepPowerDown(hadc->Instance);

    

 
  }

  if (LL_ADC_IsInternalRegulatorEnabled(hadc->Instance) == 0UL)
  {
     
    LL_ADC_EnableInternalRegulator(hadc->Instance);

     
     
     
    wait_loop_index = ((( 20UL) / 10UL) * ((SystemCoreClock / (100000UL * 2UL)) + 1UL));
    while (wait_loop_index != 0UL)
    {
      wait_loop_index--;
    }
  }

   
   
   
  if (LL_ADC_IsInternalRegulatorEnabled(hadc->Instance) == 0UL)
  {
     
    ((hadc->State) |= ((0x00000010UL)));

     
    ((hadc->ErrorCode) |= ((0x01U)));

    tmp_hal_status = HAL_ERROR;
  }

   
   
   
   
  tmp_adc_reg_is_conversion_on_going = LL_ADC_REG_IsConversionOngoing(hadc->Instance);

  if (((hadc->State & (0x00000010UL)) == 0UL)
      && (tmp_adc_reg_is_conversion_on_going == 0UL)
     )
  {
     
    (((hadc->State)) = ((((((hadc->State))) & (~((0x00000100UL)))) | ((0x00000002UL)))));



     

     
     
     
    if (LL_ADC_IsEnabled(hadc->Instance) == 0UL)
    {
      if ((LL_ADC_IsEnabled(((ADC_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x08000000UL))) | LL_ADC_IsEnabled(((ADC_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x08000100UL)))) == 0UL)
      {
         
         
         
         
         
         
         
         
         
         
         
         
         
         
         
         
         
        LL_ADC_SetCommonClock((((ADC_Common_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x08000300UL))), hadc->Init.ClockPrescaler);
      }
    }

     
     
     
     
     
     
     
     
     
    tmpCFGR  = ((((uint32_t)hadc->Init . ContinuousConvMode) << (13U))           |
                hadc->Init.Overrun                                                     |
                hadc->Init.DataAlign                                                   |
                hadc->Init.Resolution                                                  |
                (((uint32_t)hadc->Init . DiscontinuousConvMode) << (16U)));

    if (hadc->Init.DiscontinuousConvMode == ENABLE)
    {
      tmpCFGR |= (((hadc->Init . NbrOfDiscConversion) - 1UL) << (17U));
    }

     
     
     
     
     
    if (hadc->Init.ExternalTrigConv != ((0x00000000UL)))
    {
      tmpCFGR |= ((hadc->Init.ExternalTrigConv & (0x1FUL << (5U)))
                  | hadc->Init.ExternalTrigConvEdge
                 );
    }

     
    (((hadc->Instance ->CFGR)) = ((((((hadc->Instance ->CFGR))) & (~((((0x3UL << (3U)) | (0x1UL << (15U)) | (0x1UL << (13U)) | (0x1UL << (12U)) | (0x1UL << (16U)) | (0x7UL << (17U)) | (0x3UL << (10U)) | (0x1FUL << (5U))))))) | (tmpCFGR))));

     
    (((hadc->Instance ->CFGR2)) = ((((((hadc->Instance ->CFGR2))) & (~((0x1UL << (26U)) | (0x1UL << (27U))))) | (hadc->Init . SamplingMode))));

     
     
     
     
     
     
     
    tmp_adc_is_conversion_on_going_regular = LL_ADC_REG_IsConversionOngoing(hadc->Instance);
    tmp_adc_is_conversion_on_going_injected = LL_ADC_INJ_IsConversionOngoing(hadc->Instance);
    if ((tmp_adc_is_conversion_on_going_regular == 0UL)
        && (tmp_adc_is_conversion_on_going_injected == 0UL)
       )
    {
      tmpCFGR = ((0x0UL)                                            |
                 (((uint32_t)hadc->Init . LowPowerAutoWait) << (14U))        |
                 (((uint32_t)hadc->Init . DMAContinuousRequests) << (1U)));

      (((hadc->Instance ->CFGR)) = ((((((hadc->Instance ->CFGR))) & (~((((0x1UL << (1U)) | (0x1UL << (14U))))))) | (tmpCFGR))));

      if (hadc->Init.GainCompensation != 0UL)
      {
        ((hadc->Instance ->CFGR2) |= ((0x1UL << (16U))));
        (((hadc->Instance ->GCOMP)) = ((((((hadc->Instance ->GCOMP))) & (~((0x3FFFUL << (0U))))) | (hadc->Init . GainCompensation))));
      }
      else
      {
        ((hadc->Instance ->CFGR2) &= ~((0x1UL << (16U))));
        (((hadc->Instance ->GCOMP)) = ((((((hadc->Instance ->GCOMP))) & (~((0x3FFFUL << (0U))))) | (0UL))));
      }

      if (hadc->Init.OversamplingMode == ENABLE)
      {
        ((void)0U);
        ((void)0U);
        ((void)0U);
        ((void)0U);

         
         
         
         
         
        (((hadc->Instance ->CFGR2)) = ((((((hadc->Instance ->CFGR2))) & (~((0x7UL << (2U)) | (0xFUL << (5U)) | (0x1UL << (9U)) | (0x1UL << (10U))))) | ((0x1UL << (0U)) | hadc->Init . Oversampling . Ratio | hadc->Init . Oversampling . RightBitShift | hadc->Init . Oversampling . TriggeredMode | hadc->Init . Oversampling . OversamplingStopReset))));
# 660 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Src\\stm32g4xx_hal_adc.c"
      }
      else
      {
         
        ((hadc->Instance ->CFGR2) &= ~((0x1UL << (0U))));
      }

    }

     
     
     
     
     
     
     
     

    if (hadc->Init.ScanConvMode == (0x00000001UL))
    {
       
      (((hadc->Instance ->SQR1)) = ((((((hadc->Instance ->SQR1))) & (~((0xFUL << (0U))))) | ((hadc->Init . NbrOfConversion - (uint8_t)1)))));
    }
    else
    {
      ((hadc->Instance ->SQR1) &= ~((0xFUL << (0U))));
    }

     
     
    (((hadc->State)) = ((((((hadc->State))) & (~((0x00000002UL)))) | ((0x00000001UL)))));
  }
  else
  {
     
    ((hadc->State) |= ((0x00000010UL)));

    tmp_hal_status = HAL_ERROR;
  }

   
  return tmp_hal_status;
}


















 
HAL_StatusTypeDef HAL_ADC_DeInit(ADC_HandleTypeDef *hadc)
{
  HAL_StatusTypeDef tmp_hal_status;

   
  if (hadc == 0)
  {
    return HAL_ERROR;
  }

   
  ((void)0U);

   
  ((hadc->State) |= ((0x00000002UL)));

   
  tmp_hal_status = ADC_ConversionStop(hadc, ((0x00000003UL)));

   
   
   
   
   
   
  ((hadc->Instance ->CFGR) |= ((0x1UL << (21U))));

   
  if (tmp_hal_status == HAL_OK)
  {
     
    tmp_hal_status = ADC_Disable(hadc);

     
    if (tmp_hal_status == HAL_OK)
    {
       
      hadc->State = (0x00000001UL);
    }
  }

   
   
   
   
   

   
   
  (((hadc)->Instance ->IER) &= ~(((0x1UL << (9U)) | (0x1UL << (8U)) | (0x1UL << (7U)) | (0x1UL << (10U)) | (0x1UL << (4U)) | (0x1UL << (6U)) | (0x1UL << (5U)) | (0x1UL << (3U)) | (0x1UL << (2U)) | (0x1UL << (1U)) | (0x1UL << (0U)))));





   
  (((hadc)->Instance ->ISR) = (((0x1UL << (9U)) | (0x1UL << (8U)) | (0x1UL << (7U)) | (0x1UL << (10U)) | (0x1UL << (4U)) | (0x1UL << (6U)) | (0x1UL << (5U)) | (0x1UL << (3U)) | (0x1UL << (2U)) | (0x1UL << (1U)) | (0x1UL << (0U)))));





   
  


 
  ((hadc->Instance ->CR) &= ~((0x1UL << (28U)) | (0x1UL << (30U))));
  ((hadc->Instance ->CR) |= ((0x1UL << (29U))));

   
  ((hadc->Instance ->CFGR) &= ~(((0x1FUL << (26U)) | (0x1UL << (25U)) | (0x1UL << (24U)) | (0x1UL << (23U)) | (0x1UL << (22U)) | (0x1UL << (21U)) | (0x1UL << (20U)) | (0x7UL << (17U)) | (0x1UL << (16U)) | (0x1UL << (14U)) | (0x1UL << (13U)) | (0x1UL << (12U)) | (0x3UL << (10U)) | (0x1FUL << (5U)) | (0x1UL << (15U)) | (0x3UL << (3U)) | (0x1UL << (1U)) | (0x1UL << (0U)) )));
  ((hadc->Instance ->CFGR) |= ((0x1UL << (31U))));

   
  ((hadc->Instance ->CFGR2) &= ~((0x1UL << (10U)) | (0x1UL << (9U)) | (0xFUL << (5U)) | (0x7UL << (2U)) | (0x1UL << (1U)) | (0x1UL << (0U))));


   
  ((hadc->Instance ->SMPR1) &= ~(((0x7UL << (27U)) | (0x7UL << (24U)) | (0x7UL << (21U)) | (0x7UL << (18U)) | (0x7UL << (15U)) | (0x7UL << (12U)) | (0x7UL << (9U)) | (0x7UL << (6U)) | (0x7UL << (3U)) | (0x7UL << (0U)) | (0x1UL << (31U)))));

   
  ((hadc->Instance ->SMPR2) &= ~((0x7UL << (24U)) | (0x7UL << (21U)) | (0x7UL << (18U)) | (0x7UL << (15U)) | (0x7UL << (12U)) | (0x7UL << (9U)) | (0x7UL << (6U)) | (0x7UL << (3U)) | (0x7UL << (0U))));



   
  ((hadc->Instance ->TR1) &= ~((0xFFFUL << (16U)) | (0xFFFUL << (0U))));

   
  ((hadc->Instance ->TR2) &= ~((0xFFUL << (16U)) | (0xFFUL << (0U))));

   
  ((hadc->Instance ->TR3) &= ~((0xFFUL << (16U)) | (0xFFUL << (0U))));

   
  ((hadc->Instance ->SQR1) &= ~((0x1FUL << (24U)) | (0x1FUL << (18U)) | (0x1FUL << (12U)) | (0x1FUL << (6U)) | (0xFUL << (0U))));


   
  ((hadc->Instance ->SQR2) &= ~((0x1FUL << (24U)) | (0x1FUL << (18U)) | (0x1FUL << (12U)) | (0x1FUL << (6U)) | (0x1FUL << (0U))));


   
  ((hadc->Instance ->SQR3) &= ~((0x1FUL << (24U)) | (0x1FUL << (18U)) | (0x1FUL << (12U)) | (0x1FUL << (6U)) | (0x1FUL << (0U))));


   
  ((hadc->Instance ->SQR4) &= ~((0x1FUL << (6U)) | (0x1FUL << (0U))));

   

   
   

   
  ((hadc->Instance ->OFR1) &= ~((0x1UL << (31U)) | (0x1FUL << (26U)) | (0xFFFUL << (0U))));
   
  ((hadc->Instance ->OFR2) &= ~((0x1UL << (31U)) | (0x1FUL << (26U)) | (0xFFFUL << (0U))));
   
  ((hadc->Instance ->OFR3) &= ~((0x1UL << (31U)) | (0x1FUL << (26U)) | (0xFFFUL << (0U))));
   
  ((hadc->Instance ->OFR4) &= ~((0x1UL << (31U)) | (0x1FUL << (26U)) | (0xFFFUL << (0U))));

   
   

   
  ((hadc->Instance ->AWD2CR) &= ~((0x7FFFFUL << (0U))));

   
  ((hadc->Instance ->AWD3CR) &= ~((0x7FFFFUL << (0U))));

   
  ((hadc->Instance ->DIFSEL) &= ~((0x7FFFFUL << (0U))));

   
  ((hadc->Instance ->CALFACT) &= ~((0x7FUL << (16U)) | (0x7FUL << (0U))));


   

  
 
  if ((LL_ADC_IsEnabled(((ADC_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x08000000UL))) | LL_ADC_IsEnabled(((ADC_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x08000100UL)))) == 0UL)
  {
    





 
    (((((ADC_Common_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x08000300UL)))->CCR) &= ~((0x3UL << (16U)) | (0xFUL << (18U)) | (0x1UL << (24U)) | (0x1UL << (23U)) | (0x1UL << (22U)) | (0x3UL << (14U)) | (0x1UL << (13U)) | (0xFUL << (8U)) | (0x1FUL << (0U))));

     
     
     
     
     
     
     
     
# 894 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Src\\stm32g4xx_hal_adc.c"
     
    HAL_ADC_MspDeInit(hadc);

  }

   
  ((hadc)->ErrorCode = (0x00U));

   
  hadc->InjectionConfig.ContextQueue = 0;
  hadc->InjectionConfig.ChannelCount = 0;

   
  hadc->State = (0x00000000UL);

   
  do{ (hadc)->Lock = HAL_UNLOCKED; }while (0U);

   
  return tmp_hal_status;
}





 
__weak void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
   
  (void)hadc;

  

 
}







 
__weak void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc)
{
   
  (void)hadc;

  

 
}

# 1192 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Src\\stm32g4xx_hal_adc.c"



 





















 









 
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef *hadc)
{
  HAL_StatusTypeDef tmp_hal_status;

  const ADC_TypeDef *tmpADC_Master;
  uint32_t tmp_multimode_config = LL_ADC_GetMultimode((((ADC_Common_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x08000300UL))));


   
  ((void)0U);

   
  if (LL_ADC_REG_IsConversionOngoing(hadc->Instance) == 0UL)
  {
     
    do{ if((hadc)->Lock == HAL_LOCKED) { return HAL_BUSY; } else { (hadc)->Lock = HAL_LOCKED; } }while (0U);

     
    tmp_hal_status = ADC_Enable(hadc);

     
    if (tmp_hal_status == HAL_OK)
    {
       
       
       
      (((hadc->State)) = ((((((hadc->State))) & (~((0x00000001UL) | (0x00000200UL) | (0x00000400UL) | (0x00000800UL)))) | ((0x00000100UL)))));




      

 
      if ((( ( ((hadc->Instance) == ((ADC_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x08000100UL))) )? (((ADC_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x08000000UL))) : (hadc->Instance) ) == hadc->Instance)
          || (tmp_multimode_config == (0x00000000UL))
         )
      {
        ((hadc->State) &= ~((0x00100000UL)));
      }


       
       
      if ((((hadc->State) & ((0x00001000UL))) == ((0x00001000UL))))
      {
         
        ((hadc->ErrorCode) &= ~(((0x02U) | (0x04U))));
      }
      else
      {
         
        ((hadc)->ErrorCode = (0x00U));
      }

       
       
      (((hadc)->Instance ->ISR) = (((0x1UL << (2U)) | (0x1UL << (3U)) | (0x1UL << (4U)))));

       
       
       
      do{ (hadc)->Lock = HAL_UNLOCKED; }while (0U);

       
       
       
       
       
       
       
       

      if ((( ( ((hadc->Instance) == ((ADC_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x08000100UL))) )? (((ADC_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x08000000UL))) : (hadc->Instance) ) == hadc->Instance)
          || (tmp_multimode_config == (0x00000000UL))
          || (tmp_multimode_config == ( (0x04UL << (0U)) | (0x01UL << (0U))))
          || (tmp_multimode_config == ((0x08UL << (0U)) | (0x01UL << (0U))))
         )
      {
         
        if (((hadc->Instance ->CFGR) & ((0x1UL << (25U)))) != 0UL)
        {
          (((hadc->State)) = ((((((hadc->State))) & (~((0x00002000UL)))) | ((0x00001000UL)))));
        }

         
        LL_ADC_REG_StartConversion(hadc->Instance);
      }
      else
      {
         
        ((hadc->State) |= ((0x00100000UL)));
        
 
        tmpADC_Master = ( ( ((hadc->Instance) == ((ADC_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x08000100UL))) )? (((ADC_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x08000000UL))) : (hadc->Instance) );
        if (((tmpADC_Master->CFGR) & ((0x1UL << (25U)))) != 0UL)
        {
          (((hadc->State)) = ((((((hadc->State))) & (~((0x00002000UL)))) | ((0x00001000UL)))));
        }

      }
# 1338 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Src\\stm32g4xx_hal_adc.c"
    }
    else
    {
       
      do{ (hadc)->Lock = HAL_UNLOCKED; }while (0U);
    }
  }
  else
  {
    tmp_hal_status = HAL_BUSY;
  }

   
  return tmp_hal_status;
}









 
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef *hadc)
{
  HAL_StatusTypeDef tmp_hal_status;

   
  ((void)0U);

   
  do{ if((hadc)->Lock == HAL_LOCKED) { return HAL_BUSY; } else { (hadc)->Lock = HAL_LOCKED; } }while (0U);

   
  tmp_hal_status = ADC_ConversionStop(hadc, ((0x00000003UL)));

   
  if (tmp_hal_status == HAL_OK)
  {
     
    tmp_hal_status = ADC_Disable(hadc);

     
    if (tmp_hal_status == HAL_OK)
    {
       
      (((hadc->State)) = ((((((hadc->State))) & (~((0x00000100UL) | (0x00001000UL)))) | ((0x00000001UL)))));


    }
  }

   
  do{ (hadc)->Lock = HAL_UNLOCKED; }while (0U);

   
  return tmp_hal_status;
}


















 
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef *hadc, uint32_t Timeout)
{
  uint32_t tickstart;
  uint32_t tmp_Flag_End;
  uint32_t tmp_cfgr;

  const ADC_TypeDef *tmpADC_Master;
  uint32_t tmp_multimode_config = LL_ADC_GetMultimode((((ADC_Common_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x08000300UL))));


   
  ((void)0U);

   
  if (hadc->Init.EOCSelection == ((0x1UL << (3U))))
  {
    tmp_Flag_End = (0x1UL << (3U));
  }
   
  else  
  {
     
     
     
     
     
     

    if ((tmp_multimode_config == (0x00000000UL))
        || (tmp_multimode_config == ( (0x04UL << (0U)) | (0x01UL << (0U))))
        || (tmp_multimode_config == ((0x08UL << (0U)) | (0x01UL << (0U))))
       )
    {
       
      if (((hadc->Instance ->CFGR) & ((0x1UL << (0U)))) != 0UL)
      {
        ((hadc->State) |= ((0x00000020UL)));
        return HAL_ERROR;
      }
      else
      {
        tmp_Flag_End = ((0x1UL << (2U)));
      }
    }
    else
    {
       
      if (LL_ADC_GetMultiDMATransfer((((ADC_Common_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x08000300UL)))) != (0x00000000UL))
      {
        ((hadc->State) |= ((0x00000020UL)));
        return HAL_ERROR;
      }
      else
      {
        tmp_Flag_End = ((0x1UL << (2U)));
      }
    }
# 1486 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Src\\stm32g4xx_hal_adc.c"
  }

   
  tickstart = HAL_GetTick();

   
  while ((hadc->Instance->ISR & tmp_Flag_End) == 0UL)
  {
     
    if (Timeout != 0xFFFFFFFFU)
    {
      if (((HAL_GetTick() - tickstart) > Timeout) || (Timeout == 0UL))
      {
         
        if ((hadc->Instance->ISR & tmp_Flag_End) == 0UL)
        {
           
          ((hadc->State) |= ((0x00000004UL)));

           
          do{ (hadc)->Lock = HAL_UNLOCKED; }while (0U);

          return HAL_TIMEOUT;
        }
      }
    }
  }

   
  ((hadc->State) |= ((0x00000200UL)));

   
   
  if ((LL_ADC_REG_IsTriggerSourceSWStart(hadc->Instance) != 0UL)
      && (hadc->Init.ContinuousConvMode == DISABLE)
     )
  {
     
    if (((((hadc)->Instance ->ISR) & ((0x1UL << (3U)))) == ((0x1UL << (3U)))))
    {
       
      ((hadc->State) &= ~((0x00000100UL)));

      if ((hadc->State & (0x00001000UL)) == 0UL)
      {
        ((hadc->State) |= ((0x00000001UL)));
      }
    }
  }

   
   
   

  if ((( ( ((hadc->Instance) == ((ADC_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x08000100UL))) )? (((ADC_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x08000000UL))) : (hadc->Instance) ) == hadc->Instance)
      || (tmp_multimode_config == (0x00000000UL))
      || (tmp_multimode_config == ( (0x04UL << (0U)) | (0x01UL << (0U))))
      || (tmp_multimode_config == ((0x08UL << (0U)) | (0x01UL << (0U))))
     )
  {
     
    tmp_cfgr = ((hadc->Instance ->CFGR));
  }
  else
  {
     
    tmpADC_Master = ( ( ((hadc->Instance) == ((ADC_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x08000100UL))) )? (((ADC_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x08000000UL))) : (hadc->Instance) );
    tmp_cfgr = ((tmpADC_Master->CFGR));
  }





   
  if (tmp_Flag_End == (0x1UL << (3U)))
  {
    (((hadc)->Instance ->ISR) = ((0x1UL << (3U))));
  }
  else
  {
     
     
     
    if (((tmp_cfgr) & ((0x1UL << (14U)))) == 0UL)
    {
      (((hadc)->Instance ->ISR) = (((0x1UL << (2U)) | (0x1UL << (3U)))));
    }
  }

   
  return HAL_OK;
}




















 
HAL_StatusTypeDef HAL_ADC_PollForEvent(ADC_HandleTypeDef *hadc, uint32_t EventType, uint32_t Timeout)
{
  uint32_t tickstart;

   
  ((void)0U);
  ((void)0U);

   
  tickstart = HAL_GetTick();

   
  while (((((hadc)->Instance ->ISR) & (EventType)) == (EventType)) == 0UL)
  {
     
    if (Timeout != 0xFFFFFFFFU)
    {
      if (((HAL_GetTick() - tickstart) > Timeout) || (Timeout == 0UL))
      {
         
        if (((((hadc)->Instance ->ISR) & (EventType)) == (EventType)) == 0UL)
        {
           
          ((hadc->State) |= ((0x00000004UL)));

           
          do{ (hadc)->Lock = HAL_UNLOCKED; }while (0U);

          return HAL_TIMEOUT;
        }
      }
    }
  }

  switch (EventType)
  {
     
    case ((0x1UL << (1U))):
       
      ((hadc->State) |= ((0x00000800UL)));

       
      (((hadc)->Instance ->ISR) = ((0x1UL << (1U))));

      break;

     
     
     
     
     
     
     
     

     
    case ((0x1UL << (7U))):
       
      ((hadc->State) |= ((0x00010000UL)));

       
      (((hadc)->Instance ->ISR) = ((0x1UL << (7U))));

      break;

     
    case ((0x1UL << (8U))):
       
      ((hadc->State) |= ((0x00020000UL)));

       
      (((hadc)->Instance ->ISR) = ((0x1UL << (8U))));

      break;

     
    case ((0x1UL << (9U))):
       
      ((hadc->State) |= ((0x00040000UL)));

       
      (((hadc)->Instance ->ISR) = ((0x1UL << (9U))));

      break;

     
    case ((0x1UL << (10U))):
       
      ((hadc->State) |= ((0x00004000UL)));

       
      ((hadc->ErrorCode) |= ((0x08U)));

       
      (((hadc)->Instance ->ISR) = ((0x1UL << (10U))));

      break;

     
    default:  
       
       
       
       
      if (hadc->Init.Overrun == ((0x00000000UL)))
      {
         
        ((hadc->State) |= ((0x00000400UL)));

         
        ((hadc->ErrorCode) |= ((0x02U)));
      }
      else
      {
        

 
        (((hadc)->Instance ->ISR) = ((0x1UL << (4U))));
      }
      break;
  }

   
  return HAL_OK;
}























 
HAL_StatusTypeDef HAL_ADC_Start_IT(ADC_HandleTypeDef *hadc)
{
  HAL_StatusTypeDef tmp_hal_status;

  const ADC_TypeDef *tmpADC_Master;
  uint32_t tmp_multimode_config = LL_ADC_GetMultimode((((ADC_Common_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x08000300UL))));


   
  ((void)0U);

   
  if (LL_ADC_REG_IsConversionOngoing(hadc->Instance) == 0UL)
  {
     
    do{ if((hadc)->Lock == HAL_LOCKED) { return HAL_BUSY; } else { (hadc)->Lock = HAL_LOCKED; } }while (0U);

     
    tmp_hal_status = ADC_Enable(hadc);

     
    if (tmp_hal_status == HAL_OK)
    {
       
       
       
      (((hadc->State)) = ((((((hadc->State))) & (~((0x00000001UL) | (0x00000200UL) | (0x00000400UL) | (0x00000800UL)))) | ((0x00000100UL)))));




      

 
      if ((( ( ((hadc->Instance) == ((ADC_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x08000100UL))) )? (((ADC_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x08000000UL))) : (hadc->Instance) ) == hadc->Instance)
          || (tmp_multimode_config == (0x00000000UL))
         )
      {
        ((hadc->State) &= ~((0x00100000UL)));
      }


       
       
      if ((hadc->State & (0x00001000UL)) != 0UL)
      {
         
        ((hadc->ErrorCode) &= ~(((0x02U) | (0x04U))));
      }
      else
      {
         
        ((hadc)->ErrorCode = (0x00U));
      }

       
       
      (((hadc)->Instance ->ISR) = (((0x1UL << (2U)) | (0x1UL << (3U)) | (0x1UL << (4U)))));

       
       
       
      do{ (hadc)->Lock = HAL_UNLOCKED; }while (0U);

       
      (((hadc)->Instance ->IER) &= ~(((0x1UL << (2U)) | (0x1UL << (3U)) | (0x1UL << (4U)))));

       
      switch (hadc->Init.EOCSelection)
      {
        case ((0x1UL << (3U))):
          (((hadc)->Instance ->IER) |= ((0x1UL << (3U))));
          break;
         
        default:
          (((hadc)->Instance ->IER) |= ((0x1UL << (2U))));
          break;
      }

       
      

 
      if (hadc->Init.Overrun == ((0x00000000UL)))
      {
        (((hadc)->Instance ->IER) |= ((0x1UL << (4U))));
      }

       
       
       
       
       
       
       
       

      if ((( ( ((hadc->Instance) == ((ADC_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x08000100UL))) )? (((ADC_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x08000000UL))) : (hadc->Instance) ) == hadc->Instance)
          || (tmp_multimode_config == (0x00000000UL))
          || (tmp_multimode_config == ( (0x04UL << (0U)) | (0x01UL << (0U))))
          || (tmp_multimode_config == ((0x08UL << (0U)) | (0x01UL << (0U))))
         )
      {
         
        if (((hadc->Instance ->CFGR) & ((0x1UL << (25U)))) != 0UL)
        {
          (((hadc->State)) = ((((((hadc->State))) & (~((0x00002000UL)))) | ((0x00001000UL)))));

          


 
          switch (hadc->Init.EOCSelection)
          {
            case ((0x1UL << (3U))):
              (((hadc)->Instance ->IER) &= ~((0x1UL << (5U))));
              (((hadc)->Instance ->IER) |= ((0x1UL << (6U))));
              break;
             
            default:
              (((hadc)->Instance ->IER) &= ~((0x1UL << (6U))));
              (((hadc)->Instance ->IER) |= ((0x1UL << (5U))));
              break;
          }
        }

         
        LL_ADC_REG_StartConversion(hadc->Instance);
      }
      else
      {
         
        ((hadc->State) |= ((0x00100000UL)));
        
 
        tmpADC_Master = ( ( ((hadc->Instance) == ((ADC_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x08000100UL))) )? (((ADC_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x08000000UL))) : (hadc->Instance) );
        if (((tmpADC_Master->CFGR) & ((0x1UL << (25U)))) != 0UL)
        {
          
 
          (((hadc->State)) = ((((((hadc->State))) & (~((0x00002000UL)))) | ((0x00001000UL)))));
           
          switch (hadc->Init.EOCSelection)
          {
            case ((0x1UL << (3U))):
              (((hadc)->Instance ->IER) &= ~((0x1UL << (5U))));
              (((hadc)->Instance ->IER) |= ((0x1UL << (6U))));
              break;
             
            default:
              (((hadc)->Instance ->IER) &= ~((0x1UL << (6U))));
              (((hadc)->Instance ->IER) |= ((0x1UL << (5U))));
              break;
          }
        }
      }
# 1932 "..\\..\\Drivers\\STM32G4xx_HAL_Driver\\Src\\stm32g4xx_hal_adc.c"
    }
    else
    {
       
      do{ (hadc)->Lock = HAL_UNLOCKED; }while (0U);
    }

  }
  else
  {
    tmp_hal_status = HAL_BUSY;
  }

   
  return tmp_hal_status;
}







 
HAL_StatusTypeDef HAL_ADC_Stop_IT(ADC_HandleTypeDef *hadc)
{
  HAL_StatusTypeDef tmp_hal_status;

   
  ((void)0U);

   
  do{ if((hadc)->Lock == HAL_LOCKED) { return HAL_BUSY; } else { (hadc)->Lock = HAL_LOCKED; } }while (0U);

   
  tmp_hal_status = ADC_ConversionStop(hadc, ((0x00000003UL)));

   
  if (tmp_hal_status == HAL_OK)
  {
     
     
    (((hadc)->Instance ->IER) &= ~(((0x1UL << (2U)) | (0x1UL << (3U)) | (0x1UL << (4U)))));

     
    tmp_hal_status = ADC_Disable(hadc);

     
    if (tmp_hal_status == HAL_OK)
    {
       
      (((hadc->State)) = ((((((hadc->State))) & (~((0x00000100UL) | (0x00001000UL)))) | ((0x00000001UL)))));


    }
  }

   
  do{ (hadc)->Lock = HAL_UNLOCKED; }while (0U);

   
  return tmp_hal_status;
}













 
HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef *hadc, uint32_t *pData, uint32_t Length)
{
  HAL_StatusTypeDef tmp_hal_status;

  uint32_t tmp_multimode_config = LL_ADC_GetMultimode((((ADC_Common_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x08000300UL))));


   
  ((void)0U);

   
  if (LL_ADC_REG_IsConversionOngoing(hadc->Instance) == 0UL)
  {
     
    do{ if((hadc)->Lock == HAL_LOCKED) { return HAL_BUSY; } else { (hadc)->Lock = HAL_LOCKED; } }while (0U);


     
     
    if ((tmp_multimode_config == (0x00000000UL))
        || (tmp_multimode_config == ( (0x04UL << (0U)) | (0x01UL << (0U))))
        || (tmp_multimode_config == ((0x08UL << (0U)) | (0x01UL << (0U))))
       )

    {
       
      tmp_hal_status = ADC_Enable(hadc);

       
      if (tmp_hal_status == HAL_OK)
      {
         
         
         
        (((hadc->State)) = ((((((hadc->State))) & (~((0x00000001UL) | (0x00000200UL) | (0x00000400UL) | (0x00000800UL)))) | ((0x00000100UL)))));




        

 
        if ((( ( ((hadc->Instance) == ((ADC_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x08000100UL))) )? (((ADC_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x08000000UL))) : (hadc->Instance) ) == hadc->Instance)
            || (tmp_multimode_config == (0x00000000UL))
           )
        {
          ((hadc->State) &= ~((0x00100000UL)));
        }


         
        if ((hadc->State & (0x00001000UL)) != 0UL)
        {
           
          ((hadc->ErrorCode) &= ~(((0x02U) | (0x04U))));
        }
        else
        {
           
          ((hadc)->ErrorCode = (0x00U));
        }

         
        hadc->DMA_Handle->XferCpltCallback = ADC_DMAConvCplt;

         
        hadc->DMA_Handle->XferHalfCpltCallback = ADC_DMAHalfConvCplt;

         
        hadc->DMA_Handle->XferErrorCallback = ADC_DMAError;


         
         

         
         
         
        (((hadc)->Instance ->ISR) = (((0x1UL << (2U)) | (0x1UL << (3U)) | (0x1UL << (4U)))));

         
         
         
        do{ (hadc)->Lock = HAL_UNLOCKED; }while (0U);

        

 
        (((hadc)->Instance ->IER) |= ((0x1UL << (4U))));

         
        ((hadc->Instance ->CFGR) |= ((0x1UL << (0U))));

         
        tmp_hal_status = HAL_DMA_Start_IT(hadc->DMA_Handle, (uint32_t)&hadc->Instance->DR, (uint32_t)pData, Length);

         
         
         
         
         
        LL_ADC_REG_StartConversion(hadc->Instance);
      }
      else
      {
         
        do{ (hadc)->Lock = HAL_UNLOCKED; }while (0U);
      }

    }

    else
    {
      tmp_hal_status = HAL_ERROR;
       
      do{ (hadc)->Lock = HAL_UNLOCKED; }while (0U);
    }

  }
  else
  {
    tmp_hal_status = HAL_BUSY;
  }

   
  return tmp_hal_status;
}













 
HAL_StatusTypeDef HAL_ADC_Stop_DMA(ADC_HandleTypeDef *hadc)
{
  HAL_StatusTypeDef tmp_hal_status;

   
  ((void)0U);

   
  do{ if((hadc)->Lock == HAL_LOCKED) { return HAL_BUSY; } else { (hadc)->Lock = HAL_LOCKED; } }while (0U);

   
  tmp_hal_status = ADC_ConversionStop(hadc, ((0x00000003UL)));

   
  if (tmp_hal_status == HAL_OK)
  {
     
    ((hadc->Instance ->CFGR) &= ~((0x1UL << (0U))));

     
     
    if (hadc->DMA_Handle->State == HAL_DMA_STATE_BUSY)
    {
      tmp_hal_status = HAL_DMA_Abort(hadc->DMA_Handle);

       
      if (tmp_hal_status != HAL_OK)
      {
         
        ((hadc->State) |= ((0x00000040UL)));
      }
    }

     
    (((hadc)->Instance ->IER) &= ~((0x1UL << (4U))));

     
     
     
    if (tmp_hal_status == HAL_OK)
    {
      tmp_hal_status = ADC_Disable(hadc);
    }
    else
    {
      (void)ADC_Disable(hadc);
    }

     
    if (tmp_hal_status == HAL_OK)
    {
       
      (((hadc->State)) = ((((((hadc->State))) & (~((0x00000100UL) | (0x00001000UL)))) | ((0x00000001UL)))));


    }

  }

   
  do{ (hadc)->Lock = HAL_UNLOCKED; }while (0U);

   
  return tmp_hal_status;
}



















 
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *hadc)
{
   
  ((void)0U);

   
   

   
  return hadc->Instance->DR;
}









 
HAL_StatusTypeDef HAL_ADC_StartSampling(ADC_HandleTypeDef *hadc)
{
   
  ((void)0U);

   
  ((hadc->Instance ->CFGR2) |= ((0x1UL << (25U))));

   
  return HAL_OK;
}










 
HAL_StatusTypeDef HAL_ADC_StopSampling(ADC_HandleTypeDef *hadc)
{
   
  ((void)0U);

   
  ((hadc->Instance ->CFGR2) &= ~((0x1UL << (25U))));

   
  return HAL_OK;
}





 
void HAL_ADC_IRQHandler(ADC_HandleTypeDef *hadc)
{
  uint32_t overrun_error = 0UL;  
  uint32_t tmp_isr = hadc->Instance->ISR;
  uint32_t tmp_ier = hadc->Instance->IER;
  uint32_t tmp_adc_inj_is_trigger_source_sw_start;
  uint32_t tmp_adc_reg_is_trigger_source_sw_start;
  uint32_t tmp_cfgr;

  const ADC_TypeDef *tmpADC_Master;
  uint32_t tmp_multimode_config = LL_ADC_GetMultimode((((ADC_Common_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x08000300UL))));


   
  ((void)0U);
  ((void)0U);

   
  if (((tmp_isr & (0x1UL << (1U))) == (0x1UL << (1U))) && ((tmp_ier & (0x1UL << (1U))) == (0x1UL << (1U))))
  {
     
    if ((hadc->State & (0x00000010UL)) == 0UL)
    {
       
      ((hadc->State) |= ((0x00000800UL)));
    }

     



    HAL_ADCEx_EndOfSamplingCallback(hadc);


     
    (((hadc)->Instance ->ISR) = ((0x1UL << (1U))));
  }

   
  if ((((tmp_isr & (0x1UL << (2U))) == (0x1UL << (2U))) && ((tmp_ier & (0x1UL << (2U))) == (0x1UL << (2U)))) ||
      (((tmp_isr & (0x1UL << (3U))) == (0x1UL << (3U))) && ((tmp_ier & (0x1UL << (3U))) == (0x1UL << (3U)))))
  {
     
    if ((hadc->State & (0x00000010UL)) == 0UL)
    {
       
      ((hadc->State) |= ((0x00000200UL)));
    }

     
     
     
    if (LL_ADC_REG_IsTriggerSourceSWStart(hadc->Instance) != 0UL)
    {
       
       
       

      if ((( ( ((hadc->Instance) == ((ADC_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x08000100UL))) )? (((ADC_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x08000000UL))) : (hadc->Instance) ) == hadc->Instance)
          || (tmp_multimode_config == (0x00000000UL))
          || (tmp_multimode_config == ( (0x04UL << (0U)) | (0x01UL << (0U))))
          || (tmp_multimode_config == ((0x08UL << (0U)) | (0x01UL << (0U))))
         )
      {
         
        tmp_cfgr = ((hadc->Instance ->CFGR));
      }
      else
      {
         
        tmpADC_Master = ( ( ((hadc->Instance) == ((ADC_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x08000100UL))) )? (((ADC_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x08000000UL))) : (hadc->Instance) );
        tmp_cfgr = ((tmpADC_Master->CFGR));
      }




       
      if (((tmp_cfgr) & ((0x1UL << (13U)))) != (0x1UL << (13U)))
      {
         
        if (((((hadc)->Instance ->ISR) & ((0x1UL << (3U)))) == ((0x1UL << (3U)))))
        {
           
           
          if (LL_ADC_REG_IsConversionOngoing(hadc->Instance) == 0UL)
          {
             
             
             
             
            (((hadc)->Instance ->IER) &= ~((0x1UL << (2U)) | (0x1UL << (3U))));

             
            ((hadc->State) &= ~((0x00000100UL)));

            if ((hadc->State & (0x00001000UL)) == 0UL)
            {
              ((hadc->State) |= ((0x00000001UL)));
            }
          }
          else
          {
             
            ((hadc->State) |= ((0x00000010UL)));

             
            ((hadc->ErrorCode) |= ((0x01U)));
          }
        }
      }
    }

     
     
     
     
     



    HAL_ADC_ConvCpltCallback(hadc);


     
     
     
     
     
    (((hadc)->Instance ->ISR) = (((0x1UL << (2U)) | (0x1UL << (3U)))));
  }

   
  if ((((tmp_isr & (0x1UL << (5U))) == (0x1UL << (5U))) && ((tmp_ier & (0x1UL << (5U))) == (0x1UL << (5U)))) ||
      (((tmp_isr & (0x1UL << (6U))) == (0x1UL << (6U))) && ((tmp_ier & (0x1UL << (6U))) == (0x1UL << (6U)))))
  {
     
    if ((hadc->State & (0x00000010UL)) == 0UL)
    {
       
      ((hadc->State) |= ((0x00002000UL)));
    }

     
    tmp_adc_inj_is_trigger_source_sw_start = LL_ADC_INJ_IsTriggerSourceSWStart(hadc->Instance);
    tmp_adc_reg_is_trigger_source_sw_start = LL_ADC_REG_IsTriggerSourceSWStart(hadc->Instance);
     
     
     

    if ((( ( ((hadc->Instance) == ((ADC_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x08000100UL))) )? (((ADC_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x08000000UL))) : (hadc->Instance) ) == hadc->Instance)
        || (tmp_multimode_config == (0x00000000UL))
        || (tmp_multimode_config == ( (0x04UL << (0U)) | (0x02UL << (0U)) ))
        || (tmp_multimode_config == ( (0x04UL << (0U)) | (0x02UL << (0U)) | (0x01UL << (0U))))
       )
    {
      tmp_cfgr = ((hadc->Instance ->CFGR));
    }
    else
    {
      tmpADC_Master = ( ( ((hadc->Instance) == ((ADC_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x08000100UL))) )? (((ADC_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x08000000UL))) : (hadc->Instance) );
      tmp_cfgr = ((tmpADC_Master->CFGR));
    }




     
     
     
     
     
    if (tmp_adc_inj_is_trigger_source_sw_start != 0UL)
    {
      if ((((tmp_cfgr) & ((0x1UL << (25U)))) == 0UL) ||
          ((tmp_adc_reg_is_trigger_source_sw_start != 0UL) &&
           (((tmp_cfgr) & ((0x1UL << (13U)))) == 0UL)))
      {
         
        if (((((hadc)->Instance ->ISR) & ((0x1UL << (6U)))) == ((0x1UL << (6U)))))
        {
           
           
           
           
           
           
          if (((tmp_cfgr) & ((0x1UL << (21U)))) == 0UL)
          {
             
             
            if (LL_ADC_INJ_IsConversionOngoing(hadc->Instance) == 0UL)
            {
               
              (((hadc)->Instance ->IER) &= ~((0x1UL << (5U)) | (0x1UL << (6U))));

               
              ((hadc->State) &= ~((0x00001000UL)));

              if ((hadc->State & (0x00000100UL)) == 0UL)
              {
                ((hadc->State) |= ((0x00000001UL)));
              }
            }
            else
            {
               
              ((hadc->State) |= ((0x00000010UL)));

               
              ((hadc->ErrorCode) |= ((0x01U)));
            }
          }
        }
      }
    }

     
    



 



    HAL_ADCEx_InjectedConvCpltCallback(hadc);


     
    (((hadc)->Instance ->ISR) = ((0x1UL << (5U)) | (0x1UL << (6U))));
  }

   
  if (((tmp_isr & (0x1UL << (7U))) == (0x1UL << (7U))) && ((tmp_ier & (0x1UL << (7U))) == (0x1UL << (7U))))
  {
     
    ((hadc->State) |= ((0x00010000UL)));

     



    HAL_ADC_LevelOutOfWindowCallback(hadc);


     
    (((hadc)->Instance ->ISR) = ((0x1UL << (7U))));
  }

   
  if (((tmp_isr & (0x1UL << (8U))) == (0x1UL << (8U))) && ((tmp_ier & (0x1UL << (8U))) == (0x1UL << (8U))))
  {
     
    ((hadc->State) |= ((0x00020000UL)));

     



    HAL_ADCEx_LevelOutOfWindow2Callback(hadc);


     
    (((hadc)->Instance ->ISR) = ((0x1UL << (8U))));
  }

   
  if (((tmp_isr & (0x1UL << (9U))) == (0x1UL << (9U))) && ((tmp_ier & (0x1UL << (9U))) == (0x1UL << (9U))))
  {
     
    ((hadc->State) |= ((0x00040000UL)));

     



    HAL_ADCEx_LevelOutOfWindow3Callback(hadc);


     
    (((hadc)->Instance ->ISR) = ((0x1UL << (9U))));
  }

   
  if (((tmp_isr & (0x1UL << (4U))) == (0x1UL << (4U))) && ((tmp_ier & (0x1UL << (4U))) == (0x1UL << (4U))))
  {
     
     
     
     
     
     
    if (hadc->Init.Overrun == ((0x00000000UL)))
    {
      overrun_error = 1UL;
    }
    else
    {
       

      if (tmp_multimode_config != (0x00000000UL))
      {
        
 
        if (LL_ADC_GetMultiDMATransfer((((ADC_Common_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x08000300UL)))) != (0x00000000UL))
        {
          overrun_error = 1UL;
        }
      }
      else

      {
         
        if ((hadc->Instance->CFGR & (0x1UL << (0U))) != 0UL)
        {
          overrun_error = 1UL;
        }
      }
    }

    if (overrun_error == 1UL)
    {
       
      ((hadc->State) |= ((0x00000400UL)));

       
      ((hadc->ErrorCode) |= ((0x02U)));

       
       
       
       
       



      HAL_ADC_ErrorCallback(hadc);

    }

     
    (((hadc)->Instance ->ISR) = ((0x1UL << (4U))));
  }

   
  if (((tmp_isr & (0x1UL << (10U))) == (0x1UL << (10U))) && ((tmp_ier & (0x1UL << (10U))) == (0x1UL << (10U))))
  {
     
    ((hadc->State) |= ((0x00004000UL)));

     
    ((hadc->ErrorCode) |= ((0x08U)));

     
    (((hadc)->Instance ->ISR) = ((0x1UL << (10U))));

     



    HAL_ADCEx_InjectedQueueOverflowCallback(hadc);

  }

}





 
__weak void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
   
  (void)hadc;

  

 
}





 
__weak void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
   
  (void)hadc;

  

 
}





 
__weak void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef *hadc)
{
   
  (void)hadc;

  

 
}












 
__weak void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
   
  (void)hadc;

  

 
}



 














 

















 
HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef *hadc, ADC_ChannelConfTypeDef *sConfig)
{
  HAL_StatusTypeDef tmp_hal_status = HAL_OK;
  uint32_t tmpOffsetShifted;
  uint32_t tmp_config_internal_channel;
  volatile uint32_t wait_loop_index = 0UL;
  uint32_t tmp_adc_is_conversion_on_going_regular;
  uint32_t tmp_adc_is_conversion_on_going_injected;

   
  ((void)0U);
  ((void)0U);
  ((void)0U);
  ((void)0U);
  ((void)0U);
  ((void)0U);

  
 
  ((void)0U);

   
  if (sConfig->SingleDiff != (((0x1UL << (30U)) | (0x7FUL << (16U)))))
  {
    ((void)0U);
  }
  else
  {
    ((void)0U);
  }

   
  do{ if((hadc)->Lock == HAL_LOCKED) { return HAL_BUSY; } else { (hadc)->Lock = HAL_LOCKED; } }while (0U);

   
   
   
   
   
  if (LL_ADC_REG_IsConversionOngoing(hadc->Instance) == 0UL)
  {
     
    LL_ADC_REG_SetSequencerRanks(hadc->Instance, sConfig->Rank, sConfig->Channel);

     
     
     
     
     
    tmp_adc_is_conversion_on_going_regular = LL_ADC_REG_IsConversionOngoing(hadc->Instance);
    tmp_adc_is_conversion_on_going_injected = LL_ADC_INJ_IsConversionOngoing(hadc->Instance);
    if ((tmp_adc_is_conversion_on_going_regular == 0UL)
        && (tmp_adc_is_conversion_on_going_injected == 0UL)
       )
    {
       
      if (sConfig->SamplingTime == ((0x1UL << (31U)) | (0x00000000UL)))
      {
         
        LL_ADC_SetChannelSamplingTime(hadc->Instance, sConfig->Channel, (0x00000000UL));

         
        LL_ADC_SetSamplingTimeCommonConfig(hadc->Instance, ((0x1UL << (31U))));
      }
      else
      {
         
        LL_ADC_SetChannelSamplingTime(hadc->Instance, sConfig->Channel, sConfig->SamplingTime);

         
        LL_ADC_SetSamplingTimeCommonConfig(hadc->Instance, (0x00000000UL));
      }

       

       
       
      tmpOffsetShifted = (((uint32_t)sConfig->Offset) << ((((hadc)->Instance ->CFGR & (0x3UL << (3U))) >> 3UL) * 2UL));

      if (sConfig->OffsetNumber != (((0x00000003UL)) + 1U))
      {
         
        LL_ADC_SetOffset(hadc->Instance, sConfig->OffsetNumber, sConfig->Channel, tmpOffsetShifted);

        ((void)0U);
        ((void)0U);
         
        LL_ADC_SetOffsetSign(hadc->Instance, sConfig->OffsetNumber, sConfig->OffsetSign);
        LL_ADC_SetOffsetSaturation(hadc->Instance, sConfig->OffsetNumber, (sConfig->OffsetSaturation == ENABLE) ? ((0x1UL << (25U))) : (0x00000000UL));
      }
      else
      {
         
         
        if (((((LL_ADC_GetOffsetChannel(hadc->Instance, (0x00000000UL))) & ((0x7FFFFUL << (0U)))) == 0UL) ? ( ((LL_ADC_GetOffsetChannel(hadc->Instance, (0x00000000UL))) & ((0x1FUL << (26U)))) >> ((26U)) ) : ( (uint32_t)(__clz(__rbit((LL_ADC_GetOffsetChannel(hadc->Instance, (0x00000000UL)))))) ) )
            == ((((sConfig->Channel) & ((0x7FFFFUL << (0U)))) == 0UL) ? ( ((sConfig->Channel) & ((0x1FUL << (26U)))) >> ((26U)) ) : ( (uint32_t)(__clz(__rbit((sConfig->Channel)))) ) ))
        {
          LL_ADC_SetOffsetState(hadc->Instance, (0x00000000UL), (0x00000000UL));
        }
        if (((((LL_ADC_GetOffsetChannel(hadc->Instance, (0x00000001UL))) & ((0x7FFFFUL << (0U)))) == 0UL) ? ( ((LL_ADC_GetOffsetChannel(hadc->Instance, (0x00000001UL))) & ((0x1FUL << (26U)))) >> ((26U)) ) : ( (uint32_t)(__clz(__rbit((LL_ADC_GetOffsetChannel(hadc->Instance, (0x00000001UL)))))) ) )
            == ((((sConfig->Channel) & ((0x7FFFFUL << (0U)))) == 0UL) ? ( ((sConfig->Channel) & ((0x1FUL << (26U)))) >> ((26U)) ) : ( (uint32_t)(__clz(__rbit((sConfig->Channel)))) ) ))
        {
          LL_ADC_SetOffsetState(hadc->Instance, (0x00000001UL), (0x00000000UL));
        }
        if (((((LL_ADC_GetOffsetChannel(hadc->Instance, (0x00000002UL))) & ((0x7FFFFUL << (0U)))) == 0UL) ? ( ((LL_ADC_GetOffsetChannel(hadc->Instance, (0x00000002UL))) & ((0x1FUL << (26U)))) >> ((26U)) ) : ( (uint32_t)(__clz(__rbit((LL_ADC_GetOffsetChannel(hadc->Instance, (0x00000002UL)))))) ) )
            == ((((sConfig->Channel) & ((0x7FFFFUL << (0U)))) == 0UL) ? ( ((sConfig->Channel) & ((0x1FUL << (26U)))) >> ((26U)) ) : ( (uint32_t)(__clz(__rbit((sConfig->Channel)))) ) ))
        {
          LL_ADC_SetOffsetState(hadc->Instance, (0x00000002UL), (0x00000000UL));
        }
        if (((((LL_ADC_GetOffsetChannel(hadc->Instance, (0x00000003UL))) & ((0x7FFFFUL << (0U)))) == 0UL) ? ( ((LL_ADC_GetOffsetChannel(hadc->Instance, (0x00000003UL))) & ((0x1FUL << (26U)))) >> ((26U)) ) : ( (uint32_t)(__clz(__rbit((LL_ADC_GetOffsetChannel(hadc->Instance, (0x00000003UL)))))) ) )
            == ((((sConfig->Channel) & ((0x7FFFFUL << (0U)))) == 0UL) ? ( ((sConfig->Channel) & ((0x1FUL << (26U)))) >> ((26U)) ) : ( (uint32_t)(__clz(__rbit((sConfig->Channel)))) ) ))
        {
          LL_ADC_SetOffsetState(hadc->Instance, (0x00000003UL), (0x00000000UL));
        }
      }
    }

     
     
     
    if (LL_ADC_IsEnabled(hadc->Instance) == 0UL)
    {
       
      LL_ADC_SetChannelSingleDiff(hadc->Instance, sConfig->Channel, sConfig->SingleDiff);

       
      if (sConfig->SingleDiff == (((0x1UL << (30U)) | (0x7FUL << (16U)))))
      {
         
         
        LL_ADC_SetChannelSamplingTime(hadc->Instance,
                                      (uint32_t)((((((((((uint32_t)sConfig->Channel) & ((0x7FFFFUL << (0U)))) == 0UL) ? ( (((uint32_t)sConfig->Channel) & ((0x1FUL << (26U)))) >> ((26U)) ) : ( (uint32_t)(__clz(__rbit(((uint32_t)sConfig->Channel)))) ) ) + 1UL) & 0x1FUL) <= 9UL) ? ( ((((((((uint32_t)sConfig->Channel) & ((0x7FFFFUL << (0U)))) == 0UL) ? ( (((uint32_t)sConfig->Channel) & ((0x1FUL << (26U)))) >> ((26U)) ) : ( (uint32_t)(__clz(__rbit(((uint32_t)sConfig->Channel)))) ) ) + 1UL) & 0x1FUL) << ((26U))) | ((0x00001UL << (0U)) << (((((((uint32_t)sConfig->Channel) & ((0x7FFFFUL << (0U)))) == 0UL) ? ( (((uint32_t)sConfig->Channel) & ((0x1FUL << (26U)))) >> ((26U)) ) : ( (uint32_t)(__clz(__rbit(((uint32_t)sConfig->Channel)))) ) ) + 1UL) & 0x1FUL)) | ((0x00000000UL) | (((3UL * (((((((uint32_t)sConfig->Channel) & ((0x7FFFFUL << (0U)))) == 0UL) ? ( (((uint32_t)sConfig->Channel) & ((0x1FUL << (26U)))) >> ((26U)) ) : ( (uint32_t)(__clz(__rbit(((uint32_t)sConfig->Channel)))) ) ) + 1UL) & 0x1FUL))) << (20UL))) ) : ( ((((((((uint32_t)sConfig->Channel) & ((0x7FFFFUL << (0U)))) == 0UL) ? ( (((uint32_t)sConfig->Channel) & ((0x1FUL << (26U)))) >> ((26U)) ) : ( (uint32_t)(__clz(__rbit(((uint32_t)sConfig->Channel)))) ) ) + 1UL) & 0x1FUL) << ((26U))) | ((0x00001UL << (0U)) << (((((((uint32_t)sConfig->Channel) & ((0x7FFFFUL << (0U)))) == 0UL) ? ( (((uint32_t)sConfig->Channel) & ((0x1FUL << (26U)))) >> ((26U)) ) : ( (uint32_t)(__clz(__rbit(((uint32_t)sConfig->Channel)))) ) ) + 1UL) & 0x1FUL)) | ((0x02000000UL) | (((3UL * ((((((((uint32_t)sConfig->Channel) & ((0x7FFFFUL << (0U)))) == 0UL) ? ( (((uint32_t)sConfig->Channel) & ((0x1FUL << (26U)))) >> ((26U)) ) : ( (uint32_t)(__clz(__rbit(((uint32_t)sConfig->Channel)))) ) ) + 1UL) & 0x1FUL) - 10UL))) << (20UL))) ) )),
                                      sConfig->SamplingTime);
      }

    }

     
     
     
     
     

    if ((((sConfig->Channel) & ((0x80000000UL) | (0x00080000UL))) != 0UL))
    {
      tmp_config_internal_channel = LL_ADC_GetCommonPathInternalCh((((ADC_Common_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x08000300UL))));

       
       
      if (((sConfig->Channel == (((((0x10UL << (26U))) | ((0x02000000UL) | ((18UL) << (20UL))) | ((0x10000UL << (0U)))) | (0x80000000UL)))) || (sConfig->Channel == (((((0x04UL << (26U))) | ((0x00000000UL) | ((12UL) << (20UL))) | ((0x00010UL << (0U))) ) | (0x80000000UL)))))
          && ((tmp_config_internal_channel & ((0x1UL << (23U)))) == 0UL))
      {
        if ((((hadc)->Instance) == ((ADC_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x08000000UL))))
        {
          LL_ADC_SetCommonPathInternalCh((((ADC_Common_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x08000300UL))),
                                         ((0x1UL << (23U))) | tmp_config_internal_channel);

           
           
           
           
           
          wait_loop_index = (((120UL) / 10UL) * ((SystemCoreClock / (100000UL * 2UL)) + 1UL));
          while (wait_loop_index != 0UL)
          {
            wait_loop_index--;
          }
        }
      }
      else if ((sConfig->Channel == (((((0x10UL << (26U)) | (0x01UL << (26U))) | ((0x02000000UL) | ((21UL) << (20UL))) | ((0x20000UL << (0U)))) | (0x80000000UL)))) && ((tmp_config_internal_channel & ((0x1UL << (24U)))) == 0UL))
      {
        if ((((hadc)->Instance) != ((ADC_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x08000100UL))))
        {
          LL_ADC_SetCommonPathInternalCh((((ADC_Common_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x08000300UL))),
                                         ((0x1UL << (24U))) | tmp_config_internal_channel);
        }
      }
      else if ((sConfig->Channel == (((((0x10UL << (26U)) | (0x02UL << (26U))) | ((0x02000000UL) | ((24UL) << (20UL))) | ((0x40000UL << (0U)))) | (0x80000000UL))))
               && ((tmp_config_internal_channel & ((0x1UL << (22U)))) == 0UL))
      {
        if ((((hadc)->Instance) != ((ADC_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x08000100UL))))
        {
          LL_ADC_SetCommonPathInternalCh((((ADC_Common_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x08000300UL))),
                                         ((0x1UL << (22U))) | tmp_config_internal_channel);
        }
      }
      else
      {
         
      }
    }
  }

   
   
   
  else
  {
     
    ((hadc->State) |= ((0x00000020UL)));

    tmp_hal_status = HAL_ERROR;
  }

   
  do{ (hadc)->Lock = HAL_UNLOCKED; }while (0U);

   
  return tmp_hal_status;
}






















 
HAL_StatusTypeDef HAL_ADC_AnalogWDGConfig(ADC_HandleTypeDef *hadc, ADC_AnalogWDGConfTypeDef *AnalogWDGConfig)
{
  HAL_StatusTypeDef tmp_hal_status = HAL_OK;
  uint32_t tmpAWDHighThresholdShifted;
  uint32_t tmpAWDLowThresholdShifted;
  uint32_t tmp_adc_is_conversion_on_going_regular;
  uint32_t tmp_adc_is_conversion_on_going_injected;

   
  ((void)0U);
  ((void)0U);
  ((void)0U);
  ((void)0U);
  ((void)0U);

  if ((AnalogWDGConfig->WatchdogMode == ((0x1UL << (22U)) | (0x1UL << (23U))))     ||
      (AnalogWDGConfig->WatchdogMode == ((0x1UL << (22U)) | (0x1UL << (24U))))   ||
      (AnalogWDGConfig->WatchdogMode == ((0x1UL << (22U)) | (0x1UL << (23U)) | (0x1UL << (24U)))))
  {
    ((void)0U);
  }

   
  if (hadc->Init.OversamplingMode == ENABLE)
  {
    

 
    ((void)0U);
    ((void)0U);
  }
  else
  {
     
    ((void)0U);
    ((void)0U);
  }

   
  do{ if((hadc)->Lock == HAL_LOCKED) { return HAL_BUSY; } else { (hadc)->Lock = HAL_LOCKED; } }while (0U);

   
   
   
   
  tmp_adc_is_conversion_on_going_regular = LL_ADC_REG_IsConversionOngoing(hadc->Instance);
  tmp_adc_is_conversion_on_going_injected = LL_ADC_INJ_IsConversionOngoing(hadc->Instance);
  if ((tmp_adc_is_conversion_on_going_regular == 0UL)
      && (tmp_adc_is_conversion_on_going_injected == 0UL)
     )
  {
     
    if (AnalogWDGConfig->WatchdogNumber == ((((0x1FUL << (26U)) | (0x1UL << (24U)) | (0x1UL << (23U)) | (0x1UL << (22U))) | (0x00000000UL))))
    {
       
       
       
      switch (AnalogWDGConfig->WatchdogMode)
      {
        case ((0x1UL << (22U)) | (0x1UL << (23U))):
          LL_ADC_SetAnalogWDMonitChannels(hadc->Instance, (((0x1FUL << (26U)) | (0x1UL << (24U)) | (0x1UL << (23U)) | (0x1UL << (22U))) | (0x00000000UL)), ((((0x00000001UL)) == (0x00000001UL)) ? (((AnalogWDGConfig->Channel) & (((0x1FUL << (26U))) | ((0x7FFFFUL << (0U))) | ((0x80000000UL) | (0x00080000UL)))) | (0x1UL << (23U)) | (0x1UL << (22U))) : (((0x00000001UL)) == (0x00000002UL)) ? (((AnalogWDGConfig->Channel) & (((0x1FUL << (26U))) | ((0x7FFFFUL << (0U))) | ((0x80000000UL) | (0x00080000UL)))) | (0x1UL << (24U)) | (0x1UL << (22U))) : (((AnalogWDGConfig->Channel) & (((0x1FUL << (26U))) | ((0x7FFFFUL << (0U))) | ((0x80000000UL) | (0x00080000UL)))) | (0x1UL << (24U)) | (0x1UL << (23U)) | (0x1UL << (22U))) ));

          break;

        case ((0x1UL << (22U)) | (0x1UL << (24U))):
          LL_ADC_SetAnalogWDMonitChannels(hadc->Instance, (((0x1FUL << (26U)) | (0x1UL << (24U)) | (0x1UL << (23U)) | (0x1UL << (22U))) | (0x00000000UL)), ((((0x00000002UL)) == (0x00000001UL)) ? (((AnalogWDGConfig->Channel) & (((0x1FUL << (26U))) | ((0x7FFFFUL << (0U))) | ((0x80000000UL) | (0x00080000UL)))) | (0x1UL << (23U)) | (0x1UL << (22U))) : (((0x00000002UL)) == (0x00000002UL)) ? (((AnalogWDGConfig->Channel) & (((0x1FUL << (26U))) | ((0x7FFFFUL << (0U))) | ((0x80000000UL) | (0x00080000UL)))) | (0x1UL << (24U)) | (0x1UL << (22U))) : (((AnalogWDGConfig->Channel) & (((0x1FUL << (26U))) | ((0x7FFFFUL << (0U))) | ((0x80000000UL) | (0x00080000UL)))) | (0x1UL << (24U)) | (0x1UL << (23U)) | (0x1UL << (22U))) ));

          break;

        case ((0x1UL << (22U)) | (0x1UL << (23U)) | (0x1UL << (24U))):
          LL_ADC_SetAnalogWDMonitChannels(hadc->Instance, (((0x1FUL << (26U)) | (0x1UL << (24U)) | (0x1UL << (23U)) | (0x1UL << (22U))) | (0x00000000UL)), ((((0x00000003UL)) == (0x00000001UL)) ? (((AnalogWDGConfig->Channel) & (((0x1FUL << (26U))) | ((0x7FFFFUL << (0U))) | ((0x80000000UL) | (0x00080000UL)))) | (0x1UL << (23U)) | (0x1UL << (22U))) : (((0x00000003UL)) == (0x00000002UL)) ? (((AnalogWDGConfig->Channel) & (((0x1FUL << (26U))) | ((0x7FFFFUL << (0U))) | ((0x80000000UL) | (0x00080000UL)))) | (0x1UL << (24U)) | (0x1UL << (22U))) : (((AnalogWDGConfig->Channel) & (((0x1FUL << (26U))) | ((0x7FFFFUL << (0U))) | ((0x80000000UL) | (0x00080000UL)))) | (0x1UL << (24U)) | (0x1UL << (23U)) | (0x1UL << (22U))) ));

          break;

        case ((0x1UL << (23U))):
          LL_ADC_SetAnalogWDMonitChannels(hadc->Instance, (((0x1FUL << (26U)) | (0x1UL << (24U)) | (0x1UL << (23U)) | (0x1UL << (22U))) | (0x00000000UL)), (((0x7FFFFUL << (0U))) | (0x1UL << (23U)) ));
          break;

        case ((0x1UL << (24U))):
          LL_ADC_SetAnalogWDMonitChannels(hadc->Instance, (((0x1FUL << (26U)) | (0x1UL << (24U)) | (0x1UL << (23U)) | (0x1UL << (22U))) | (0x00000000UL)), (((0x7FFFFUL << (0U))) | (0x1UL << (24U)) ));
          break;

        case ((0x1UL << (23U)) | (0x1UL << (24U))):
          LL_ADC_SetAnalogWDMonitChannels(hadc->Instance, (((0x1FUL << (26U)) | (0x1UL << (24U)) | (0x1UL << (23U)) | (0x1UL << (22U))) | (0x00000000UL)), (((0x7FFFFUL << (0U))) | (0x1UL << (24U)) | (0x1UL << (23U)) ));
          break;

        default:  
          LL_ADC_SetAnalogWDMonitChannels(hadc->Instance, (((0x1FUL << (26U)) | (0x1UL << (24U)) | (0x1UL << (23U)) | (0x1UL << (22U))) | (0x00000000UL)), (0x00000000UL));
          break;
      }

       
      (((hadc->Instance ->TR1)) = ((((((hadc->Instance ->TR1))) & (~((0x7UL << (12U))))) | (AnalogWDGConfig->FilteringConfig))));



       
      ((hadc->State) &= ~((0x00010000UL)));

       
       
       
       
      LL_ADC_ClearFlag_AWD1(hadc->Instance);

       
      if (AnalogWDGConfig->ITMode == ENABLE)
      {
        LL_ADC_EnableIT_AWD1(hadc->Instance);
      }
      else
      {
        LL_ADC_DisableIT_AWD1(hadc->Instance);
      }
    }
     
    else
    {
      switch (AnalogWDGConfig->WatchdogMode)
      {
        case ((0x1UL << (22U)) | (0x1UL << (23U))):
        case ((0x1UL << (22U)) | (0x1UL << (24U))):
        case ((0x1UL << (22U)) | (0x1UL << (23U)) | (0x1UL << (24U))):
           
           
          if (AnalogWDGConfig->WatchdogNumber == ((((0x7FFFFUL << (0U))) | (0x00100000UL))))
          {
            ((hadc->Instance ->AWD2CR) |= ((1UL << (((((AnalogWDGConfig->Channel) & ((0x7FFFFUL << (0U)))) == 0UL) ? ( ((AnalogWDGConfig->Channel) & ((0x1FUL << (26U)))) >> ((26U)) ) : ( (uint32_t)(__clz(__rbit((AnalogWDGConfig->Channel)))) ) ) & 0x1FUL))));
          }
          else
          {
            ((hadc->Instance ->AWD3CR) |= ((1UL << (((((AnalogWDGConfig->Channel) & ((0x7FFFFUL << (0U)))) == 0UL) ? ( ((AnalogWDGConfig->Channel) & ((0x1FUL << (26U)))) >> ((26U)) ) : ( (uint32_t)(__clz(__rbit((AnalogWDGConfig->Channel)))) ) ) & 0x1FUL))));
          }
          break;

        case ((0x1UL << (23U))):
        case ((0x1UL << (24U))):
        case ((0x1UL << (23U)) | (0x1UL << (24U))):
          LL_ADC_SetAnalogWDMonitChannels(hadc->Instance, AnalogWDGConfig->WatchdogNumber, (((0x7FFFFUL << (0U))) | (0x1UL << (24U)) | (0x1UL << (23U)) ));
          break;

        default:  
          LL_ADC_SetAnalogWDMonitChannels(hadc->Instance, AnalogWDGConfig->WatchdogNumber, (0x00000000UL));
          break;
      }

      if (AnalogWDGConfig->WatchdogNumber == ((((0x7FFFFUL << (0U))) | (0x00100000UL))))
      {
         
        ((hadc->State) &= ~((0x00020000UL)));

         
         
         
         
        LL_ADC_ClearFlag_AWD2(hadc->Instance);

         
        if (AnalogWDGConfig->ITMode == ENABLE)
        {
          LL_ADC_EnableIT_AWD2(hadc->Instance);
        }
        else
        {
          LL_ADC_DisableIT_AWD2(hadc->Instance);
        }
      }
       
      else
      {
         
        ((hadc->State) &= ~((0x00040000UL)));

         
         
         
         
        LL_ADC_ClearFlag_AWD3(hadc->Instance);

         
        if (AnalogWDGConfig->ITMode == ENABLE)
        {
          LL_ADC_EnableIT_AWD3(hadc->Instance);
        }
        else
        {
          LL_ADC_DisableIT_AWD3(hadc->Instance);
        }
      }
    }

  }

   
  if (AnalogWDGConfig->WatchdogNumber == ((((0x1FUL << (26U)) | (0x1UL << (24U)) | (0x1UL << (23U)) | (0x1UL << (22U))) | (0x00000000UL))))
  {
     
     
     
    tmpAWDHighThresholdShifted = ((AnalogWDGConfig->HighThreshold) << ((((hadc)->Instance ->CFGR & (0x3UL << (3U))) >> 3UL) * 2UL));
    tmpAWDLowThresholdShifted  = ((AnalogWDGConfig->LowThreshold) << ((((hadc)->Instance ->CFGR & (0x3UL << (3U))) >> 3UL) * 2UL));
  }
   
  else
  {
     
     
     
    tmpAWDHighThresholdShifted = ((((hadc)->Instance ->CFGR & (0x3UL << (3U))) != ((0x2UL << (3U)) | (0x1UL << (3U)))) ? ((AnalogWDGConfig->HighThreshold) >> ((4UL - ((((hadc)->Instance ->CFGR & (0x3UL << (3U))) >> 3UL) * 2UL)) & 0x1FUL)) : ((AnalogWDGConfig->HighThreshold) << 2UL) );
    tmpAWDLowThresholdShifted  = ((((hadc)->Instance ->CFGR & (0x3UL << (3U))) != ((0x2UL << (3U)) | (0x1UL << (3U)))) ? ((AnalogWDGConfig->LowThreshold) >> ((4UL - ((((hadc)->Instance ->CFGR & (0x3UL << (3U))) >> 3UL) * 2UL)) & 0x1FUL)) : ((AnalogWDGConfig->LowThreshold) << 2UL) );
  }

   
  LL_ADC_ConfigAnalogWDThresholds(hadc->Instance, AnalogWDGConfig->WatchdogNumber, tmpAWDHighThresholdShifted,
                                  tmpAWDLowThresholdShifted);

   
  do{ (hadc)->Lock = HAL_UNLOCKED; }while (0U);

   
  return tmp_hal_status;
}




 
















 










 
uint32_t HAL_ADC_GetState(ADC_HandleTypeDef *hadc)
{
   
  ((void)0U);

   
  return hadc->State;
}





 
uint32_t HAL_ADC_GetError(ADC_HandleTypeDef *hadc)
{
   
  ((void)0U);

  return hadc->ErrorCode;
}



 



 



 










 
HAL_StatusTypeDef ADC_ConversionStop(ADC_HandleTypeDef *hadc, uint32_t ConversionGroup)
{
  uint32_t tickstart;
  uint32_t Conversion_Timeout_CPU_cycles = 0UL;
  uint32_t conversion_group_reassigned = ConversionGroup;
  uint32_t tmp_ADC_CR_ADSTART_JADSTART;
  uint32_t tmp_adc_is_conversion_on_going_regular;
  uint32_t tmp_adc_is_conversion_on_going_injected;

   
  ((void)0U);
  ((void)0U);

   
   
  tmp_adc_is_conversion_on_going_regular = LL_ADC_REG_IsConversionOngoing(hadc->Instance);
  tmp_adc_is_conversion_on_going_injected = LL_ADC_INJ_IsConversionOngoing(hadc->Instance);
  if ((tmp_adc_is_conversion_on_going_regular != 0UL)
      || (tmp_adc_is_conversion_on_going_injected != 0UL)
     )
  {
     
     
     
     
     
     
    if (((hadc->Instance->CFGR & (0x1UL << (25U))) != 0UL)
        && (hadc->Init.ContinuousConvMode == ENABLE)
        && (hadc->Init.LowPowerAutoWait == ENABLE)
       )
    {
       
      conversion_group_reassigned = ((0x00000001UL));

       
      while (((((hadc)->Instance ->ISR) & ((0x1UL << (6U)))) == ((0x1UL << (6U)))) == 0UL)
      {
        if (Conversion_Timeout_CPU_cycles >= ((653UL * 4096UL * 256UL) * 4UL))
        {
           
          ((hadc->State) |= ((0x00000010UL)));

           
          ((hadc->ErrorCode) |= ((0x01U)));

          return HAL_ERROR;
        }
        Conversion_Timeout_CPU_cycles ++;
      }

       
      (((hadc)->Instance ->ISR) = ((0x1UL << (6U))));
    }

     
    if (conversion_group_reassigned != ((0x00000002UL)))
    {
       
      if (LL_ADC_REG_IsConversionOngoing(hadc->Instance) != 0UL)
      {
        if (LL_ADC_IsDisableOngoing(hadc->Instance) == 0UL)
        {
           
          LL_ADC_REG_StopConversion(hadc->Instance);
        }
      }
    }

     
    if (conversion_group_reassigned != ((0x00000001UL)))
    {
       
      if (LL_ADC_INJ_IsConversionOngoing(hadc->Instance) != 0UL)
      {
        if (LL_ADC_IsDisableOngoing(hadc->Instance) == 0UL)
        {
           
          LL_ADC_INJ_StopConversion(hadc->Instance);
        }
      }
    }

     
    switch (conversion_group_reassigned)
    {
      case ((0x00000003UL)):
        tmp_ADC_CR_ADSTART_JADSTART = ((0x1UL << (2U)) | (0x1UL << (3U)));
        break;
      case ((0x00000002UL)):
        tmp_ADC_CR_ADSTART_JADSTART = (0x1UL << (3U));
        break;
       
      default:
        tmp_ADC_CR_ADSTART_JADSTART = (0x1UL << (2U));
        break;
    }

     
    tickstart = HAL_GetTick();

    while ((hadc->Instance->CR & tmp_ADC_CR_ADSTART_JADSTART) != 0UL)
    {
      if ((HAL_GetTick() - tickstart) > ( 5UL))
      {
         
        if ((hadc->Instance->CR & tmp_ADC_CR_ADSTART_JADSTART) != 0UL)
        {
           
          ((hadc->State) |= ((0x00000010UL)));

           
          ((hadc->ErrorCode) |= ((0x01U)));

          return HAL_ERROR;
        }
      }
    }

  }

   
  return HAL_OK;
}







 
HAL_StatusTypeDef ADC_Enable(ADC_HandleTypeDef *hadc)
{
  uint32_t tickstart;

   
   
   
   
  if (LL_ADC_IsEnabled(hadc->Instance) == 0UL)
  {
     
    if ((hadc->Instance->CR & ((0x1UL << (31U)) | (0x1UL << (5U)) | (0x1UL << (4U)) | (0x1UL << (3U)) | (0x1UL << (2U))
                               | (0x1UL << (1U)) | (0x1UL << (0U)))) != 0UL)
    {
       
      ((hadc->State) |= ((0x00000010UL)));

       
      ((hadc->ErrorCode) |= ((0x01U)));

      return HAL_ERROR;
    }

     
    LL_ADC_Enable(hadc->Instance);

     
    tickstart = HAL_GetTick();

    while (((((hadc)->Instance ->ISR) & ((0x1UL << (0U)))) == ((0x1UL << (0U)))) == 0UL)
    {
      




 
       
       
      if (LL_ADC_IsEnabled(hadc->Instance) == 0UL)
      {
        LL_ADC_Enable(hadc->Instance);
      }

      if ((HAL_GetTick() - tickstart) > (2UL))
      {
         
        if (((((hadc)->Instance ->ISR) & ((0x1UL << (0U)))) == ((0x1UL << (0U)))) == 0UL)
        {
           
          ((hadc->State) |= ((0x00000010UL)));

           
          ((hadc->ErrorCode) |= ((0x01U)));

          return HAL_ERROR;
        }
      }
    }
  }

   
  return HAL_OK;
}







 
HAL_StatusTypeDef ADC_Disable(ADC_HandleTypeDef *hadc)
{
  uint32_t tickstart;
  const uint32_t tmp_adc_is_disable_on_going = LL_ADC_IsDisableOngoing(hadc->Instance);

   
   
   
  if ((LL_ADC_IsEnabled(hadc->Instance) != 0UL)
      && (tmp_adc_is_disable_on_going == 0UL)
     )
  {
     
    if ((hadc->Instance->CR & ((0x1UL << (3U)) | (0x1UL << (2U)) | (0x1UL << (0U)))) == (0x1UL << (0U)))
    {
       
      LL_ADC_Disable(hadc->Instance);
      (((hadc)->Instance ->ISR) = (((0x1UL << (1U)) | (0x1UL << (0U)))));
    }
    else
    {
       
      ((hadc->State) |= ((0x00000010UL)));

       
      ((hadc->ErrorCode) |= ((0x01U)));

      return HAL_ERROR;
    }

     
     
    tickstart = HAL_GetTick();

    while ((hadc->Instance->CR & (0x1UL << (0U))) != 0UL)
    {
      if ((HAL_GetTick() - tickstart) > (2UL))
      {
         
        if ((hadc->Instance->CR & (0x1UL << (0U))) != 0UL)
        {
           
          ((hadc->State) |= ((0x00000010UL)));

           
          ((hadc->ErrorCode) |= ((0x01U)));

          return HAL_ERROR;
        }
      }
    }
  }

   
  return HAL_OK;
}





 
void ADC_DMAConvCplt(DMA_HandleTypeDef *hdma)
{
   
  ADC_HandleTypeDef *hadc = (ADC_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

   
  if ((hadc->State & ((0x00000010UL) | (0x00000040UL))) == 0UL)
  {
     
    ((hadc->State) |= ((0x00000200UL)));

     
     
     
     
    if ((hadc->Instance->ISR & (0x1UL << (3U))) != 0UL)
    {
       
      if (LL_ADC_REG_IsTriggerSourceSWStart(hadc->Instance) != 0UL)
      {
         
        if (((hadc->Instance ->CFGR) & ((0x1UL << (13U)))) == 0UL)
        {
           
          ((hadc->State) &= ~((0x00000100UL)));
          if ((hadc->State & (0x00001000UL)) == 0UL)
          {
            ((hadc->State) |= ((0x00000001UL)));
          }
        }
      }
    }
    else
    {
      
 
      if (((hadc->Instance ->CFGR) & ((0x1UL << (1U)))) == 0UL)
      {
         
        ((hadc->State) &= ~((0x00000100UL)));
        if ((hadc->State & (0x00001000UL)) == 0UL)
        {
          ((hadc->State) |= ((0x00000001UL)));
        }
      }
    }

     



    HAL_ADC_ConvCpltCallback(hadc);

  }
  else  
  {
    if ((hadc->State & (0x00000010UL)) != 0UL)
    {
       



      HAL_ADC_ErrorCallback(hadc);

    }
    else
    {
       
      hadc->DMA_Handle->XferErrorCallback(hdma);
    }
  }
}





 
void ADC_DMAHalfConvCplt(DMA_HandleTypeDef *hdma)
{
   
  ADC_HandleTypeDef *hadc = (ADC_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

   



  HAL_ADC_ConvHalfCpltCallback(hadc);

}





 
void ADC_DMAError(DMA_HandleTypeDef *hdma)
{
   
  ADC_HandleTypeDef *hadc = (ADC_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

   
  ((hadc->State) |= ((0x00000040UL)));

   
  ((hadc->ErrorCode) |= ((0x04U)));

   



  HAL_ADC_ErrorCallback(hadc);

}



 




 



 

 
