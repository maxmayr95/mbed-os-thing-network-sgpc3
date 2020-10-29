
   #ifndef MBED_ANALOGIN_H
   #define MBED_ANALOGIN_H
   
   #include "platform/platform.h"
   
   #if DEVICE_ANALOGIN || defined(DOXYGEN_ONLY)
   
   #include "hal/analogin_api.h"
   #include "platform/SingletonPtr.h"
   #include "platform/PlatformMutex.h"
   
   #include <cmath>
   
   namespace mbed {
    class AnalogIn {
    
    public:

        AnalogIn(const PinMap &pinmap, float vref = MBED_CONF_TARGET_DEFAULT_ADC_VREF);
        AnalogIn(const PinMap &&, float vref = MBED_CONF_TARGET_DEFAULT_ADC_VREF) = delete; // prevent passing of temporary objects
        AnalogIn(PinName pin, float vref = MBED_CONF_TARGET_DEFAULT_ADC_VREF);
    
        /** Read the input voltage, represented as a float in the range [0.0, 1.0]
         *
         * @returns A floating-point value representing the current input voltage, measured as a percentage
         */
        float read();
    
        /** Read the input voltage, represented as an unsigned short in the range [0x0, 0xFFFF]
         *
         * @returns
        *   16-bit unsigned short representing the current input voltage, normalized to a 16-bit value
        */
       unsigned short read_u16();
   
       float read_voltage();

       void set_reference_voltage(float vref);

       float get_reference_voltage() const;

       operator float()
       {
           // Underlying call is thread safe
           return read();
       }
   
       virtual ~AnalogIn()
       {
           lock();
           analogin_free(&_adc);
           unlock();
       }
   
   protected:
   #if !defined(DOXYGEN_ONLY)
       virtual void lock()
       {
           _mutex->lock();
       }
   
       virtual void unlock()
       {
           _mutex->unlock();
       }
   
       analogin_t _adc;
       static SingletonPtr<PlatformMutex> _mutex;
   
       float _vref;
   
   #endif //!defined(DOXYGEN_ONLY)
   
   };
   
   /** @}*/
   
   } // namespace mbed
   
   #endif
   
   #endif