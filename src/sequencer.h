#ifndef SEQUENCER_H
#define SEQUENCER_H
//includes
#include <stdint.h>
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include <DMAChannel.h>
#include <ADC.h>

#include <DMAChannel.h>
#include <ADC.h>


//macros
#define DAC_TOP (4095)


#define GATE1_PIN (22) 
#define GATE2_PIN (23)


//PDB config: software trigger, enable, interrupt enable, continuous mode, enable DMA triggering
#define PDB_CONFIG                                              \
    (PDB_SC_TRGSEL(15) | PDB_SC_PDBEN | PDB_SC_PDBIE | PDB_SC_CONT | \
     PDB_SC_DMAEN /*| PDB_SC_MULT(3)*/)

//useful envelope indexes 
#define PRESSED_IDX (0)

//debug
#define ATTACH_ISR (1)


#define MAX_ENV_SIZE 4096
#define MAX_SEQ_SIZE 512

//DMAs



class Sequencer{

    public:
        //public functions
        Sequencer();
        int init(uint16_t sequence_size, uint16_t envelope_res, uint16_t bpm);
        int run();
        int stop();

    private:
        //private functions
        void pins_setup();
        uint8_t dac_setup(uint8_t channel);
        void lut_seq_setup(uint16_t seq_size);
        uint8_t lut_env_setup(uint16_t attack, uint16_t decay, uint16_t sustain, uint16_t release, uint16_t env_size);
        void dma_seq_setup(uint16_t seq_size) ;
        void dma_env_setup(uint16_t env_size);
        void dma_gate_setup();
        void dma_div_setup(uint8_t num_beats, uint16_t env_size);
        void trigger_clock_setup(uint16_t bps, uint16_t env_size);

        //dma objects
        DMAChannel dma_seq; //dma for CV sequence on DAC0
        DMAChannel dma_gate_toggle; //dma for gate toggling
        DMAChannel dma_env; //dma for envelope on DAC1
        DMAChannel dma_div; //dma for dma trigger divider

        //sources and destination for DMAs
        uint16_t lut_seq[MAX_SEQ_SIZE]; //lut for CV sequence [SRC]
        uint16_t lut_env[MAX_ENV_SIZE]; //lut for envelope shape [SRC]
        uint32_t toggle = 1 << 1; //to write to toggle register of GPIO [SRC]
        volatile byte dummy = 0xFF; //dummy byte for delay divider DMA [SRC, DEST]
        volatile uint32_t &GPIO_ADDR = GPIOC_PTOR; //toggle register for port C [DEST]
        ADC adc;
        //other
        uint8_t initialized;


};

#endif