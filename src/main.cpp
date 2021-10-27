//includes
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

#include <DMAChannel.h>


//macros
#define DAC_TOP (4095)

#define SEQ_SIZE (16) //number of steps in sequence
#define ENV_SIZE (2048) //resolution of envelope

#define GATE1_PIN (22) 
#define GATE2_PIN (23)

#define BPM (120) //target BPM
#define BPS (BPM/60) 

//PDB config: software trigger, enable, interrupt enable, continuous mode, enable DMA triggering
#define PDB_CONFIG                                                   \
    (PDB_SC_TRGSEL(15) | PDB_SC_PDBEN | PDB_SC_PDBIE | PDB_SC_CONT | \
     PDB_SC_DMAEN)

//useful envelope indexes 
#define PRESSED_IDX (0)
#define RELEASED_IDX (ENV_SIZE/2)

//debug
#define ATTACH_ISR (1)


int t0 = millis();

//DMAs
DMAChannel dma_seq(true); //dma for CV sequence on DAC0
DMAChannel dma_gate_toggle(true); //dma for gate toggling
DMAChannel dma_env(true); //dma for envelope on DAC1
DMAChannel dma_div(true); //dma for dma trigger divider

// sources and destination for DMAs
uint16_t lut_seq[SEQ_SIZE]; //lut for CV sequence [SRC]
uint16_t lut_env[ENV_SIZE]; //lut for envelope shape [SRC]
uint32_t toggle = 1 << 1; //to write to toggle register of GPIO [SRC]
volatile byte dummy = 0xFF; //dummy byte for delay divider DMA [SRC, DEST]
volatile uint32_t &GPIO_ADDR = GPIOC_PTOR; //toggle register for port C [DEST]





void __custom_isr__(){
  Serial.println("change!\r\n");
}

void pins_setup(){
  pinMode(GATE1_PIN, OUTPUT); //PORTC 1
  pinMode(GATE2_PIN, OUTPUT); //PORTC 2
  //need initial states for pins because we are toggling them
  digitalWrite(GATE1_PIN, 1);
  digitalWrite(GATE2_PIN, 1);

  #if ATTACH_ISR
  attachInterrupt(digitalPinToInterrupt(GATE1_PIN), __custom_isr__, CHANGE);
  attachInterrupt(digitalPinToInterrupt(GATE2_PIN), __custom_isr__, CHANGE);
  #endif

}

uint8_t dac_setup(uint8_t channel) {
  // Set clock gates for DACs
  SIM_SCGC2 |= SIM_SCGC2_DAC0 | SIM_SCGC2_DAC1;

  // Enable selected channel
  if (channel == (uint8_t)0)
      DAC0_C0 = DAC_C0_DACEN | DAC_C0_DACRFS;
  else if (channel == (uint8_t)1)
      DAC1_C0 = DAC_C0_DACEN | DAC_C0_DACRFS;
  else
      return 1;

  return 0;
}

/* LUT setup functions*/
void lut_seq_setup(){
  for(int iCV = 0; iCV < SEQ_SIZE; iCV++){
    lut_seq[iCV] = iCV*((DAC_TOP/SEQ_SIZE)); //need to find a more interesting sequence
  }

}

//simulate ADSR envelope
uint8_t lut_env_setup(uint16_t attack, uint16_t decay, uint16_t sustain, uint16_t release){

  if(sustain > DAC_TOP){
    return 1;
  }
  if(attack == 0 || decay ==0  || release == 0){
    return 2;
  }


  unsigned int lut_env_idx = 1;
  uint16_t lut_env_val = 0;
  uint16_t attack_slope = (DAC_TOP/attack);
  uint16_t decay_slope = ((DAC_TOP-sustain)/decay);

  /*gate pressed*/
  //attack
  for(; lut_env_idx < attack && lut_env_idx < RELEASED_IDX; lut_env_idx++){
    if(lut_env_val + attack_slope > DAC_TOP){
      break; // we have gone "above" dac top
    }
    lut_env_val += attack_slope;
    lut_env[lut_env_idx] = lut_env_val;
  }
  //decay
  for(; lut_env_idx < (attack+decay) && lut_env_idx < RELEASED_IDX; lut_env_idx++){
    if(lut_env_val - decay_slope > DAC_TOP || (lut_env_val - decay_slope) < sustain){
      lut_env[lut_env_idx] = sustain;
      break; // we have gone "below" 0
    }
    lut_env_val -= decay_slope;
    lut_env[lut_env_idx] = lut_env_val;
  }
  //sustain
  for(; lut_env_idx < RELEASED_IDX; lut_env_idx++){
    lut_env_val = sustain;
    lut_env[lut_env_idx] = lut_env_val;
  }
  /*gate relased*/
  //release
  uint16_t release_slope = ((lut_env_val)/release);

  for(; lut_env_idx < RELEASED_IDX+release && lut_env_idx < ENV_SIZE; lut_env_idx++){
    if(lut_env_val - release_slope > DAC_TOP){
      lut_env[lut_env_idx] = 0;
      break; // we have gone "below" 0
    }
    lut_env_val -= release_slope;
    lut_env[lut_env_idx] = lut_env_val;
  }
  //rest
  for(; lut_env_idx < ENV_SIZE; lut_env_idx++){
    lut_env[lut_env_idx] = 0;
  }

    

  return 0;
}

/* DMA setup functions */
//lut_seq -> DAC0
void dma_seq_setup() {
    dma_seq.disable();
    dma_seq.sourceBuffer(lut_seq, SEQ_SIZE * sizeof(uint16_t));
    dma_seq.transferSize(sizeof(uint16_t));
    dma_seq.transferCount(SEQ_SIZE);
    dma_seq.destination(*(volatile uint16_t *)&(DAC0_DAT0L));
    dma_seq.triggerAtCompletionOf(dma_env);
    dma_seq.enable();
}

//lut_env -> DAC1
void dma_env_setup(){
  dma_env.disable();
  dma_env.sourceBuffer(lut_env, ENV_SIZE * sizeof(uint16_t));
  dma_env.transferSize(sizeof(uint16_t));
  dma_env.destination(*(volatile uint16_t *)&(DAC1_DAT0L));
  dma_env.transferCount(ENV_SIZE);
  dma_env.triggerAtHardwareEvent(DMAMUX_SOURCE_PDB);
  dma_env.enable();
}

//toggle -> port C pin
void dma_gate_setup() {
    dma_gate_toggle.disable();
    dma_gate_toggle.source(*(volatile uint32_t*)&toggle);
    dma_gate_toggle.destination(*(volatile uint32_t *)&(GPIO_ADDR));  
    dma_gate_toggle.transferSize(sizeof(uint32_t));
    dma_gate_toggle.transferCount(1);
    dma_gate_toggle.triggerAtCompletionOf(dma_div);
    dma_gate_toggle.enable();
}

//dummy -> dummy
void dma_div_setup(uint8_t num_beats){
  dma_div.disable();
  dma_div.source(*(volatile uint8_t*) &dummy);
  dma_div.transferSize(sizeof(dummy));
  dma_div.destination(*(volatile uint8_t*)&dummy);
  dma_div.transferCount(ENV_SIZE/(num_beats));
  dma_div.triggerAtHardwareEvent(DMAMUX_SOURCE_PDB);
  dma_div.enable();
}

/* PDB */
void trigger_clock_setup() {
    SIM_SCGC6 |= SIM_SCGC6_PDB;  // Enable PDB clock

    uint32_t mod = F_BUS / (BPS*ENV_SIZE); //PDB is essentially a counter

    PDB0_MOD = (uint16_t)(mod - 1);      // Counter between PDB activations
    PDB0_IDLY = 0x0;                     // PDB interrupts delay
    PDB0_SC = PDB_CONFIG | PDB_SC_LDOK;  // Load configuration
    PDB0_SC = PDB_CONFIG | PDB_SC_SWTRIG;
    PDB0_CH0C1 = 0x0101;  // Enable pre-trigger
} 


void setup() {
  //pins
  pins_setup();

  //DACs
  dac_setup(0); 
  dac_setup(1); 
  //LUTs
  lut_env_setup(512,256,2048,512);
  lut_seq_setup();
  //DMAs
  dma_gate_setup();
  dma_seq_setup();
  dma_env_setup();
  dma_div_setup(1);
  //start
  trigger_clock_setup();
}


void loop() {
    if(millis() - t0 > 500){
      t0 = millis();
      Serial.println(dma_div.error());
   }
}