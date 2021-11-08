//includes
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include "sequencer.h"






/*void __custom_isr__(){
  Serial.println("change!\r\n");
}*/

void Sequencer::pins_setup(){
  pinMode(GATE1_PIN, OUTPUT); //PORTC 1
  pinMode(GATE2_PIN, OUTPUT); //PORTC 2
  //need initial states for pins because we are toggling them
  digitalWrite(GATE1_PIN, 1);
  digitalWrite(GATE2_PIN, 1);

  #if ATTACH_ISR
  //attachInterrupt(digitalPinToInterrupt(GATE1_PIN), __custom_isr__, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(GATE2_PIN), __custom_isr__, CHANGE);
  #endif

  

}

uint8_t Sequencer::dac_setup(uint8_t channel) {
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
void Sequencer::lut_seq_setup(uint16_t seq_size){
  for(int iCV = 0; iCV < seq_size; iCV++){
    lut_seq[iCV] = iCV*((DAC_TOP/seq_size)); //need to find a more interesting sequence
  } 

}

//simulate ADSR envelope
uint8_t Sequencer::lut_env_setup(uint16_t attack, uint16_t decay, uint16_t sustain, uint16_t release, uint16_t env_size){

  if(sustain > DAC_TOP){
    return 1;
  }
  if(attack == 0 || decay ==0  || release == 0){
    return 2;
  }
    uint16_t released_idx = env_size/2;



  unsigned int lut_env_idx = 1;
  uint16_t lut_env_val = 0;
  uint16_t attack_slope = (DAC_TOP/attack);
  uint16_t decay_slope = ((DAC_TOP-sustain)/decay);

  /*gate pressed*/
  //attack
  for(; lut_env_idx < attack && lut_env_idx < released_idx; lut_env_idx++){
    if(lut_env_val + attack_slope > DAC_TOP){
      break; // we have gone "above" dac top
    }
    lut_env_val += attack_slope;
    lut_env[lut_env_idx] = lut_env_val;
  }
  //decay
  for(; lut_env_idx < (attack+decay) && lut_env_idx < released_idx; lut_env_idx++){
    if(lut_env_val - decay_slope > DAC_TOP || (lut_env_val - decay_slope) < sustain){
      lut_env[lut_env_idx] = sustain;
      break; // we have gone "below" 0
    }
    lut_env_val -= decay_slope;
    lut_env[lut_env_idx] = lut_env_val;
  }
  //sustain
  for(; lut_env_idx < released_idx; lut_env_idx++){
    lut_env_val = sustain;
    lut_env[lut_env_idx] = lut_env_val;
  }
  /*gate relased*/
  //release
  uint16_t release_slope = ((lut_env_val)/release);

  for(; lut_env_idx < released_idx+release && lut_env_idx < env_size; lut_env_idx++){
    if(lut_env_val - release_slope > DAC_TOP){
      lut_env[lut_env_idx] = 0;
      break; // we have gone "below" 0
    }
    lut_env_val -= release_slope;
    lut_env[lut_env_idx] = lut_env_val;
  }
  //rest
  for(; lut_env_idx < env_size; lut_env_idx++){
    lut_env[lut_env_idx] = 0;
  }

    

  return 0;
}

/* DMA setup functions */
//lut_seq -> DAC0
void Sequencer::dma_seq_setup(uint16_t seq_size) {
    dma_seq.disable();
    dma_seq.sourceBuffer(lut_seq, seq_size * sizeof(uint16_t));
    dma_seq.transferSize(sizeof(uint16_t));
    dma_seq.transferCount(seq_size);
    dma_seq.destination(*(volatile uint16_t *)&(DAC0_DAT0L));
    dma_seq.triggerAtCompletionOf(dma_env);
    dma_seq.enable();
}

//lut_env -> DAC1
void Sequencer::dma_env_setup(uint16_t env_size){
  dma_env.disable();
  dma_env.sourceBuffer(lut_env, env_size * sizeof(uint16_t));
  dma_env.transferSize(sizeof(uint16_t));
  dma_env.destination(*(volatile uint16_t *)&(DAC1_DAT0L));
  dma_env.transferCount(env_size);
  dma_env.triggerAtHardwareEvent(DMAMUX_SOURCE_PDB);
  dma_env.enable();
}

//toggle -> port C pin
void Sequencer::dma_gate_setup() {
    dma_gate_toggle.disable();
    dma_gate_toggle.source(*(volatile uint32_t*)&toggle);
    dma_gate_toggle.destination(*(volatile uint32_t *)&(GPIO_ADDR));  
    dma_gate_toggle.transferSize(sizeof(uint32_t));
    dma_gate_toggle.transferCount(1);
    dma_gate_toggle.triggerAtCompletionOf(dma_div);
    dma_gate_toggle.enable();
}

//dummy -> dummy
void Sequencer::dma_div_setup(uint8_t num_beats, uint16_t env_size){
  dma_div.disable();
  dma_div.source(*(volatile uint8_t*) &dummy);
  dma_div.transferSize(sizeof(dummy));
  dma_div.destination(*(volatile uint8_t*)&dummy);
  dma_div.transferCount(env_size/(num_beats));
  dma_div.triggerAtHardwareEvent(DMAMUX_SOURCE_PDB);
  dma_div.enable();
}

/* PDB */
void Sequencer::trigger_clock_setup(uint16_t bps, uint16_t env_size) {
    SIM_SCGC6 |= SIM_SCGC6_PDB;  // Enable PDB clock

    uint32_t mod = F_BUS / (bps*env_size); //PDB is essentially a counter

    PDB0_MOD = (uint16_t)(mod - 1);      // Counter between PDB activations
    PDB0_IDLY = 0x0;                     // PDB interrupts delay
    PDB0_SC = PDB_CONFIG | PDB_SC_LDOK;  // Load configuration


}

Sequencer::Sequencer(){
  initialized = 0;
}



int Sequencer::init(uint16_t sequence_size, uint16_t envelope_res, uint16_t bpm){

  if(sequence_size > MAX_SEQ_SIZE){
    return -1;
  }
  if(envelope_res > MAX_ENV_SIZE){
    return -2;
  }


  dma_env.begin();
  dma_seq.begin();
  dma_div.begin();
  dma_gate_toggle.begin();
  
  this->pins_setup();
  this->dac_setup(0);
  this->dac_setup(1);

  this->lut_env_setup(512, 256, 2048, 512, envelope_res);
  this->lut_seq_setup(sequence_size);
  
  this->dma_gate_setup();
  this->dma_seq_setup(sequence_size);
  this->dma_env_setup(envelope_res);
  this->dma_div_setup(1, envelope_res);

  this->trigger_clock_setup((bpm/60), envelope_res);

  initialized = 1;
  return 0;

}
int Sequencer::run(){
  if(!initialized){
    return -1;
  }
  PDB0_SC = PDB_CONFIG | PDB_SC_SWTRIG;
  PDB0_CH0C1 = 0x0101;  // Enable pre-trigger
  return 0;

}
int Sequencer::stop(){
  return 0;
 
}