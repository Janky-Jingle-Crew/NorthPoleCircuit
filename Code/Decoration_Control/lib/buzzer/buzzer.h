#pragma once

#include "funconfig.h"
#include "stdint.h"
#include "ch32v003fun.h"
#include "stdbool.h"


#define TIM2_DEFAULT 0xff
#define PRESCALE_DIV4 0x03

unsigned int freqz[] = {
  0,
  261, 277, 293, 311, 329, 349, 369, 391, 415, 440, 466, 493,
  523, 554, 587, 622, 659, 698, 739, 783, 830, 880, 932, 987
};

typedef struct
{
    uint8_t note;
    uint16_t duration;
} note_tt;

const note_tt last_christmas[] = {{0,100},{17,737},{0,64},{17,438},{0,93},{15,108},{0,427},{10,60},{0,206},{17,193},{0,73},{17,202},{0,65},{19,161},{0,107},{15,578},{0,222},{12,191},{0,75},{12,165},{0,97},{17,214},{0,51},{17,198},{0,70},{19,180},{0,354},{15,571},{0,225},{12,64},{0,204},{14,247},{0,20},{15,266},{0,0},{14,208},{0,60},{12,880},{0,455},{19,681},{0,120},{17,806},{0,261},{12,77},{0,191},{19,197},{0,66},{20,194},{0,72},{19,166},{0,101},{17,697},{0,376},{15,63},{0,202},{14,246},{0,21},{15,207},{0,58},{14,63},{0,207},{14,412},{0,120},{15,90},{0,446},{14,218},{0,318},{10,936}, {0,1000},{0,1000}};
const note_tt jingle[] = {{0,100},{11,105},{0,233},{11,1},{11,1},{0,166},{11,121},{0,217},{11,113},{0,55},{10,90},{0,248},{10,66},{0,103},{10,87},{0,251},{10,84},{0,84},{8,309},{0,29},{10,108},{0,60},{8,143},{0,195},{3,381},{0,127},{1,129},{0,39},{1,92},{0,76},{4,63},{0,105},{6,74},{0,95},{8,198},{0,140},{10,71},{0,97},{8,158},{0,180},{3,540},{0,137},{6,360},{0,148},{8,328},{0,10},{10,79},{0,90},{8,119},{0,219},{4,423},{0,593},{4,116},{0,52},{1,137},{0,201},{3,76},{0,92},{4,95},{0,243},{6,354},{0,153},{8,338},{0,169},{6,121},{0,47},{1,100},{0,238},{3,74},{0,95},{4,105},{0,233},{6,423},{0,254},{6,254},{0,84},{6,84},{0,84},{8,227},{0,280},{8,296},{0,211},{10,119},{0,219},{8,111},{0,58},{6,105},{0,233},{11,582},{0,1000},{0,1000},};
const note_tt i_want[] = {{0,100},{1,428},{0,0},{5,428},{0,0},{8,428},{0,0},{12,214},{0,0},{13,428},{0,0},{12,428},{0,0},{10,428},{0,0},{8,642},{0,0},{15,428},{0,0},{13,428},{0,0},{13,214},{0,0},{12,428},{0,0},{13,428},{0,0},{12,428},{0,0},{10,214},{0,0},{8,857},{0,428},{10,428},{0,0},{13,428},{0,0},{15,214},{0,0},{17,428},{0,0},{15,428},{0,0},{13,428},{0,0},{10,642},{0,0},{6,428},{0,0},{9,214},{0,0},{13,642},{0,0},{15,214},{0,0},{16,428},{0,0},{15,428},{0,0},{11,428},{0,0},{9,642},{0,0},{13,428},{0,0},{15,428},{0,0},{12,428},{0,0},{13,214},{0,0},{10,428},{0,0},{12,428},{0,0},{9,1071},{0,0},{13,428},{0,0},{15,428},{0,0},{12,428},{0,0},{13,214},{0,0},{10,428},{0,0},{12,428},{0,0},{9,1071},{0,0},{8,428},{0,0},{10,428},{0,0},{13,214},{0,0},{20,428},{0,0},{18,428},{0,0},{20,214},{0,0},{18,428},{0,428},{20,107},{0,107},{20,214},{0,0},{17,428},{0,0},{15,428},{0,0},{13,428},{0,0},{10,428},{0,0},{9,428},{0,0},{15,857},{0,0},{17,428},{0,0},{13,2142},{0,1000}};
const note_tt rockin[] = {{0,100},{14,260},{0,17},{14,121},{0,17},{14,399},{0,17},{16,399},{0,17},{16,399},{0,17},{14,399},{0,17},{14,260},{0,17},{16,538},{0,17},{14,260},{0,17},{14,121},{0,17},{18,399},{0,17},{16,399},{0,17},{12,399},{0,17},{9,260},{0,17},{6,243},{0,312},{2,121},{0,156},{2,121},{0,17},{1,260},{0,17},{2,538},{0,17},{16,260},{0,17},{16,121},{0,17},{16,399},{0,17},{18,399},{0,17},{18,399},{0,17},{16,399},{0,17},{16,260},{0,17},{18,538},{0,17},{16,260},{0,17},{16,121},{0,17},{14,399},{0,17},{15,399},{0,17},{16,399},{0,17},{12,399},{0,17},{11,208},{0,104},{7,121},{0,156},{7,121},{0,17},{6,260},{0,17},{7,538},{0,121},{14,260},{0,17},{14,121},{0,17},{14,399},{0,17},{16,399},{0,17},{16,399},{0,17},{14,399},{0,17},{14,260},{0,17},{16,538},{0,17},{14,260},{0,17},{14,121},{0,17},{18,399},{0,17},{16,399},{0,17},{12,399},{0,17},{9,260},{0,17},{6,243},{0,312},{2,121},{0,156},{2,121},{0,17},{1,260},{0,17},{2,538},{0,17},{16,260},{0,17},{16,121},{0,17},{16,399},{0,17},{18,399},{0,17},{18,399},{0,17},{16,399},{0,17},{16,260},{0,17},{18,538},{0,17},{16,260},{0,17},{16,121},{0,17},{14,399},{0,17},{15,399},{0,17},{16,399},{0,17},{6,399},{0,17},{7,1145},{0,1000}};
const note_tt feliz[] = {{0,100}, {8,196},{0,10},{13,284},{0,129},{12,191},{0,15},{13,202},{0,4},{10,385},{0,28},{8,56},{0,150},{6,64},{0,142},{5,56},{0,150},{1,62},{0,351},{8,206},{0,620},{10,191},{0,15},{15,269},{0,144},{13,174},{0,32},{10,206},{0,0},{8,381},{0,32},{8,60},{0,146},{6,68},{0,137},{5,64},{0,142},{8,68},{0,344},{1,206},{0,620},{8,206},{0,0},{13,262},{0,150},{12,185},{0,21},{13,206},{0,0},{10,450},{0,170},{6,153},{0,53},{10,280},{0,133},{10,185},{0,21},{8,237},{0,176},{8,219},{0,193},{8,183},{0,23},{6,254},{0,159},{6,206},{0,0},{5,413},{0, 1000}, {0,1000}};
const note_tt wish_merry[] = {{0,100},{3,394},{0,0},{8,197},{0,197},{8,193},{0,4},{10,197},{0,0},{8,197},{0,0},{7,185},{0,12},{5,234},{0,160},{5,201},{0,193},{5,361},{0,32},{10,197},{0,197},{10,189},{0,8},{12,197},{0,0},{10,197},{0,0},{8,197},{0,0},{7,394},{0,0},{3,189},{0,205},{3,275},{0,119},{12,164},{0,230},{12,197},{0,0},{13,197},{0,0},{12,197},{0,0},{10,185},{0,12},{8,337},{0,57},{5,283},{0,111},{3,106},{0,90},{3,185},{0,12},{5,341},{0,53},{10,304},{0,90},{7,234},{0,160},{8,596},{0,193},{3,378},{0,16},{8,271},{0,123},{8,320},{0,74},{8,312},{0,82},{7,686},{0,102},{7,353},{0,41},{8,337},{0,57},{7,365},{0,28},{5,394},{0,0},{3,670},{0,119},{10,263},{0,131},{12,394},{0,0},{10,127},{0,69},{10,185},{0,12},{8,131},{0,65},{8,197},{0,0},{15,357},{0,37},{3,263},{0,131},{3,119},{0,78},{3,160},{0,37},{5,378},{0,16},{10,349},{0,45},{7,312},{0,82},{8,629},{0,1000}};
const note_tt silent[] = {{0,100},{8,768},{0,7},{10,166},{0,10},{8,497},{5,1406},{0,127},{8,812},{0,15},{10,171},{0,7},{8,486},{0,2},{5,1156},{0,346},{15,937},{0,39},{15,502},{0,23},{12,1497},{0,0},{13,976},{0,33},{13,476},{0,18},{8,489},{0,10},{6,489},{0,10},{8,489},{0,23},{10,958},{0,18},{10,440},{0,62},{13,750},{0,59},{12,182},{0,2},{10,442},{0,49},{8,825},{0,10},{10,114},{0,57},{8,505},{0,0},{5,1499},{0,2},{10,966},{0,57},{10,411},{0,52},{13,760},{0,59},{12,177},{0,2},{10,447},{0,49},{8,763},{0,2},{10,247},{8,497},{0,0},{5,1500},{15,953},{0,44},{15,505},{0,2},{18,786},{0,7},{15,182},{0,20},{12,500},{13,1502},{0,26},{17,1473},{0,13},{13,500},{0,7},{8,473},{0,5},{5,500},{0,0},{8,820},{0,0},{6,179},{0,13},{3,486},{0,2},{1,1666},{0, 1000}, {0,1000}};
const note_tt funky[] = {{0,100},{13, 99}, {0, 251}, {13, 137}, {0, 213}, {11, 140}, {0, 210}, {13, 128}, {0, 573}, {8, 193}, {0, 509}, {8, 169}, {0, 181}, {13, 152}, {0, 199}, {18, 172}, {0, 178}, {17, 137}, {0, 213}, {13, 143}, {0, 1612}, {13, 102}, {0, 248}, {13, 125}, {0, 225}, {11, 134}, {0, 216}, {13, 128}, {0, 573}, {8, 169}, {0, 532}, {8, 146}, {0, 204}, {13, 149}, {0, 201}, {18, 178}, {0, 172}, {17, 155}, {0, 196}, {13, 146}, {0, 1610}, {1, 172}, {0, 178}, {1, 172}, {0, 178}, {1, 172}, {0, 178}, {1, 348}, {0, 2}, {5, 524}, {0, 178}, {5, 172}, {0, 178}, {5, 172}, {0, 178}, {8, 172}, {0, 178}, {8, 172}, {0, 178}, {8, 348}, {0, 2}, {17, 524}, {0, 178}, {15, 172}, {0, 178}, {13, 351}, {0, 351}, {13, 96}, {0, 254}, {13, 131}, {0, 219}, {11, 140}, {0, 210}, {13, 125}, {0, 576}, {8, 152}, {0, 550}, {8, 161}, {0, 190}, {13, 140}, {0, 210}, {18, 166}, {0, 184}, {17, 155}, {0, 196}, {13, 146},{0,1000}};
const note_tt groove[] = {{0,100},{1,666},{0,128},{4,72},{0,201},{6,436},{0,106},{4,442},{0,95},{3,604},{0,190},{8,408},{0,140},{6,106},{0,156},{8,364},{0,179},{1,666},{0,128},{4,72},{0,201},{6,436},{0,106},{4,442},{0,95},{3,604},{0,190},{8,408},{0,140},{6,106},{0,156},{8,364},{0,173},{13,420},{0,649},{8,263},{0,16},{11,240},{0,28},{13,263},{0,0},{15,190},{0,352},{11,145},{0,392},{6,397},{0,128},{8,252},{0,16},{11,235},{0,33},{11,173},{0,638},{20,246},{0,28},{18,756},{0,39},{16,257},{0,22},{15,778},{0,16},{22,263},{0,16},{20,800},{0,806},{22,84},{0,184},{22,196},{0,336},{8,263},{0,16},{11,240},{0,28},{13,263},{0,0},{15,184},{0,358},{11,145},{0,392},{6,528},{0,0},{8,252},{0,16},{11,235},{0,33},{11,173},{0,638},{20,246},{0,28},{18,756},{0,39},{16,257},{0,22},{15,778},{0,16},{22,263},{0,16},{20,800},{0,1000}};
const note_tt rickroll[] = {{0,100},{6,824},{0,75},{8,825},{0,74},{1,550},{0,49},{8,825},{0,74},{10,825},{0,74},{13,137},{0,12},{11,137},{0,12},{10,137},{0,12},{6,975},{0,74},{8,825},{0,75},{1,2024},{0,375},{13,137},{0,12},{11,137},{0,12},{10,137},{0,12},{6,974},{0,75},{8,825},{0,75},{1,549},{0,49},{8,825},{0,75},{10,825},{0,75},{13,137},{0,12},{11,137},{0,12},{10,137},{0,12},{6,975},{0,75},{8,824},{0,75},{1,2250},{0,150},{1,137},{0,12},{3,137},{0,12},{6,137},{0,12},{3,137},{0,12},{10,412},{0,187},{10,274},{0,25},{8,849},{0,49},{1,137},{0,12},{3,137},{0,12},{6,137},{0,12},{3,137},{0,12},{8,412},{0,187},{8,275},{0,24},{6,412},{0,37},{5,137},{0,12},{3,274},{0,25},{1,137},{0,12},{3,137},{0,12},{6,137},{0,12},{3,137},{0,12},{6,549},{0,49},{8,275},{0,24},{5,412},{0,37},{3,137},{0,12},{1,375},{0,224},{1,262},{0,37},{8,549},{0,50},{6,1149},{0,49},{1,137},{0,12},{3,137},{0,12},{6,137},{0,12},{3,137},{0,12},{10,412},{0,187},{10,275},{0,24},{8,850},{0,49},{1,137},{0,12},{3,137},{0,12},{6,137},{0,12},{3,137},{0,12},{13,550},{0,49},{5,274},{0,25},{6,412},{0,37},{5,137},{0,12},{3,275},{0,25},{1,137},{0,12},{3,137},{0,12},{6,137},{0,12},{3,137},{0,12},{6,549},{0,49},{8,275},{0,25},{5,412},{0,37},{3,137},{0,12},{1,412},{0,187},{1,275},{0,25},{8,549},{0,49},{6,1299},{0,1000}};
const note_tt santa_down[] = {{0,300},{1,103},{0,25},{10,97},{0,1173},{10,97},{0,25},{1,103},{0,10000},{0,1000}};

const note_tt * songs[] = {
  last_christmas,
  wish_merry,
  silent,
  jingle,
  feliz,
  santa_down,
  //i_want,
  //rockin,
};


#define NOTE_SIZE (sizeof(note_tt))

const uint8_t song_lengths[] = {
  sizeof(last_christmas)/NOTE_SIZE,    // jumping santa
  sizeof(wish_merry)/NOTE_SIZE,        // Snowman
  sizeof(silent)/NOTE_SIZE,            // Reindeer
  sizeof(jingle)/NOTE_SIZE,            // present
  sizeof(feliz)/NOTE_SIZE,             // Tree
  sizeof(santa_down)/NOTE_SIZE,        // Santa down
  //sizeof(i_want)/NOTE_SIZE,
  //sizeof(rockin)/NOTE_SIZE,
};

typedef struct {
    const note_tt * curr_song;
    uint8_t curr_song_length;
    uint16_t note_index;
    bool playing;
} music_state_t;


void _t2pwm_init(void)
{
    // Enable TIM2
    RCC->APB1PCENR |= RCC_APB1Periph_TIM2;

    // If using T2CH3 on D2 must also enable GPIOC
    RCC->APB2PCENR |= RCC_APB2Periph_GPIOD;

    // PD2 is T2CH3, 10MHz Output alt func, push-pull
    GPIOD->CFGLR &= ~(0xf << (4 * 2));
    GPIOD->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF) << (4 * 2);

    // Reset TIM2 to init all regs
    RCC->APB1PRSTR |= RCC_APB1Periph_TIM2;
    RCC->APB1PRSTR &= ~RCC_APB1Periph_TIM2;

    // TIM2 pin mux
    AFIO->PCFR1 |= ((0b01) << 8);

    // SMCFGR: default clk input is CK_INT
    // set TIM2 clock prescaler divider
    TIM2->PSC = PRESCALE_DIV4;
    // set PWM total cycle width
    TIM2->ATRLR = 1023;
    TIM2->CH3CVR = 1024;

    // for channel 1 and 2, let CCxS stay 00 (output), set OCxM to 110 (PWM I)
    // enabling preload causes the new pulse width in compare capture register only to come into effect when UG bit in SWEVGR is set (= initiate update) (auto-clears)
    TIM2->CHCTLR2 |= TIM_OC3M_2 | TIM_OC3M_1 | TIM_OC3PE;

    // CTLR1: default is up, events generated, edge align
    // enable auto-reload of preload
    TIM2->CTLR1 |= TIM_ARPE;

    // Enable Channel outputs, set default state (based on TIM2_DEFAULT)
    TIM2->CCER |= TIM_CC3E | (TIM_CC3P & TIM2_DEFAULT);

    // initialize counter
    TIM2->SWEVGR |= TIM_UG;

    // Enable TIM2
    //TIM2->CTLR1 |= TIM_CEN;
}


void _tone(uint32_t freq) {
    // ATRLR = clock/(freqz*prescaler)
    if(freq == 0){
        // If timer period (ATRLR) < compare value (CH3CVR), then PWM = 0
        TIM2->ATRLR = (uint16_t) 1023;
        TIM2->CH3CVR = (uint16_t) 1024;

        // Initiate timer update
        TIM2->SWEVGR |= TIM_UG;
        TIM2->CCER &= ~(TIM_CC3E | (TIM_CC3P & TIM2_DEFAULT));
        return;
    }

    uint16_t period = FUNCONF_SYSTEM_CORE_CLOCK / (freq * (PRESCALE_DIV4 + 1));
    TIM2->ATRLR = (uint16_t) period;

    // Set compare value to half the period for 50% duty cycle
    TIM2->CH3CVR = (uint16_t) period / 2;

    // Initiate timer update     
    TIM2->SWEVGR |= TIM_UG;
    TIM2->CCER |= TIM_CC3E | (TIM_CC3P & TIM2_DEFAULT);
}


void music_off(music_state_t * state) {
    state->playing = false;
    Delay_Ms(2);
    TIM2->ATRLR = (uint16_t) 1023;
    TIM2->CH3CVR = (uint16_t) 1024;
    TIM2->SWEVGR |= TIM_UG;
    TIM2->CCER &= ~(TIM_CC3E | (TIM_CC3P & TIM2_DEFAULT));
    TIM2->CTLR1 &= ~TIM_CEN;
}

void music_on(music_state_t * state) {
    state->playing = true;
    state->note_index = 0;
    //ms_cnt = 0;
    TIM2->CCER |= TIM_CC3E | (TIM_CC3P & TIM2_DEFAULT);
    TIM2->CTLR1 |= TIM_CEN;

}

void music_init(music_state_t * state){
    _t2pwm_init();

    state->playing = 0;
    state->note_index = 0;
    state->curr_song = songs[0];
    state->curr_song_length = song_lengths[0];

    music_off(state);

}

void music_change_song(music_state_t * state, uint8_t song_idx) {
    //music_off();

    // Check index in range
    if(song_idx >= (sizeof(songs)/sizeof(songs[0]))){ return; }
    state->curr_song = songs[song_idx];
    state->curr_song_length = song_lengths[song_idx];
    state->note_index = 0;
    //ms_cnt = 0;
    //music_on();
}

// Play next note
// returns: note number (0-24) of the new note, or -1 if end of song is reached
int8_t music_next_note(music_state_t * state){
    state->note_index++;
    uint16_t i = state->note_index;

    if(i >= state->curr_song_length){

        music_off(state);
        return -1;
    }

    uint8_t note = state->curr_song[i].note;

    _tone(freqz[note]);

    return note;
}

uint16_t music_get_current_note_duration(const music_state_t * state){

    return state->curr_song[state->note_index].duration;

}
